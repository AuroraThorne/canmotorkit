"""CAN 总线抽象：PcanBus（Windows）、SocketCanBus（Linux）与 SimBus。"""
from __future__ import annotations

import time
from dataclasses import dataclass



@dataclass
class Frame:
    t: float            # 主机时间 perf_counter
    can_id: int
    data: bytes
    tx: bool = False
    extended: bool = False
    rtr: bool = False
    dlc: int | None = None

    @property
    def wire_dlc(self) -> int:
        """报文在线上的 DLC；RTR 帧不能从 data 长度推断。"""
        return len(self.data) if self.dlc is None else self.dlc


class CanBus:
    name = "base"

    def open(self) -> None: ...
    def close(self) -> None: ...
    def send(self, can_id: int, data: bytes, extended: bool = False,
             rtr: bool = False, dlc: int | None = None) -> None: ...
    def recv_all(self) -> list[Frame]: return []
    def status_text(self) -> str: return "OK"


class BusError(RuntimeError):
    pass


# ====================================================================
class PcanBus(CanBus):
    def __init__(self, channel: int, baud: int):
        from .pcan_basic import PCANBasic
        self.api = PCANBasic()
        self.ch = channel
        self.baud = baud
        self.name = f"PCAN 0x{channel:X}"
        self._open = False
        # PCAN 返回的时间戳是设备接收时间；保存一个映射到 perf_counter 的
        # 原点，避免批量读取接收队列时把所有帧都标成“现在”。
        self._pcan_ts0: int | None = None
        self._perf_ts0: float | None = None

    def open(self) -> None:
        r = self.api.initialize(self.ch, self.baud)
        if r != 0:
            raise BusError(f"打开 PCAN 失败: {self.api.error_text(r)}")
        self._open = True
        self._pcan_ts0 = None
        self._perf_ts0 = None

    def close(self) -> None:
        if self._open:
            self.api.uninitialize(self.ch)
            self._open = False
        self._pcan_ts0 = None
        self._perf_ts0 = None

    def _frame_time(self, ts) -> float:
        """把 PCAN 的毫秒+微秒时间戳映射到本进程的 perf_counter。"""
        # millis_overflow 是 millis 的高 32 位；micros 是当前毫秒内的
        # 微秒部分。PCAN 时间戳在初始化后单调递增，适合用于帧间隔计算。
        raw_us = (((int(ts.millis_overflow) << 32) + int(ts.millis)) * 1000
                  + int(ts.micros))
        now = time.perf_counter()
        if self._pcan_ts0 is None:
            self._pcan_ts0 = raw_us
            self._perf_ts0 = now
        return (self._perf_ts0 or now) + (raw_us - self._pcan_ts0) / 1_000_000.0

    def send(self, can_id: int, data: bytes, extended: bool = False,
             rtr: bool = False, dlc: int | None = None) -> None:
        r = self.api.write(self.ch, can_id, data, extended=extended,
                           rtr=rtr, dlc=dlc)
        if r != 0:
            raise BusError(f"发送失败: {self.api.error_text(r)}")

    def recv_all(self) -> list[Frame]:
        from .pcan_basic import (PCAN_ERROR_QRCVEMPTY, PCAN_MESSAGE_STATUS,
                                 PCAN_MESSAGE_RTR, PCAN_MESSAGE_EXTENDED,
                                 PCAN_MESSAGE_ERRFRAME)
        out = []
        for _ in range(512):
            r, msg, ts = self.api.read(self.ch)
            if r & PCAN_ERROR_QRCVEMPTY:
                break
            if r != 0:
                break
            if msg.MSGTYPE & (PCAN_MESSAGE_STATUS | PCAN_MESSAGE_ERRFRAME):
                continue
            rtr = bool(msg.MSGTYPE & PCAN_MESSAGE_RTR)
            data = b"" if rtr else bytes(msg.DATA[:msg.LEN])
            out.append(Frame(self._frame_time(ts), msg.ID, data,
                             extended=bool(msg.MSGTYPE & PCAN_MESSAGE_EXTENDED),
                             rtr=rtr, dlc=int(msg.LEN)))
        return out

    def status_text(self) -> str:
        r = self.api.get_status(self.ch)
        return "OK" if r == 0 else self.api.error_text(r)
