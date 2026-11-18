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

    def open(self) -> None:
        r = self.api.initialize(self.ch, self.baud)
        if r != 0:
            raise BusError(f"打开 PCAN 失败: {self.api.error_text(r)}")
        self._open = True

    def close(self) -> None:
        if self._open:
            self.api.uninitialize(self.ch)
            self._open = False

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
            r, msg, _ts = self.api.read(self.ch)
            if r & PCAN_ERROR_QRCVEMPTY:
                break
            if r != 0:
                break
            if msg.MSGTYPE & (PCAN_MESSAGE_STATUS | PCAN_MESSAGE_ERRFRAME):
                continue
            rtr = bool(msg.MSGTYPE & PCAN_MESSAGE_RTR)
            data = b"" if rtr else bytes(msg.DATA[:msg.LEN])
            out.append(Frame(time.perf_counter(), msg.ID, data,
                             extended=bool(msg.MSGTYPE & PCAN_MESSAGE_EXTENDED),
                             rtr=rtr, dlc=int(msg.LEN)))
        return out

    def status_text(self) -> str:
        r = self.api.get_status(self.ch)
        return "OK" if r == 0 else self.api.error_text(r)
