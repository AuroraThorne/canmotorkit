"""CAN 总线抽象：PcanBus（真实硬件）与 SimBus（虚拟 HM 电机，用于无硬件调试）。"""
from __future__ import annotations

import math
import random
import threading
import time
from collections import deque
from dataclasses import dataclass

from ..drivers import dm_protocol as P


@dataclass
class Frame:
    t: float            # 主机时间 perf_counter
    can_id: int
    data: bytes
    tx: bool = False


class CanBus:
    name = "base"

    def open(self) -> None: ...
    def close(self) -> None: ...
    def send(self, can_id: int, data: bytes) -> None: ...
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

    def send(self, can_id: int, data: bytes) -> None:
        r = self.api.write(self.ch, can_id, data)
        if r != 0:
            raise BusError(f"发送失败: {self.api.error_text(r)}")

    def recv_all(self) -> list[Frame]:
        from .pcan_basic import PCAN_ERROR_QRCVEMPTY, PCAN_MESSAGE_STATUS, PCAN_MESSAGE_RTR, PCAN_MESSAGE_ERRFRAME
        out = []
        for _ in range(512):
            r, msg, _ts = self.api.read(self.ch)
            if r & PCAN_ERROR_QRCVEMPTY:
                break
            if r != 0:
                break
            if msg.MSGTYPE & (PCAN_MESSAGE_STATUS | PCAN_MESSAGE_RTR | PCAN_MESSAGE_ERRFRAME):
                continue
            out.append(Frame(time.perf_counter(), msg.ID, bytes(msg.DATA[:msg.LEN])))
        return out

    def status_text(self) -> str:
        r = self.api.get_status(self.ch)
        return "OK" if r == 0 else self.api.error_text(r)


# ====================================================================
class SimMotor:
    """简化的 HM 电机模型：J*a = tau - b*v，按手册协议应答。"""

    def __init__(self, motor_id: int = 1, master_id: int = 0):
        # 实测 HM9816G9: p_max=π（单圈位置），v_max=45，t_max=54
        self.lim = P.DEFAULT_LIMITS.with_maps(p_max=math.pi)
        self.regs: dict[int, float | int] = {
            0x00: 15.0, 0x01: 0.45, 0x02: 100.0, 0x03: 20.0, 0x07: master_id, 0x08: motor_id,
            0x09: 0, 0x0B: 0.02, 0x0C: 0.01, 0x0D: 9816, 0x0E: 106, 0x10: 21, 0x11: 0.121,
            0x12: 70e-6, 0x13: 0.0035, 0x14: 9.0, 0x15: self.lim.p_max, 0x16: self.lim.v_max,
            0x17: self.lim.t_max, 0x18: 1000.0, 0x1D: 40.0, 0x23: 4, 0x32: 0.0, 0x33: 0.0,
            0x3B: 50.0, 0x3C: 24.0, 0x3D: 30.0, 0x3E: 30.0, 0x50: 0.0, 0x51: 0.0,
        }
        self.enabled = False
        self.err = 0
        self.pos = 0.3          # 上电位置稍偏离零点
        self.vel = 0.0
        self.tau = 0.0
        self.offset = 0.0
        self.cmd = P.MitCommand()
        self.temp = 30.0

    @property
    def motor_id(self) -> int:
        return int(self.regs[0x08])

    @property
    def master_id(self) -> int:
        return int(self.regs[0x07])

    def step(self, dt: float) -> None:
        J = float(self.regs[0x0C]) * 3.0 + 0.02   # 含负载
        b = float(self.regs[0x0B])
        if self.enabled and self.err == 0:
            c = self.cmd
            tau = c.kp * (c.p - self.out_pos()) + c.kd * (c.v - self.vel) + c.t
            tau = P.clamp(tau, self.lim.t_min, self.lim.t_max)
        else:
            tau = 0.0
        fric = 0.05 * math.tanh(self.vel * 20)
        acc = (tau - b * self.vel - fric) / J
        self.vel += acc * dt
        self.pos += self.vel * dt
        self.tau = tau
        self.temp += (30.0 + abs(tau) * 0.8 - self.temp) * dt * 0.05
        self.regs[0x3D] = self.temp
        self.regs[0x3E] = self.temp - 2
        self.regs[0x51] = self.pos - self.offset
        self.regs[0x50] = math.fmod((self.pos - self.offset) * float(self.regs[0x14]), 2 * math.pi)
        self.regs[0x3C] = 24.0 + random.uniform(-0.05, 0.05)

    def out_pos(self) -> float:
        """输出轴单圈位置 [-π, π)，与实机一致。"""
        x = self.pos - self.offset
        return (x + math.pi) % (2 * math.pi) - math.pi

    def code(self) -> int:
        if self.err:
            return self.err
        return P.CODE_ENABLED if self.enabled else P.CODE_DISABLED

    def status(self) -> bytes:
        noise = random.gauss(0, 0.0005)
        return P.pack_status(self.motor_id, self.code(), self.out_pos() + noise, self.vel,
                             self.tau, int(self.temp), int(self.temp - 2), self.lim)

    def handle(self, can_id: int, d: bytes) -> list[tuple[int, bytes]]:
        """处理一帧，返回要发出的帧列表。"""
        if len(d) != 8:
            return []
        out = []
        if can_id == self.motor_id:
            if P.is_special(d):
                code = d[7]
                if code == P.SPECIAL_ENABLE:
                    self.enabled = True
                    self.cmd = P.MitCommand()
                elif code == P.SPECIAL_DISABLE:
                    self.enabled = False
                elif code == P.SPECIAL_ZERO:
                    self.offset = self.pos
                    self.cmd.p = 0.0
                elif code == P.SPECIAL_CLEAR_ERROR:
                    self.err = 0
            else:
                self.cmd = P.unpack_mit(d, self.lim)
            out.append((self.master_id, self.status()))
        elif can_id == P.CONFIG_ID:
            tgt = d[0] | (d[1] << 8)
            if tgt != self.motor_id:
                return []
            cmd = d[2]
            if cmd == P.CMD_READ:
                reg = P.REG_BY_RID.get(d[3])
                val = self.regs.get(d[3], 0)
                raw = P.encode_value(reg.typ if reg else "u", val)
                out.append((self.master_id, bytes(d[:4]) + raw))
            elif cmd == P.CMD_WRITE:
                reg = P.REG_BY_RID.get(d[3])
                if reg and not reg.ro:
                    self.regs[d[3]] = P.decode_value(reg.typ, d[4:8])
                raw = P.encode_value(reg.typ if reg else "u", self.regs.get(d[3], 0))
                out.append((self.master_id, bytes(d[:4]) + raw))
            elif cmd == P.CMD_SAVE:
                if d[3] == P.SAVE_SUB and not self.enabled:
                    out.append((self.master_id, bytes(d[:4]) + bytes(4)))
            elif cmd == P.CMD_STATE:
                out.append((self.master_id, self.status()))
        return out


class SimBus(CanBus):
    """虚拟总线：后台线程以 1 kHz 积分电机模型。可挂多台电机以测试扫描。"""

    def __init__(self, motors: list[SimMotor] | None = None):
        self.name = "仿真总线"
        self.motors = motors or [SimMotor(1, 0), SimMotor(5, 0)]
        self._rx: deque[Frame] = deque(maxlen=10000)
        self._lock = threading.Lock()
        self._run = False
        self._th: threading.Thread | None = None

    def open(self) -> None:
        self._run = True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def close(self) -> None:
        self._run = False
        if self._th:
            self._th.join(timeout=1)

    def _loop(self) -> None:
        last = time.perf_counter()
        while self._run:
            time.sleep(0.001)
            now = time.perf_counter()
            dt = min(now - last, 0.02)
            last = now
            with self._lock:
                for m in self.motors:
                    n = max(1, int(dt / 0.0005))
                    for _ in range(n):
                        m.step(dt / n)
                    to = int(m.regs[0x09])
                    # 不实现超时报警细节，仅占位
                    _ = to

    def send(self, can_id: int, data: bytes) -> None:
        if not self._run:
            raise BusError("总线未打开")
        with self._lock:
            now = time.perf_counter()
            for m in self.motors:
                for rid, rd in m.handle(can_id, bytes(data)):
                    self._rx.append(Frame(now + 0.0002, rid, rd))

    def recv_all(self) -> list[Frame]:
        with self._lock:
            out = list(self._rx)
            self._rx.clear()
        return out
