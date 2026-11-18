"""演示台：虚拟 CAN 总线 + 多种虚拟电机（无需硬件）。

用途：演示软件功能、自测。涵盖：
- HM9816G9（ID 1 / Master 0x00）：HM 寄存器集，p_max=π（与实机一致），静摩擦约 0.8 Nm
- 达妙 J8009P（ID 3 / Master 0x13）：达妙寄存器集，p_max=12.5
- 大疆 M3508/C620（ID 1，反馈 0x201）、M2006/C610（ID 2，反馈 0x202）、GM6020 电流模式（ID 2，反馈 0x206）
物理模型在输出轴侧：J·a = k·u − b·v − 库仑摩擦（含静摩擦）；应答有约 1 ms 延迟。
"""
from __future__ import annotations

import heapq
import math
import random
import threading
import time
from dataclasses import dataclass

from ..core.bus import BusError, CanBus, Frame
from ..drivers import dji
from ..drivers import dm_protocol as P


@dataclass
class Plant:
    J: float            # 输出轴等效惯量 kg·m²
    b: float            # 粘滞 Nm·s/rad
    fc: float           # 库仑摩擦 Nm
    fs: float           # 静摩擦 Nm（≥ fc）
    k: float = 1.0      # 输出量 → 输出轴扭矩系数（达妙/HM 为 1，大疆为 Nm/A）
    pos: float = 0.0    # 输出轴累计位置
    vel: float = 0.0

    def step(self, u: float, dt: float):
        tau = self.k * u
        if abs(self.vel) < 1e-3 and abs(tau) <= self.fs:
            self.vel = 0.0
            return
        s = math.copysign(1.0, self.vel if abs(self.vel) >= 1e-3 else tau)
        acc = (tau - self.fc * s - self.b * self.vel) / self.J
        v_new = self.vel + acc * dt
        if self.vel != 0.0 and (v_new > 0) != (self.vel > 0) and abs(tau) <= self.fs:
            v_new = 0.0     # 穿过零速且扭矩不足以克服静摩擦 → 停住
        self.pos += 0.5 * (self.vel + v_new) * dt
        self.vel = v_new


def _wrap(x: float, half: float) -> float:
    return (x + half) % (2 * half) - half


# ============================ 达妙 / HM ============================
class DmSimMotor:
    def __init__(self, family: str, motor_id: int, master_id: int, p_max: float, v_max: float,
                 t_max: float, plant: Plant):
        self.family = family
        self.plant = plant
        self.enabled = False
        self.err = 0
        self.cmd = P.MitCommand()
        self.tau = 0.0
        self.offset = 0.0
        self.temp = 30.0
        self.last_cmd_t = time.perf_counter()
        self.regs: dict[int, float | int] = {}
        for r in P.REGISTERS:
            if r.src in ("both", family):
                self.regs[r.rid] = 0.0 if r.typ == "f" else 0
        self.regs.update({0x00: 12.0, 0x01: 0.12, 0x02: 100.0, 0x03: 14.0, 0x07: master_id,
                          0x08: motor_id, 0x09: 0, 0x0B: 0.0, 0x0C: 0.0, 0x10: 21, 0x11: 0.121,
                          0x12: 70e-6, 0x13: 0.0035, 0x14: 9.0, 0x15: p_max, 0x16: v_max, 0x17: t_max,
                          0x18: 1000.0, 0x1D: 60.0, 0x23: 4, 0x50: 0.0, 0x51: 0.0})
        if family == "hm":
            self.regs.update({0x0D: 1, 0x0E: 2, 0x3B: 40.0, 0x3C: 24.0, 0x3D: 30.0, 0x3E: 30.0})
        else:
            self.regs.update({0x0A: 1, 0x0D: 11, 0x0E: 5013, 0x0F: 20261110, 0x24: 1})

    @property
    def lim(self) -> P.Limits:
        return P.DEFAULT_LIMITS.with_maps(float(self.regs[0x15]), float(self.regs[0x16]),
                                          float(self.regs[0x17]))

    @property
    def motor_id(self) -> int:
        return int(self.regs[0x08])

    @property
    def master_id(self) -> int:
        return int(self.regs[0x07])

    def out_pos(self) -> float:
        p_max = float(self.regs[0x15])
        return _wrap(self.plant.pos - self.offset, p_max)

    def code(self) -> int:
        if self.err:
            return self.err
        return P.CODE_ENABLED if self.enabled else P.CODE_DISABLED

    def inject_fault(self, code: int):
        self.err = code
        self.enabled = False

    def step(self, dt: float, now: float):
        to = int(self.regs.get(0x09, 0))
        if self.enabled and to > 0 and now - self.last_cmd_t > to * 50e-6:
            self.inject_fault(0xD)
        if self.enabled and not self.err:
            c = self.cmd
            tau = c.kp * (c.p - self.out_pos()) + c.kd * (c.v - self.plant.vel) + c.t
            L = self.lim
            tau = P.clamp(tau, L.t_min, L.t_max)
        else:
            tau = 0.0
        self.tau = tau
        self.plant.step(tau, dt)
        self.temp += (30.0 + abs(tau) * 0.8 - self.temp) * dt * 0.05
        if self.family == "hm":
            self.regs[0x3D] = self.temp
            self.regs[0x3E] = self.temp
            self.regs[0x3C] = 24.0 + random.uniform(-0.05, 0.05)
        self.regs[0x51] = self.plant.pos - self.offset
        self.regs[0x50] = _wrap((self.plant.pos - self.offset) * float(self.regs[0x14]), math.pi)

    def status(self) -> bytes:
        L = self.lim
        return P.pack_status(self.motor_id, self.code(), self.out_pos() + random.gauss(0, 2e-4),
                             self.plant.vel, self.tau, int(self.temp), int(self.temp), L)

    def handle(self, can_id: int, d: bytes, now: float) -> list[tuple[int, bytes]]:
        if len(d) != 8:
            return []
        if can_id == self.motor_id:
            if P.is_special(d):
                c = d[7]
                if c == P.SPECIAL_ENABLE and not self.err:
                    self.enabled = True
                    self.last_cmd_t = now
                elif c == P.SPECIAL_DISABLE:
                    self.enabled = False
                elif c == P.SPECIAL_ZERO:
                    self.offset = self.plant.pos
                    self.cmd.p = 0.0
                elif c == P.SPECIAL_CLEAR_ERROR:
                    self.err = 0
            else:
                self.cmd = P.unpack_mit(d, self.lim)
                self.last_cmd_t = now
            return [(self.master_id, self.status())]
        if can_id != P.CONFIG_ID or (d[0] | (d[1] << 8)) != self.motor_id:
            return []
        cmd, rid = d[2], d[3]
        if cmd == P.CMD_STATE:
            return [(self.master_id, self.status())]
        if cmd == P.CMD_SAVE:
            return [(self.master_id, bytes(d[:4]) + bytes(4))] if d[3] == 1 and not self.enabled else []
        if rid not in self.regs:
            return []           # 本家族没有的寄存器：不应答
        reg = P.REG_BY_RID.get(rid)
        typ = reg.typ if reg else "u"
        if cmd == P.CMD_WRITE:
            self.regs[rid] = P.decode_value(typ, d[4:8])
        elif cmd != P.CMD_READ:
            return []
        return [(self.master_id, bytes(d[:4]) + P.encode_value(typ, self.regs[rid]))]


# ============================ 大疆 ============================
class DjiSimMotor:
    def __init__(self, model: str, motor_id: int, plant: Plant):
        self.m = dji.MODELS[model]
        self.motor_id = motor_id
        self.plant = plant
        self.cur = 0.0
        self.last_cmd_t = 0.0
        self.temp = 32.0

    @property
    def fb_id(self) -> int:
        return self.m.fb_base + self.motor_id

    def handle(self, can_id: int, d: bytes, now: float) -> list[tuple[int, bytes]]:
        cid, slot = (self.m.ctrl_lo, self.motor_id - 1) if self.motor_id <= 4 else (
            self.m.ctrl_hi, self.motor_id - 5)
        if can_id == cid and len(d) >= 2 * slot + 2:
            raw = int.from_bytes(d[2 * slot:2 * slot + 2], "big", signed=True)
            self.cur = max(-1.0, min(1.0, raw / self.m.raw_max)) * self.m.amp_max
            self.last_cmd_t = now
        return []

    def step(self, dt: float, now: float):
        if now - self.last_cmd_t > 0.1:       # 大疆电调断联保护：停止输出
            self.cur = 0.0
        self.plant.step(self.cur, dt)
        self.temp += (32.0 + abs(self.cur) * 0.5 - self.temp) * dt * 0.05

    def feedback(self) -> bytes:
        gr = self.m.gr
        rotor = (self.plant.pos * gr) % (2 * math.pi)
        ang = int(rotor / (2 * math.pi) * 8192) % 8192
        rpm = int(round(self.plant.vel * gr * 60 / (2 * math.pi)))
        rpm = max(-32768, min(32767, rpm))
        cur = int(round(self.cur / self.m.amp_max * self.m.raw_max + random.gauss(0, 20)))
        cur = max(-32768, min(32767, cur))
        temp = int(self.temp) if self.m.has_temp else 0
        return (ang.to_bytes(2, "big") + rpm.to_bytes(2, "big", signed=True) +
                cur.to_bytes(2, "big", signed=True) + bytes([temp, 0]))


# ============================ 虚拟总线 ============================
def default_motors() -> list:
    return [
        DmSimMotor("hm", 1, 0x00, math.pi, 45.0, 54.0, Plant(J=0.006, b=0.02, fc=0.6, fs=0.8)),
        DmSimMotor("dm", 3, 0x13, 12.5, 45.0, 54.0, Plant(J=0.006, b=0.03, fc=0.5, fs=0.7)),
        DjiSimMotor("M3508 (C620)", 1, Plant(J=0.002, b=0.004, fc=0.08, fs=0.1, k=0.3)),
        DjiSimMotor("M2006 (C610)", 2, Plant(J=0.0008, b=0.002, fc=0.03, fs=0.04, k=0.18)),
        DjiSimMotor("GM6020 (电流模式)", 2, Plant(J=0.003, b=0.01, fc=0.05, fs=0.07, k=0.741)),
    ]


class DemoBus(CanBus):
    """后台 2 kHz 积分；大疆电机 1 kHz 广播反馈；应答延迟 latency 秒后可读。"""

    def __init__(self, motors: list | None = None, latency: float = 0.001):
        self.name = "演示台"
        self.motors = default_motors() if motors is None else motors
        self.latency = latency
        self._q: list[tuple[float, int, int, bytes]] = []
        self._seq = 0
        self._lock = threading.Lock()
        self._run = False
        self._th: threading.Thread | None = None

    def motor(self, kind: type, motor_id: int):
        return next(m for m in self.motors if isinstance(m, kind) and m.motor_id == motor_id)

    def open(self) -> None:
        self._run = True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def close(self) -> None:
        self._run = False
        if self._th:
            self._th.join(timeout=1)

    def _push(self, t: float, can_id: int, data: bytes):
        self._seq += 1
        heapq.heappush(self._q, (t, self._seq, can_id, data))

    def _loop(self) -> None:
        last = time.perf_counter()
        next_fb = last
        while self._run:
            time.sleep(0.0005)
            now = time.perf_counter()
            dt = min(now - last, 0.02)
            last = now
            with self._lock:
                n = max(1, int(dt / 0.0005))
                for m in self.motors:
                    for _ in range(n):
                        m.step(dt / n, now)
                if now >= next_fb:
                    next_fb = max(next_fb + 0.001, now - 0.005)
                    for m in self.motors:
                        if isinstance(m, DjiSimMotor):
                            self._push(now, m.fb_id, m.feedback())

    def send(self, can_id: int, data: bytes, extended: bool = False,
             rtr: bool = False, dlc: int | None = None) -> None:
        if not self._run:
            raise BusError("总线未打开")
        with self._lock:
            now = time.perf_counter()
            for m in self.motors:
                for rid, rd in m.handle(can_id, bytes(data), now):
                    self._push(now + self.latency * random.uniform(0.8, 1.6), rid, rd)

    def recv_all(self) -> list[Frame]:
        now = time.perf_counter()
        out = []
        with self._lock:
            while self._q and self._q[0][0] <= now:
                t, _, cid, d = heapq.heappop(self._q)
                out.append(Frame(t, cid, d))
        return out
