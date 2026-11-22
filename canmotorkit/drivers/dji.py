"""大疆 RoboMaster 电调驱动（C620/M3508、C610/M2006、GM6020、RM 6623）。

协议要点（reference/ 下大疆说明书）：
- 控制：一帧控制 4 台，16 位有符号、高字节在前。本软件只控制选中的一台，同帧其余槽位填 0（会把
  同组其它电机的电流置 0）。
- 反馈：电调以约 1 kHz 主动广播，无需查询；角度 0~8191 为转子一圈，转速为转子 rpm。
- 无使能命令：「使能」是软件开关，失能时发 0 电流。
位置/速度按减速比换算到输出轴；电流为厂家比例对应的 A。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .base import Caps, Driver, Feedback, Frames


@dataclass(frozen=True)
class DjiModel:
    name: str
    esc: str
    ids: range
    fb_base: int          # 反馈 ID = fb_base + 电机 ID
    ctrl_lo: int          # 前 4 个设备的控制帧 ID
    ctrl_hi: int          # 后 4 个设备的控制帧 ID
    raw_max: int          # 指令原始值上限
    amp_max: float        # 对应输出单位的上限
    gr: float             # 减速比
    has_temp: bool
    note: str = ""
    feedback_format: str = "rpm_current_temp"
    zero_based_id: bool = False
    out_name: str = "电流"
    out_unit: str = "A"


MODELS: dict[str, DjiModel] = {
    "M3508 (C620)": DjiModel("M3508 (C620)", "C620", range(1, 9), 0x200, 0x200, 0x1FF, 16384, 20.0,
                             3591 / 187, True),
    "M2006 (C610)": DjiModel("M2006 (C610)", "C610", range(1, 9), 0x200, 0x200, 0x1FF, 10000, 10.0,
                             36.0, False, "反馈电流比例按 ±10000↔±10 A 假定，待核对；C610 无温度反馈"),
    "GM6020 (电流模式)": DjiModel("GM6020 (电流模式)", "GM6020", range(1, 8), 0x204, 0x1FE, 0x2FE, 16384,
                                3.0, 1.0, True,
                                "需在 RoboMaster Assistant 中打开电流环；反馈电流比例按 ±16384↔±3 A 假定，待核对"),
    "RM 6623": DjiModel("RM 6623", "6623", range(0, 8), 0x205, 0x1FF, 0x2FF, 5000,
                        5000.0, 1.0, False,
                        "6623 电流原始值范围 ±5000；反馈无 RPM/温度字段，设备 ID 0~7",
                        feedback_format="6623", zero_based_id=True, out_unit="原始值"),
}


def _i16(hi: int, lo: int) -> int:
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v


class DjiDriver(Driver):
    protocol = "dji"
    protocol_name = "大疆 RoboMaster"
    caps = Caps()
    @property
    def out_name(self) -> str:
        return self.m.out_name

    @property
    def out_unit(self) -> str:
        return self.m.out_unit

    def __init__(self, model: str = "M3508 (C620)", motor_id: int = 1):
        m = MODELS[model]
        if motor_id not in m.ids:
            raise ValueError(f"{model} 的 ID 范围为 {m.ids.start}~{m.ids.stop - 1}")
        super().__init__(model, motor_id)
        self.m = m
        self.note = m.note

    @property
    def out_max(self) -> float:
        return self.m.amp_max

    @property
    def wrap_span(self) -> float:
        return 2 * math.pi / self.m.gr

    @property
    def fb_id(self) -> int:
        return self.m.fb_base + self.motor_id

    @property
    def ctrl(self) -> tuple[int, int]:
        """(控制帧 ID, 槽位 0~3)"""
        if self.m.zero_based_id:
            if self.motor_id < 4:
                return self.m.ctrl_lo, self.motor_id
            return self.m.ctrl_hi, self.motor_id - 4
        if self.motor_id <= 4:
            return self.m.ctrl_lo, self.motor_id - 1
        return self.m.ctrl_hi, self.motor_id - 5

    def title(self) -> str:
        cid, slot = self.ctrl
        return f"{self.model}  ID {self.motor_id}（反馈 0x{self.fb_id:03X} / 控制 0x{cid:03X}#{slot + 1}）"

    def accepts(self, can_id: int, data: bytes) -> bool:
        return can_id == self.fb_id and len(data) >= 7

    def parse(self, can_id: int, data: bytes) -> Feedback | None:
        d = data
        ang = (d[0] << 8) | d[1]
        if self.m.feedback_format == "6623":
            # 6623: angle, actual current, commanded current, reserved.
            cur = _i16(d[2], d[3])
            rotor = ang / 8192.0 * 2 * math.pi
            if rotor >= math.pi:
                rotor -= 2 * math.pi
            return Feedback(pos=rotor / self.m.gr, vel=float("nan"), out=float(cur),
                            temp=float("nan"), motor_id=self.motor_id, raw=bytes(d))
        rpm = _i16(d[2], d[3])
        cur = _i16(d[4], d[5])
        rotor = ang / 8192.0 * 2 * math.pi
        if rotor >= math.pi:
            rotor -= 2 * math.pi
        gr = self.m.gr
        return Feedback(pos=rotor / gr, vel=rpm * 2 * math.pi / 60.0 / gr,
                        out=cur / self.m.raw_max * self.m.amp_max,
                        temp=float(d[6]) if self.m.has_temp else float("nan"),
                        motor_id=self.motor_id, raw=bytes(d))

    def encode(self, out: float) -> int:
        a = self.m.amp_max
        out = max(-a, min(a, out))
        return int(round(out / a * self.m.raw_max))

    def output_frames(self, out: float) -> Frames:
        cid, slot = self.ctrl
        buf = bytearray(8)
        buf[2 * slot:2 * slot + 2] = self.encode(out).to_bytes(2, "big", signed=True)
        return [(cid, bytes(buf))]

    def describe(self, can_id: int, data: bytes, tx: bool) -> str:
        cid, slot = self.ctrl
        if tx and can_id == cid and len(data) == 8:
            raw = _i16(data[2 * slot], data[2 * slot + 1])
            if self.m.feedback_format == "6623":
                return f"下发电流: {raw:+d} 原始值"
            return f"下发电流: {raw / self.m.raw_max * self.m.amp_max:+.3f} A"
        if not tx and can_id == self.fb_id:
            if len(data) < 7:
                return "反馈（数据长度不足）"
            ang = (data[0] << 8) | data[1]
            rotor = ang / 8192.0 * 2 * math.pi
            if rotor >= math.pi:
                rotor -= 2 * math.pi
            pos = rotor / self.m.gr
            if self.m.feedback_format == "6623":
                actual = _i16(data[2], data[3])
                command = _i16(data[4], data[5])
                return (f"位置: {pos:>8.5f} rad, 速度: —, "
                        f"电流: {actual:>+6d} 原始值, 指令: {command:>+6d} 原始值")
            rpm = _i16(data[2], data[3])
            cur = _i16(data[4], data[5])
            current = cur / self.m.raw_max * self.m.amp_max
            position_text = f"{pos:>8.5f}"
            speed_text = f"{rpm:>5d}"
            current_text = f"{current:>+8.5f}"
            temperature_text = f", 温度: {data[6]:>3.0f} °C" if self.m.has_temp else ""
            return (f"位置: {position_text} rad, 速度: {speed_text} rpm, "
                    f"电流: {current_text} A{temperature_text}")
        return ""


def guess_models(fb_id: int, data: bytes) -> list[tuple[str, int]]:
    """根据反馈帧 ID 猜测可能的 (型号, 设备 ID)。多个 DJI 型号的 ID 会重叠。"""
    out = []
    for m in MODELS.values():
        mid = fb_id - m.fb_base
        if mid in m.ids:
            out.append((m.name, mid))
    # C610 无温度字节：温度为 0 时把 M2006 排前，否则 M3508 排前
    no_temp = len(data) >= 7 and data[6] == 0
    out.sort(key=lambda x: {"M2006 (C610)": 0 if no_temp else 2, "M3508 (C620)": 1}.get(x[0], 3))
    return out
