"""高擎 GD 经典 CAN 协议（用户提供的 1.3 手册，int16 + MIT）。

位置/速度在线上为圈、圈/s，在上位机转换为 rad、rad/s。
MIT Kp/Kd 保留手册的原生增益刻度（按圈、圈/s 计算），不套用达妙打包。
协议量程不是电机的额定机械能力；测试输出仍受 Supervisor 限幅。
"""
from __future__ import annotations

import math
import struct

from .base import Caps, Driver, Feedback
from . import dm_protocol as P

MODEL = "高擎 GD（经典 CAN）"
MODEL_5036 = "GD 5036-02-CNE"
MODEL_5036_FAMILY = "GD 5036（通用）"
MODEL_6036 = "GD 6036（通用）"
MODELS = [MODEL_5036, MODEL_5036_FAMILY, MODEL_6036, MODEL]
TX_BASE = 0x18000
MIT_BASE = 0x58000
RX_BASE = 0x10000
QUERY_STATUS = 0x0E
QUERIES = {"firmware": 0x04, "hardware": 0x05, "model": 0x07}
SYSTEM = {"reboot": 1, "save": 2, "zero": 3, "change_id": 4}
CONTROL_MODES = {"velocity": 0x1C, "position": 0x1D, "position_velocity_torque": 0x1F,
                 "coast": 0x01, "brake": 0x18}
TAU = 2 * math.pi
LIMITS = P.Limits(p_min=-3.2768 * TAU, p_max=3.2767 * TAU,
                  v_min=-10 * TAU, v_max=10 * TAU,
                  t_min=-100, t_max=100,
                  kp_min=-800, kp_max=800, kd_min=-200, kd_max=200)


def motor_from_status(can_id: int, data: bytes) -> int | None:
    mid = (can_id >> 8) & 0x7F
    if (1 <= mid <= 126 and can_id == (RX_BASE | (mid << 8))
            and len(data) == 8 and data[0] == QUERY_STATUS):
        return mid
    return None


def pack_mit(cmd: P.MitCommand) -> bytes:
    fields = ((cmd.p / TAU, -3.2768, 3.2767, 16, 0),
              (cmd.v / TAU, -10, 10, 12, 16),
              (cmd.t, -100, 100, 12, 28),
              (cmd.kp, -800, 800, 12, 40),
              (cmd.kd, -200, 200, 12, 52))
    raw = 0
    for value, lo, hi, bits, shift in fields:
        if not math.isfinite(value):
            raise ValueError("MIT 参数必须为有限数值")
        raw |= P.float_to_uint(value, lo, hi, bits) << shift
    return raw.to_bytes(8, "little")


def unpack_mit(data: bytes) -> P.MitCommand:
    if len(data) != 8:
        raise ValueError("GD MIT 帧必须为 8 字节")
    raw = int.from_bytes(data, "little")
    def field(shift, bits, lo, hi):
        return P.uint_to_float((raw >> shift) & ((1 << bits) - 1), lo, hi, bits)
    return P.MitCommand(p=field(0, 16, -3.2768, 3.2767) * TAU,
                        v=field(16, 12, -10, 10) * TAU,
                        t=field(28, 12, -100, 100),
                        kp=field(40, 12, -800, 800),
                        kd=field(52, 12, -200, 200))


class GdDriver(Driver):
    protocol = "gd"
    protocol_name = "高擎 GD"
    extended = True
    caps = Caps(mit=True, zero=True, status_code=True)
    lim = LIMITS
    note = "经典 CAN 1 Mbps 扩展帧；扭矩单位 Nm；反馈不含温度和使能状态"
    limits_source = "协议固定量程"

    def __init__(self, model: str = MODEL, motor_id: int = 1):
        if model not in MODELS:
            raise ValueError(f"未知 GD 型号：{model}")
        if not 1 <= motor_id <= 126:
            raise ValueError("GD 电机 ID 范围为 1~126")
        super().__init__(model, motor_id)
        self.identity = {}

    def query_frame(self, kind):
        return bytes((0, QUERIES[kind]))

    def system_frame(self, kind, value=None):
        if kind == "periodic":
            if isinstance(value, bool) or not isinstance(value, int) or not (value == 0 or 100 <= value <= 0xFFFFFFFF):
                raise ValueError("周期为 0（关闭）或 100~4294967295 微秒")
            return bytes((3, 0, 5, QUERY_STATUS)) + struct.pack("<I", value)
        frame = bytes((3, 3, SYSTEM[kind]))
        if kind == "change_id":
            if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 126:
                raise ValueError("GD 电机 ID 范围为 1~126")
            frame += bytes((value,))
        return frame

    @staticmethod
    def parse_identity(kind, data):
        if kind not in QUERIES or not data or data[0] != QUERIES[kind] or len(data) > 8:
            raise ValueError("查询返回格式不符")
        if kind != "model":
            if len(data) != 4:
                raise ValueError("版本返回应为 4 字节")
            return f"{data[3]}.{data[2]}.{data[1]}"
        if len(data) < 2 or data[1] == 0:
            raise ValueError("型号返回为空")
        length = data[1]
        payload = data[2:2 + length]
        try:
            name = payload.decode("ascii")
        except UnicodeDecodeError as e:
            raise ValueError("型号包含非 ASCII 数据") from e
        if not name or any(ord(c) < 32 or ord(c) > 126 for c in name):
            raise ValueError("型号数据无效")
        return {"text": name, "complete": length <= len(data) - 2, "length": length}

    def apply_identity(self, kind, value):
        self.identity[kind] = value
        if kind == "model":
            name = value["text"].upper()
            if name.startswith("5036") and "5036" not in self.model:
                self.model = MODEL_5036_FAMILY
            elif name.startswith("6036"):
                self.model = MODEL_6036

    @staticmethod
    def _scaled(value, scale):
        if not math.isfinite(value):
            raise ValueError("控制参数必须为有限数值")
        raw = round(value * scale)
        if not -32768 <= raw <= 32767:
            raise ValueError("控制参数超出 int16 协议范围")
        return raw

    def control_frames(self, kind, values):
        mode = CONTROL_MODES[kind]
        fields = []
        if kind in ("position", "position_velocity_torque"):
            fields.append(self._scaled(values["position"], 10000 / TAU))
        if kind in ("velocity", "position_velocity_torque"):
            velocity = values["velocity"]
            if kind == "position_velocity_torque" and velocity < 0:
                raise ValueError("最大速度不能为负数")
            fields.append(self._scaled(velocity, 4000 / TAU))
        if kind == "position_velocity_torque":
            torque = values["torque"]
            if not 0 <= torque <= self.out_max:
                raise ValueError("扭矩上限应在 0~100 Nm")
            fields.append(self._scaled(torque, 100))
        return [(self.tx_id, bytes((mode, QUERY_STATUS)) + struct.pack("<" + "h" * len(fields), *fields))]

    @property
    def tx_id(self):
        return TX_BASE | self.motor_id

    @property
    def fb_id(self):
        return RX_BASE | (self.motor_id << 8)

    @property
    def out_max(self):
        return 100.0  # 与 MIT 公用的输出范围，不表示电机额定扭矩

    @property
    def wrap_span(self):
        return 65536 / 10000 * TAU  # 状态位置为 int16，0.0001 圈/LSB

    def accepts(self, can_id, data):
        return can_id == self.fb_id and motor_from_status(can_id, data) == self.motor_id

    def parse(self, can_id, data):
        if not self.accepts(can_id, data):
            return None
        pos, vel, torque = struct.unpack_from("<hhh", data, 2)
        return Feedback(pos=pos / 10000 * TAU, vel=vel / 4000 * TAU,
                        out=torque / 100, code=data[1], motor_id=self.motor_id,
                        raw=bytes(data))

    def output_frames(self, out):
        if not math.isfinite(out):
            raise ValueError("扭矩必须为有限数值")
        raw = round(P.clamp(out, -self.out_max, self.out_max) * 100)
        return [(self.tx_id, bytes((0x1B, QUERY_STATUS)) + struct.pack("<h", raw))]

    def mit_frames(self, cmd):
        # 偏移码无法精确表示零增益。纯扭矩用原生 0x1B，保证零命令精确。
        if cmd.kp == 0 and cmd.kd == 0:
            return self.output_frames(cmd.t)
        # MIT 自身没有查询码，另发状态查询，不改电机的周期返回配置。
        return [(MIT_BASE | self.motor_id, pack_mit(cmd)), *self.idle_frames()]

    def idle_frames(self):
        return [(self.tx_id, bytes((0x00, QUERY_STATUS)))]

    def disable_frames(self):
        return [(self.tx_id, bytes((0x01, QUERY_STATUS)))]  # 三相断开，自由停转

    def zero_frames(self):
        return [(self.tx_id, bytes((0x03, 0x03, 0x03)))]

    def code_text(self, code):
        return "无错误" if code == 0 else f"错误码 0x{code:02X}"

    def is_fault(self, code):
        return code > 0

    def describe(self, can_id, data, tx):
        if tx:
            if can_id == MIT_BASE | self.motor_id and len(data) == 8:
                c = unpack_mit(data)
                return f"MIT 位置: {c.p:+.4f} rad, 速度: {c.v:+.3f} rad/s, 前馈: {c.t:+.3f} Nm"
            if can_id == self.tx_id and len(data) >= 2:
                if data[0] == 0x1B and len(data) == 4:
                    return f"下发扭矩: {struct.unpack_from('<h', data, 2)[0] / 100:+.3f} Nm"
                if data[0] == 0:
                    return {4: "查询固件版本", 5: "查询硬件版本", 7: "查询电机型号"}.get(data[1], "查询状态")
                if data[0] == 3:
                    if len(data) == 8 and data[:4] == bytes((3, 0, 5, QUERY_STATUS)):
                        return f"设置周期反馈: {int.from_bytes(data[4:], 'little')} μs"
                    if len(data) >= 3:
                        return {1: "软重启", 2: "保存设置", 3: "重设零位", 4: f"修改ID: {data[3] if len(data) == 4 else '?'}"}.get(data[2], "系统命令")
                if data[0] in (0x1C, 0x1D) and len(data) == 4:
                    raw = struct.unpack_from("<h", data, 2)[0]
                    return (f"目标速度: {raw * TAU / 4000:+.4f} rad/s" if data[0] == 0x1C else
                            f"目标位置: {raw * TAU / 10000:+.4f} rad")
                if data[0] == 0x1F and len(data) == 8:
                    p, v, t = struct.unpack_from("<hhh", data, 2)
                    return f"位置: {p * TAU / 10000:+.4f} rad, 速度上限: {v * TAU / 4000:.3f} rad/s, 扭矩上限: {t / 100:.2f} Nm"
                return {0x01: "自由停转", 0x18: "阻尼制动"}.get(data[0], "")
            return ""
        if can_id == self.fb_id and data:
            if len(data) == 2 and data[0] == 3:
                return "系统命令确认" if data[1] == 0 else f"系统命令失败: 0x{data[1]:02X}"
            for kind, query in QUERIES.items():
                if data[0] == query:
                    try:
                        value = self.parse_identity(kind, data)
                    except ValueError as e:
                        return str(e)
                    if kind == "model":
                        return "电机型号: " + value["text"] + ("" if value["complete"] else "（不完整）")
                    return ("固件版本: " if kind == "firmware" else "硬件版本: ") + value
        fb = self.parse(can_id, data)
        if fb is None:
            return ""
        return (f"位置: {fb.pos:>+9.4f} rad, 速度: {fb.vel:>+9.3f} rad/s, "
                f"扭矩: {fb.out:>+8.3f} Nm, {self.code_text(fb.code)}")
