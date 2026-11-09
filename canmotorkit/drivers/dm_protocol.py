"""HM 系列关节伺服 CAN 协议（依据《使用手册 V1.06》）。纯函数，无 IO。"""
from __future__ import annotations

import struct
from dataclasses import dataclass, replace

# ---------------- 常量 ----------------
CONFIG_ID = 0x7FF            # 自定义协议请求帧 ID
CMD_READ = 0x33
CMD_WRITE = 0x55
CMD_SAVE = 0xAA              # d3=0x01 保存；d3=0x23 改波特率(仅 CAN FD 版)
CMD_STATE = 0xCC
SAVE_SUB = 0x01
BAUD_SUB = 0x23
CONFIG_CMDS = (CMD_READ, CMD_WRITE, CMD_SAVE)

SPECIAL_CLEAR_ERROR = 0xFB
SPECIAL_ENABLE = 0xFC        # MOTOR_CMD 进入运行模式
SPECIAL_DISABLE = 0xFD       # MENU_CMD  进入菜单/待机模式
SPECIAL_ZERO = 0xFE          # 设当前位置为零点
SPECIAL_NAMES = {0xFB: "清除错误", 0xFC: "使能", 0xFD: "失能", 0xFE: "设零点"}


# ---------------- 量程 ----------------
@dataclass(frozen=True)
class Limits:
    p_min: float = -12.5
    p_max: float = 12.5
    v_min: float = -45.0
    v_max: float = 45.0
    t_min: float = -54.0
    t_max: float = 54.0
    kp_min: float = 0.0
    kp_max: float = 500.0
    kd_min: float = 0.0
    kd_max: float = 5.0

    def with_maps(self, p_max: float | None = None, v_max: float | None = None,
                  t_max: float | None = None) -> "Limits":
        """用寄存器 0x15/0x16/0x17 读到的对称映射范围覆盖。"""
        kw = {}
        if p_max and p_max > 0:
            kw.update(p_min=-p_max, p_max=p_max)
        if v_max and v_max > 0:
            kw.update(v_min=-v_max, v_max=v_max)
        if t_max and t_max > 0:
            kw.update(t_min=-t_max, t_max=t_max)
        return replace(self, **kw)


# 兜底量程（手册 G9 默认值）。p/v/t 实际量程以连接后从电机读取的 0x15/0x16/0x17 为准，
# 未读取成功时禁止使能；KP/KD 量程寄存器表中没有，固定为 0~500 / 0~5。
DEFAULT_LIMITS = Limits()


# ---------------- 定点编解码 ----------------
def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """手册 float_to_uint，增加钳位与四舍五入，防止溢出回绕。"""
    span = x_max - x_min
    full = (1 << bits) - 1
    x = clamp(x, x_min, x_max)
    return max(0, min(full, int(round((x - x_min) * full / span))))


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    return x_int * span / ((1 << bits) - 1) + x_min


def lsb(x_min: float, x_max: float, bits: int) -> float:
    return (x_max - x_min) / ((1 << bits) - 1)


# ---------------- MIT 控制帧 ----------------
@dataclass
class MitCommand:
    p: float = 0.0
    v: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    t: float = 0.0


def pack_mit(cmd: MitCommand, lim: Limits) -> bytes:
    p = float_to_uint(cmd.p, lim.p_min, lim.p_max, 16)
    v = float_to_uint(cmd.v, lim.v_min, lim.v_max, 12)
    kp = float_to_uint(cmd.kp, lim.kp_min, lim.kp_max, 12)
    kd = float_to_uint(cmd.kd, lim.kd_min, lim.kd_max, 12)
    t = float_to_uint(cmd.t, lim.t_min, lim.t_max, 12)
    return bytes((
        p >> 8, p & 0xFF,
        v >> 4, ((v & 0x0F) << 4) | ((kp >> 8) & 0x0F),
        kp & 0xFF,
        kd >> 4, ((kd & 0x0F) << 4) | ((t >> 8) & 0x0F),
        t & 0xFF,
    ))


def unpack_mit(d: bytes, lim: Limits) -> MitCommand:
    p = (d[0] << 8) | d[1]
    v = (d[2] << 4) | (d[3] >> 4)
    kp = ((d[3] & 0x0F) << 8) | d[4]
    kd = (d[5] << 4) | (d[6] >> 4)
    t = ((d[6] & 0x0F) << 8) | d[7]
    return MitCommand(
        uint_to_float(p, lim.p_min, lim.p_max, 16),
        uint_to_float(v, lim.v_min, lim.v_max, 12),
        uint_to_float(kp, lim.kp_min, lim.kp_max, 12),
        uint_to_float(kd, lim.kd_min, lim.kd_max, 12),
        uint_to_float(t, lim.t_min, lim.t_max, 12),
    )


def is_special(d: bytes) -> bool:
    return len(d) == 8 and all(b == 0xFF for b in d[:7]) and d[7] in SPECIAL_NAMES


def special_frame(code: int) -> bytes:
    return bytes([0xFF] * 7 + [code & 0xFF])


# ---------------- 状态反馈帧 ----------------
@dataclass
class Status:
    motor_id4: int      # 低 4 位 ID
    err: int            # 高 4 位错误码
    pos: float
    vel: float
    torque: float
    t_mos: int          # d6（手册表格称 v_margin，代码称 t_mos，待实机确认）
    t_rotor: int        # d7
    raw: bytes = b""


# 状态帧 d0 高 4 位状态码。实测：失能=0，使能=1（与达妙协议一致）；
# >=8 的故障含义参考达妙定义，未经厂家确认。
CODE_DISABLED = 0
CODE_ENABLED = 1
FAULT_NAMES = {0x8: "过压", 0x9: "欠压", 0xA: "过电流", 0xB: "MOS过温", 0xC: "线圈过温",
               0xD: "通讯丢失", 0xE: "过载"}


def is_fault(code: int) -> bool:
    return code >= 2


def code_text(code: int) -> str:
    if code < 0:
        return "未知"
    if code == CODE_DISABLED:
        return "失能"
    if code == CODE_ENABLED:
        return "已使能"
    name = FAULT_NAMES.get(code)
    return f"故障 {code}" + (f"({name}?)" if name else "")


def pack_status(motor_id: int, err: int, pos: float, vel: float, torque: float,
                t_mos: int, t_rotor: int, lim: Limits) -> bytes:
    p = float_to_uint(pos, lim.p_min, lim.p_max, 16)
    v = float_to_uint(vel, lim.v_min, lim.v_max, 12)
    t = float_to_uint(torque, lim.t_min, lim.t_max, 12)
    return bytes((
        ((err & 0x0F) << 4) | (motor_id & 0x0F),
        p >> 8, p & 0xFF,
        v >> 4, ((v & 0x0F) << 4) | ((t >> 8) & 0x0F),
        t & 0xFF,
        int(clamp(t_mos, 0, 255)), int(clamp(t_rotor, 0, 255)),
    ))


def unpack_status(d: bytes, lim: Limits) -> Status:
    p = (d[1] << 8) | d[2]
    v = (d[3] << 4) | (d[4] >> 4)
    t = ((d[4] & 0x0F) << 8) | d[5]
    return Status(
        motor_id4=d[0] & 0x0F,
        err=(d[0] >> 4) & 0x0F,
        pos=uint_to_float(p, lim.p_min, lim.p_max, 16),
        vel=uint_to_float(v, lim.v_min, lim.v_max, 12),
        torque=uint_to_float(t, lim.t_min, lim.t_max, 12),
        t_mos=d[6], t_rotor=d[7], raw=bytes(d),
    )


# ---------------- 寄存器表 ----------------
@dataclass(frozen=True)
class Register:
    rid: int
    name: str
    typ: str            # 'f' float32 / 'u' uint32 / 'i' int32
    desc: str
    ro: bool = False
    unit: str = ""


REGISTERS: list[Register] = [
    Register(0x00, "uv_value", "f", "低压保护值", unit="V"),
    Register(0x01, "kt", "f", "扭矩系数", unit="Nm/A"),
    Register(0x02, "temp_max", "f", "过温保护值", unit="℃"),
    Register(0x03, "i_fw_max", "f", "持续电流保护", unit="A"),
    Register(0x07, "can_master", "u", "反馈ID (MASTER_ID)"),
    Register(0x08, "can_id", "u", "接收ID (电机ID)"),
    Register(0x09, "can_timeout", "u", "超时警报时间(0为关闭)"),
    Register(0x0B, "damp", "f", "电机粘滞系数"),
    Register(0x0C, "inertia", "f", "电机转动惯量"),
    Register(0x0D, "motor_type", "i", "电机型号"),
    Register(0x0E, "config_version", "i", "软件版本号"),
    Register(0x10, "ppairs", "i", "电机极对数"),
    Register(0x11, "r_nominal", "f", "电机相电阻", unit="Ω"),
    Register(0x12, "ls", "f", "电机相电感", unit="H"),
    Register(0x13, "flux", "f", "电机磁链值", unit="Wb"),
    Register(0x14, "gr", "f", "齿轮减速比"),
    Register(0x15, "p_max", "f", "位置映射范围", unit="rad"),
    Register(0x16, "v_max", "f", "速度映射范围", unit="rad/s"),
    Register(0x17, "t_max", "f", "扭矩映射范围", unit="Nm"),
    Register(0x18, "i_bw", "f", "电流环控制带宽", unit="Hz"),
    Register(0x1D, "ov_value", "f", "过压保护值", unit="V"),
    Register(0x23, "can_baudrate", "i", "波特率索引(仅CAN FD版可改)", ro=True),
    Register(0x32, "adc_a_offset", "f", "A相零电流偏移"),
    Register(0x33, "adc_b_offset", "f", "B相零电流偏移"),
    Register(0x3B, "i_max", "f", "驱动板最大电流", unit="A"),
    Register(0x3C, "v_bus", "f", "电源电压", ro=True, unit="V"),
    Register(0x3D, "t_mos", "f", "驱动板温度", ro=True, unit="℃"),
    Register(0x3E, "t_rotor", "f", "电机温度", ro=True, unit="℃"),
    Register(0x50, "angle_singleturn", "f", "电机轴当前位置", ro=True, unit="rad"),
    Register(0x51, "output_angle_multiturn", "f", "输出轴位置", ro=True, unit="rad"),
]
REG_BY_RID = {r.rid: r for r in REGISTERS}
REG_BY_NAME = {r.name: r for r in REGISTERS}

_FMT = {"f": "<f", "u": "<I", "i": "<i"}


def encode_value(typ: str, value) -> bytes:
    if typ == "f":
        return struct.pack("<f", float(value))
    return struct.pack(_FMT[typ], int(value))


def decode_value(typ: str, raw: bytes):
    return struct.unpack(_FMT[typ], bytes(raw[:4]))[0]


# ---------------- 自定义协议帧 ----------------
def _hdr(motor_id: int, cmd: int) -> list[int]:
    return [motor_id & 0xFF, (motor_id >> 8) & 0xFF, cmd]


def read_reg_frame(motor_id: int, rid: int) -> bytes:
    return bytes(_hdr(motor_id, CMD_READ) + [rid & 0xFF, 0, 0, 0, 0])


def write_reg_frame(motor_id: int, rid: int, typ: str, value) -> bytes:
    return bytes(_hdr(motor_id, CMD_WRITE) + [rid & 0xFF]) + encode_value(typ, value)


def save_frame(motor_id: int) -> bytes:
    return bytes(_hdr(motor_id, CMD_SAVE) + [SAVE_SUB, 0, 0, 0, 0])


def query_state_frame(motor_id: int) -> bytes:
    return bytes(_hdr(motor_id, CMD_STATE) + [0, 0, 0, 0, 0])


def baud_frame(motor_id: int, index: int) -> bytes:
    return bytes(_hdr(motor_id, CMD_SAVE) + [BAUD_SUB, index & 0xFF, 0, 0, 0])


@dataclass
class ConfigReply:
    motor_id: int
    cmd: int
    sub: int            # RID 或 0xAA 的子命令
    raw4: bytes


def parse_config_reply(d: bytes) -> ConfigReply | None:
    if len(d) < 8 or d[2] not in CONFIG_CMDS:
        return None
    return ConfigReply(d[0] | (d[1] << 8), d[2], d[3], bytes(d[4:8]))


def matches_request(d: bytes, req: bytes) -> bool:
    """应答与请求的 d0..d3 一致（ID、命令、RID/子命令）。"""
    return len(d) >= 4 and len(req) >= 4 and bytes(d[:4]) == bytes(req[:4]) and d[2] in CONFIG_CMDS


def hexdata(d: bytes) -> str:
    return " ".join(f"{b:02X}" for b in d)
