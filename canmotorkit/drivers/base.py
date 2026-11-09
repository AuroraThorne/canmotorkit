"""电机驱动接口。

每种协议一个 Driver 子类，负责：声明能力、把「电流/扭矩指令」「MIT 指令」「使能/失能」等翻译成
CAN 帧，并把反馈帧解析成统一的 Feedback。驱动本身不做 IO。

单位约定（AGENTS.md）：输出量按厂家报文约定（达妙/HM 为 Nm，大疆为 A），不做换算；
位置 rad、速度 rad/s，均为输出轴侧。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

Frames = list[tuple[int, bytes]]
NAN = float("nan")


@dataclass(frozen=True)
class Caps:
    mit: bool = False           # 支持 MIT 控制帧
    registers: bool = False     # 支持参数读写
    enable_cmd: bool = False    # 有使能/失能命令；否则「使能」为软件开关
    zero: bool = False          # 支持设零点
    clear: bool = False         # 支持清错
    status_code: bool = False   # 反馈带状态码


@dataclass
class Feedback:
    pos: float                  # rad，电机反馈原值（可能单圈回绕）
    vel: float                  # rad/s
    out: float                  # 实际电流/扭矩（厂家单位）
    temp: float = NAN           # ℃
    temp2: float = NAN
    code: int = -1              # 状态码；-1 = 协议无状态码
    enabled: bool | None = None  # 电机报告的使能状态；None = 协议无此信息
    motor_id: int = -1
    raw: bytes = b""

    def max_temp(self) -> float:
        vals = [x for x in (self.temp, self.temp2) if not math.isnan(x)]
        return max(vals) if vals else 0.0


class Driver:
    protocol = ""
    protocol_name = ""
    caps = Caps()
    out_name = "扭矩"
    out_unit = "Nm"
    note = ""

    def __init__(self, model: str, motor_id: int):
        self.model = model
        self.motor_id = motor_id

    # ---- 属性 ----
    @property
    def ready(self) -> bool:
        """编解码所需参数是否已知（达妙/HM 需要先读量程）。"""
        return True

    @property
    def out_max(self) -> float:
        raise NotImplementedError

    @property
    def wrap_span(self) -> float:
        """反馈位置的回绕周期（rad），用于多圈累计。"""
        return 2 * math.pi

    def title(self) -> str:
        return f"{self.model}  ID {self.motor_id}"

    # ---- 解析 ----
    def accepts(self, can_id: int, data: bytes) -> bool:
        raise NotImplementedError

    def parse(self, can_id: int, data: bytes) -> Feedback | None:
        raise NotImplementedError

    # ---- 指令 ----
    def output_frames(self, out: float) -> Frames:
        raise NotImplementedError

    def stop_frames(self) -> Frames:
        return self.output_frames(0.0)

    def mit_frames(self, cmd) -> Frames:
        return []

    def idle_frames(self) -> Frames:
        """未使能且发送打开时按频率发送的帧（如状态查询）。"""
        return []

    def enable_frames(self) -> Frames:
        return []

    def disable_frames(self) -> Frames:
        return []

    def clear_frames(self) -> Frames:
        return []

    def zero_frames(self) -> Frames:
        return []

    def code_text(self, code: int) -> str:
        return ""

    def is_fault(self, code: int) -> bool:
        return False

    def describe(self, can_id: int, data: bytes, tx: bool) -> str:
        """报文日志注释；与本电机无关返回空串。"""
        return ""
