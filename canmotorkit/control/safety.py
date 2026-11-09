"""安全监督：速度 / 位置（多圈累计）/ 电流或扭矩 / 温度 / 反馈看门狗。

约定（见 AGENTS.md）：
- 超限或急停时输出零电流/零扭矩，不失能（达妙/HM 失能会猛刹车）。
- 触发后锁存（tripped），必须用户显式 reset 才恢复输出。
- 单位按厂家报文约定：电流限制即对 MIT t（Nm）或大疆电流（A）的限幅。
纯逻辑，无 Qt / IO，便于测试。
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class SafetyLimits:
    vel_max: float = 10.0          # |速度| 上限 rad/s；0 = 不检查
    pos_enabled: bool = False      # 是否启用位置窗口
    pos_min: float = -6.0          # 多圈累计位置窗口 rad（相对累计原点）
    pos_max: float = 6.0
    out_max: float = 1.0           # |电流/扭矩指令| 限幅（厂家单位）；超出则限幅而非触发
    temp_max: float = 80.0         # ℃；0 = 不检查
    fb_timeout: float = 0.1        # 输出非零时反馈超时 s；0 = 不检查


class MultiTurn:
    """把单圈位置（在 [-span/2, span/2) 内回绕）累计为多圈位置。"""

    def __init__(self, span: float = 2 * math.pi):
        self.span = span
        self.last: float | None = None
        self.turns = 0

    def reset(self):
        self.last = None
        self.turns = 0

    def update(self, raw: float) -> float:
        if self.last is not None:
            d = raw - self.last
            if d > self.span / 2:
                self.turns -= 1
            elif d < -self.span / 2:
                self.turns += 1
        self.last = raw
        return raw + self.turns * self.span


class Supervisor:
    def __init__(self, limits: SafetyLimits | None = None):
        self.limits = limits or SafetyLimits()
        self.mt = MultiTurn()
        self.tripped = False
        self.reason = ""
        self.pos = 0.0              # 多圈累计位置
        self.last_fb_t: float | None = None

    def set_wrap_span(self, span: float):
        """反馈位置回绕周期。达妙/HM 单圈 2π；若 p_max>π 则按 2*p_max。"""
        self.mt.span = span
        self.mt.reset()

    def trip(self, reason: str):
        if not self.tripped:
            self.tripped = True
            self.reason = reason

    def reset(self) -> bool:
        """用户解除锁存。若当前仍超限则拒绝。"""
        why = self._check_state(self._vel, self._temp)
        if why:
            self.reason = why
            return False
        self.tripped = False
        self.reason = ""
        return True

    _vel = 0.0
    _temp = 0.0

    def _check_state(self, vel: float, temp: float) -> str:
        L = self.limits
        if L.vel_max > 0 and abs(vel) > L.vel_max:
            return f"速度超限 {vel:+.3f} rad/s（限 ±{L.vel_max:g}）"
        if L.pos_enabled and not (L.pos_min <= self.pos <= L.pos_max):
            return f"位置超限 {self.pos:+.3f} rad（窗口 [{L.pos_min:g}, {L.pos_max:g}]）"
        if L.temp_max > 0 and temp > L.temp_max:
            return f"温度超限 {temp:.0f} ℃（限 {L.temp_max:g}）"
        return ""

    def on_feedback(self, t: float, raw_pos: float, vel: float, temp: float):
        self.last_fb_t = t
        self.pos = self.mt.update(raw_pos)
        self._vel, self._temp = vel, temp
        why = self._check_state(vel, temp)
        if why:
            self.trip(why)

    def zero_position(self):
        """把当前累计位置作为位置窗口的参考零点。"""
        if self.mt.last is not None:
            self.mt.turns = 0
            self.pos = self.mt.last

    def filter_output(self, now: float, out: float) -> float:
        """返回允许输出的电流/扭矩。已触发 → 0；否则限幅。"""
        L = self.limits
        if (not self.tripped and L.fb_timeout > 0 and out != 0.0 and self.last_fb_t is not None
                and now - self.last_fb_t > L.fb_timeout):
            self.trip(f"反馈超时 >{L.fb_timeout * 1000:.0f} ms")
        if self.tripped:
            return 0.0
        if L.out_max > 0:
            out = max(-L.out_max, min(L.out_max, out))
        return out
