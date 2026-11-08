"""测试信号发生器与阶跃响应统计。"""
from __future__ import annotations

import math
from dataclasses import dataclass

KINDS = {"const": "恒定", "step": "阶跃", "square": "方波", "sine": "正弦", "chirp": "扫频"}


@dataclass
class SignalGen:
    kind: str = "step"
    amp: float = 0.5
    offset: float = 0.0
    freq: float = 1.0           # 正弦/方波频率，扫频起始频率 [Hz]
    freq_end: float = 10.0      # 扫频终止频率 [Hz]
    duration: float = 3.0       # [s]
    delay: float = 0.5          # 阶跃延时 [s]

    def value(self, t: float) -> float | None:
        """t 为开始后的时间；超过 duration 返回 None。"""
        if t > self.duration:
            return None
        k = self.kind
        if k == "const":
            return self.offset + self.amp
        if k == "step":
            return self.offset + (self.amp if t >= self.delay else 0.0)
        if k == "square":
            return self.offset + (self.amp if math.sin(2 * math.pi * self.freq * t) >= 0 else -self.amp)
        if k == "sine":
            return self.offset + self.amp * math.sin(2 * math.pi * self.freq * t)
        if k == "chirp":
            T = max(self.duration, 1e-6)
            ph = 2 * math.pi * (self.freq * t + (self.freq_end - self.freq) * t * t / (2 * T))
            return self.offset + self.amp * math.sin(ph)
        raise ValueError(k)

    def final(self) -> float:
        """结束后保持的值。"""
        if self.kind in ("const", "step"):
            return self.offset + self.amp
        return self.offset


@dataclass
class StepMetrics:
    initial: float
    target: float
    rise_time: float | None     # 10%→90%
    overshoot_pct: float
    ss_error: float
    settle_time: float | None   # 进入并保持在 ±2% 带内

    def text(self) -> str:
        rt = f"{self.rise_time * 1000:.1f} ms" if self.rise_time is not None else "—"
        st = f"{self.settle_time * 1000:.1f} ms" if self.settle_time is not None else "—"
        return (f"阶跃 {self.initial:.4f} → {self.target:.4f} | 上升时间 {rt} | "
                f"超调 {self.overshoot_pct:.1f}% | 调节时间(2%) {st} | 稳态误差 {self.ss_error:+.4f}")


def step_metrics(t: list[float], cmd: list[float], fb: list[float]) -> StepMetrics | None:
    """根据指令中最大的一次跳变计算阶跃响应指标（忽略开始时的微小目标调整）。"""
    n = len(t)
    if n < 10:
        return None
    k0 = max(range(1, n), key=lambda i: abs(cmd[i] - cmd[i - 1]))
    if abs(cmd[k0] - cmd[k0 - 1]) < 1e-9:
        return None
    t0, y0, r = t[k0], fb[max(0, k0 - 1)], cmd[k0]
    span = r - y0
    if abs(span) < 1e-9:
        return None
    tt = t[k0:]
    yy = [(v - y0) / span for v in fb[k0:]]     # 归一化到 0→1
    t10 = next((tt[i] for i, v in enumerate(yy) if v >= 0.1), None)
    t90 = next((tt[i] for i, v in enumerate(yy) if v >= 0.9), None)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
    over = max(0.0, (max(yy) - 1.0) * 100.0)
    tail = fb[n - max(3, (n - k0) // 10):]
    ss = r - sum(tail) / len(tail)
    settle = None
    for i in range(len(yy) - 1, -1, -1):
        if abs(yy[i] - 1.0) > 0.02:
            settle = (tt[i + 1] - t0) if i + 1 < len(tt) else None
            break
    else:
        settle = 0.0
    return StepMetrics(y0, r, rise, over, ss, settle)
