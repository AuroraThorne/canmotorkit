"""测试与闭环的「运行」对象：在 Worker 线程内按发送频率调用 output() 得到电流/扭矩指令，
并由 record() 记录反馈。单位按厂家约定（达妙/HM Nm，大疆 A）。

- ExciteRun：开环电流激励（阶跃/步进正弦/扫频/PRBS），可叠加弱速度偏置环以越过静摩擦。
- LoopRun：上位机单环闭环（速度 PI 或 位置 PID），参考信号为阶跃/斜坡/正弦。
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

NAN = float("nan")


@dataclass
class Record:
    t: list = field(default_factory=list)
    u: list = field(default_factory=list)       # 实际下发电流/扭矩
    ref: list = field(default_factory=list)     # 激励或参考
    pos: list = field(default_factory=list)     # 多圈累计
    vel: list = field(default_factory=list)
    fb_out: list = field(default_factory=list)  # 反馈电流/扭矩
    seg: list = field(default_factory=list)     # 步进正弦段号（其它 -1）

    def __len__(self):
        return len(self.t)


class Run:
    title = ""

    def __init__(self):
        self.rec = Record()
        self.t0 = 0.0
        self.ref = NAN
        self.done = False
        self.reason = ""
        self.seg = -1

    def start(self, t0: float, pos0: float, vel0: float):
        self.t0, self.pos0, self.vel0 = t0, pos0, vel0

    def output(self, now: float, fb, pos_mt: float) -> float | None:
        raise NotImplementedError

    def record(self, t: float, u: float, fb, pos_mt: float):
        r = self.rec
        r.t.append(t - self.t0)
        r.u.append(u)
        r.ref.append(self.ref)
        r.pos.append(pos_mt)
        r.vel.append(fb.vel)
        r.fb_out.append(fb.out)
        r.seg.append(self.seg)

    def finish(self, reason: str = ""):
        self.done = True
        self.reason = reason


# ============================ 开环激励 ============================
EXCITE_KINDS = {"step": "阶跃", "stepsine": "步进正弦", "chirp": "扫频", "prbs": "PRBS"}


def log_freqs(f0: float, f1: float, n: int) -> list[float]:
    if n <= 1:
        return [f0]
    return [f0 * (f1 / f0) ** (i / (n - 1)) for i in range(n)]


@dataclass
class ExciteConfig:
    kind: str = "stepsine"
    amp: float = 0.5            # 幅值（厂家单位）
    offset: float = 0.0
    step_on: float = 0.3        # 阶跃持续时间 s
    step_delay: float = 0.2
    step_tail: float = 0.5      # 阶跃结束后继续记录 s（输出为 offset）
    f0: float = 1.0
    f1: float = 30.0
    n_freq: int = 10
    cycles: int = 6             # 步进正弦每个频点的周期数（前 1 个丢弃作过渡）
    min_seg: float = 0.3        # 每个频点最短时长 s
    duration: float = 5.0       # 扫频 / PRBS 时长
    prbs_dt: float = 0.01       # PRBS 码元宽度 s
    bias_vel: float = 0.0       # 偏置速度 rad/s：>0 时叠加弱速度环越过静摩擦
    bias_kp: float = 0.0        # 偏置速度环增益（单位 / (rad/s)）
    seed: int = 1

    def label(self) -> str:
        k = EXCITE_KINDS[self.kind]
        if self.kind == "step":
            return f"{k} {self.amp:g}，持续 {self.step_on:g} s"
        return f"{k} ±{self.amp:g}，{self.f0:g}~{self.f1:g} Hz"


class ExciteRun(Run):
    def __init__(self, cfg: ExciteConfig):
        super().__init__()
        self.cfg = cfg
        self.title = "响应测试：" + cfg.label()
        self.segs: list[tuple[float, float, float]] = []   # (开始, 结束, 频率)
        if cfg.kind == "stepsine":
            t = 0.0
            for f in log_freqs(cfg.f0, cfg.f1, cfg.n_freq):
                dur = max(cfg.cycles / f, cfg.min_seg)
                dur = math.ceil(dur * f) / f          # 整数周期
                self.segs.append((t, t + dur, f))
                t += dur
            self.total = t
        elif cfg.kind == "step":
            self.total = cfg.step_delay + cfg.step_on + cfg.step_tail
        else:
            self.total = cfg.duration
        rnd = random.Random(cfg.seed)
        n = int(self.total / cfg.prbs_dt) + 2
        self._prbs = [1.0 if rnd.random() < 0.5 else -1.0 for _ in range(n)]
        self._vbias_i = 0.0

    def excitation(self, t: float) -> float:
        c = self.cfg
        self.seg = -1
        if c.kind == "step":
            return c.amp if c.step_delay <= t < c.step_delay + c.step_on else 0.0
        if c.kind == "stepsine":
            for i, (a, b, f) in enumerate(self.segs):
                if a <= t < b:
                    self.seg = i
                    return c.amp * math.sin(2 * math.pi * f * (t - a))
            return 0.0
        if c.kind == "chirp":
            T = self.total
            k = math.log(c.f1 / c.f0) / T          # 对数扫频
            ph = 2 * math.pi * c.f0 * (math.exp(k * t) - 1) / k
            return c.amp * math.sin(ph)
        if c.kind == "prbs":
            return c.amp * self._prbs[int(t / c.prbs_dt)]
        return 0.0

    def output(self, now, fb, pos_mt):
        t = now - self.t0
        if t >= self.total:
            self.ref = NAN
            return None
        x = self.excitation(t)
        self.ref = x
        u = self.cfg.offset + x
        if self.cfg.bias_vel and self.cfg.bias_kp and fb is not None:
            u += self.cfg.bias_kp * (self.cfg.bias_vel - fb.vel)
        return u


# ============================ 上位机闭环 ============================
LOOP_KINDS = {"vel": "速度环（PI）", "pos": "位置环（PID）"}
REF_KINDS = {"step": "阶跃", "ramp": "斜坡", "sine": "正弦"}


@dataclass
class LoopConfig:
    loop: str = "vel"           # vel / pos
    kp: float = 0.1
    ki: float = 0.0
    kd: float = 0.0
    ref_kind: str = "step"
    amp: float = 2.0            # 阶跃幅值 / 斜坡终值 / 正弦幅值（rad/s 或 rad）
    freq: float = 1.0           # 正弦频率
    ramp_time: float = 1.0
    delay: float = 0.2
    duration: float = 2.0
    out_max: float = 1.0        # 控制器输出限幅（另有安全限幅）
    relative: bool = True       # 位置环参考相对起始位置
    d_on_meas: bool = True      # 位置环微分作用于测量速度（避免阶跃冲击）

    def label(self) -> str:
        return (f"{LOOP_KINDS[self.loop]} {REF_KINDS[self.ref_kind]} {self.amp:g} "
                f"Kp={self.kp:g} Ki={self.ki:g} Kd={self.kd:g}")


class LoopRun(Run):
    def __init__(self, cfg: LoopConfig):
        super().__init__()
        self.cfg = cfg
        self.title = "闭环测试：" + cfg.label()
        self.integ = 0.0
        self.prev_t: float | None = None
        self.sat = 0
        self.n = 0

    def reference(self, t: float) -> float:
        c = self.cfg
        if t < c.delay:
            x = 0.0
        elif c.ref_kind == "step":
            x = c.amp
        elif c.ref_kind == "ramp":
            x = c.amp * min(1.0, (t - c.delay) / max(c.ramp_time, 1e-6))
        else:
            x = c.amp * math.sin(2 * math.pi * c.freq * (t - c.delay))
        if c.loop == "pos" and c.relative:
            x += self.pos0
        return x

    def output(self, now, fb, pos_mt):
        c = self.cfg
        t = now - self.t0
        if t >= c.duration:
            self.ref = NAN
            return None
        if fb is None:
            return 0.0
        r = self.reference(t)
        self.ref = r
        dt = 0.0 if self.prev_t is None else max(0.0, now - self.prev_t)
        self.prev_t = now
        if c.loop == "vel":
            e = r - fb.vel
            d = 0.0
        else:
            e = r - pos_mt
            d = -fb.vel if c.d_on_meas else 0.0
        u_pi = c.kp * e + c.ki * self.integ + c.kd * d
        u = max(-c.out_max, min(c.out_max, u_pi))
        self.n += 1
        if u != u_pi:
            self.sat += 1
        else:
            self.integ += e * dt      # 饱和时停止积分（抗饱和）
        return u

    @property
    def sat_ratio(self) -> float:
        return self.sat / self.n if self.n else 0.0
