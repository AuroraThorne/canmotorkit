"""频响辨识、模型拟合与 PID 建议（单环，厂家单位）。

模型（输出轴侧，单位以厂家输出量 u 为准）：
    J·dv/dt = u − b·v − Fc·sign(v)，  并带纯延迟 Td
    电流→速度  Gv(s) = e^{-sTd} / (J s + b)
    电流→位置  Gp(s) = Gv(s) / s
这里 J、b、Fc 的单位是「输出量」单位：达妙/HM 为 Nm（即物理量），大疆为 A（等效 J/Kt）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .runs import ExciteRun, Record


@dataclass
class Bode:
    f: np.ndarray           # Hz
    mag_v: np.ndarray       # |速度/电流|
    ph_v: np.ndarray        # deg
    mag_p: np.ndarray       # |位置/电流|
    ph_p: np.ndarray
    coh: np.ndarray         # 0~1 拟合质量


@dataclass
class Model:
    J: float
    b: float
    Fc: float
    Td: float               # s
    unit: str = "Nm"

    def text(self) -> str:
        u = self.unit
        return (f"J = {self.J:.4g} {u}·s²/rad，b = {self.b:.4g} {u}·s/rad，"
                f"库仑摩擦 Fc = {self.Fc:.4g} {u}，等效延迟 Td = {self.Td * 1000:.2f} ms")


def _arr(rec: Record):
    t = np.asarray(rec.t, float)
    u = np.asarray(rec.u, float)
    v = np.asarray(rec.vel, float)
    p = np.asarray(rec.pos, float)
    ok = np.isfinite(t) & np.isfinite(u) & np.isfinite(v) & np.isfinite(p)
    return t[ok], u[ok], v[ok], p[ok], np.asarray(rec.seg)[ok]


def _sine_fit(t, y, f):
    """最小二乘拟合 y ≈ a·sin + b·cos + c + d·t，返回复振幅 (a + j b) 与 R²。"""
    w = 2 * math.pi * f
    A = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t), t - t.mean()])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ coef
    yv = y - (A[:, 2:] @ coef[2:])
    ss = float(np.sum(yv ** 2))
    r2 = 1 - float(np.sum(res ** 2)) / ss if ss > 0 else 0.0
    return complex(coef[0], coef[1]), max(0.0, r2)


def bode_stepsine(run: ExciteRun) -> Bode | None:
    t, u, v, p, seg = _arr(run.rec)
    rows = []
    for i, (a, b, f) in enumerate(run.segs):
        settle = a + 1.0 / f if (b - a) * f >= 3 else a + 0.3 * (b - a)
        m = (seg == i) & (t >= settle)
        if m.sum() < 8:
            continue
        tu, U, rv, V = t[m], None, None, None
        U, r2u = _sine_fit(tu, u[m], f)
        V, r2v = _sine_fit(tu, v[m], f)
        Pp, _ = _sine_fit(tu, p[m], f)
        if abs(U) < 1e-12:
            continue
        gv, gp = V / U, Pp / U
        rows.append((f, abs(gv), math.degrees(math.atan2(gv.imag, gv.real)), abs(gp),
                     math.degrees(math.atan2(gp.imag, gp.real)), min(r2u, r2v)))
    if len(rows) < 2:
        return None
    a = np.array(rows)
    return Bode(a[:, 0], a[:, 1], np.unwrap(np.radians(a[:, 2])) * 180 / math.pi, a[:, 3],
                np.unwrap(np.radians(a[:, 4])) * 180 / math.pi, a[:, 5])


def bode_broadband(run: ExciteRun, nper: int = 4) -> Bode | None:
    """扫频 / PRBS：重采样到均匀网格，用 H1 = Suv/Suu 估计（Welch 分段平均）。"""
    t, u, v, p, _ = _arr(run.rec)
    if len(t) < 64:
        return None
    fs = (len(t) - 1) / (t[-1] - t[0])
    tg = np.linspace(t[0], t[-1], len(t))
    ug, vg, pg = (np.interp(tg, t, x) for x in (u, v, p))
    vg = vg - vg.mean()
    pg = pg - np.polyval(np.polyfit(tg, pg, 1), tg)
    ug = ug - ug.mean()
    n = len(tg)
    L = max(64, n // nper)
    step = L // 2
    win = np.hanning(L)
    Suu = Suv = Sup = Svv = 0
    for s0 in range(0, n - L + 1, step):
        U = np.fft.rfft(win * ug[s0:s0 + L])
        V = np.fft.rfft(win * vg[s0:s0 + L])
        Pq = np.fft.rfft(win * pg[s0:s0 + L])
        Suu = Suu + np.abs(U) ** 2
        Suv = Suv + np.conj(U) * V
        Sup = Sup + np.conj(U) * Pq
        Svv = Svv + np.abs(V) ** 2
    f = np.fft.rfftfreq(L, 1 / fs)
    c = run.cfg
    m = (f >= c.f0) & (f <= c.f1) & (Suu > 1e-12 * np.max(Suu))
    if m.sum() < 3:
        return None
    Gv, Gp = Suv[m] / Suu[m], Sup[m] / Suu[m]
    coh = np.abs(Suv[m]) ** 2 / (Suu[m] * Svv[m] + 1e-30)
    # 对数分箱降点，约 20 点
    fm = f[m]
    edges = np.geomspace(fm[0], fm[-1] * 1.0001, 21)
    rows = []
    for a, b in zip(edges[:-1], edges[1:]):
        k = (fm >= a) & (fm < b)
        if k.any():
            gv, gp = Gv[k].mean(), Gp[k].mean()
            rows.append((fm[k].mean(), abs(gv), math.degrees(np.angle(gv)), abs(gp),
                         math.degrees(np.angle(gp)), float(coh[k].mean())))
    if len(rows) < 3:
        return None
    a = np.array(rows)
    return Bode(a[:, 0], a[:, 1], np.unwrap(np.radians(a[:, 2])) * 180 / math.pi, a[:, 3],
                np.unwrap(np.radians(a[:, 4])) * 180 / math.pi, a[:, 5])


def bode_from_run(run: ExciteRun) -> Bode | None:
    if run.cfg.kind == "stepsine":
        return bode_stepsine(run)
    if run.cfg.kind in ("chirp", "prbs"):
        return bode_broadband(run)
    return None


def fit_model(bode: Bode, unit: str = "Nm") -> Model | None:
    """用 1/G = J·jω + b 的幅值拟合 J、b；相位残差斜率估计延迟。"""
    ok = bode.coh > 0.5
    if ok.sum() < 2:
        ok = np.ones_like(bode.f, bool)
    w = 2 * np.pi * bode.f[ok]
    mag = bode.mag_v[ok]
    inv2 = 1.0 / mag ** 2                   # = J²ω² + b²
    A = np.column_stack([w ** 2, np.ones_like(w)])
    sol, *_ = np.linalg.lstsq(A, inv2, rcond=None)
    J = math.sqrt(max(sol[0], 1e-18))
    b = math.sqrt(max(sol[1], 0.0))
    ph_model = -np.degrees(np.arctan2(J * w, b))
    res = np.radians(bode.ph_v[ok] - ph_model)
    res = (res + np.pi) % (2 * np.pi) - np.pi
    Td = float(-np.sum(res * w) / np.sum(w * w)) if len(w) else 0.0
    return Model(J, b, 0.0, max(0.0, Td), unit)


def step_model(rec: Record, amp: float, delay: float, on: float, unit: str = "Nm") -> Model | None:
    """阶跃激励：用速度响应做 v' = (u − Fc·sgn v − b v)/J 的线性回归。"""
    t = np.asarray(rec.t)
    u = np.asarray(rec.u)
    v = np.asarray(rec.vel)
    if len(t) < 20:
        return None
    tg = np.linspace(t[0], t[-1], len(t))
    vg = np.interp(tg, t, v)
    ug = np.interp(tg, t, u)
    dt = tg[1] - tg[0]
    k = max(1, int(0.004 / dt))
    vs = np.convolve(vg, np.ones(2 * k + 1) / (2 * k + 1), mode="same")
    a = np.gradient(vs, dt)
    m = (np.abs(vs) > 0.05 * max(np.max(np.abs(vs)), 1e-9))
    m[:k] = m[-k:] = False
    if m.sum() < 10:
        return None
    # u = J a + b v + Fc sgn(v)
    A = np.column_stack([a[m], vs[m], np.sign(vs[m])])
    sol, *_ = np.linalg.lstsq(A, ug[m], rcond=None)
    J, b, Fc = (float(x) for x in sol)
    if J <= 0:
        return None
    return Model(J, max(b, 0.0), max(Fc, 0.0), 0.0, unit)


@dataclass
class Suggestion:
    vel_kp: float
    vel_ki: float
    vel_bw: float           # Hz
    pos_kp: float
    pos_kd: float
    pos_ki: float
    pos_bw: float           # Hz
    unit: str
    note: str

    def text(self) -> str:
        u = self.unit
        return (f"速度环 PI（带宽 {self.vel_bw:.1f} Hz）：Kp = {self.vel_kp:.4g} {u}/(rad/s)，"
                f"Ki = {self.vel_ki:.4g} {u}/rad\n"
                f"位置环 PID（带宽 {self.pos_bw:.1f} Hz）：Kp = {self.pos_kp:.4g} {u}/rad，"
                f"Kd = {self.pos_kd:.4g} {u}/(rad/s)，Ki = {self.pos_ki:.4g} {u}/(rad·s)\n"
                + self.note)


def suggest(model: Model, latency: float = 0.003, rate_hz: float = 500.0,
            pm_deg: float = 55.0) -> Suggestion:
    """按相位裕度约束选带宽，给出速度 PI 与位置 PID（均为单环：输出即电流/扭矩）。

    总延迟 = 模型延迟 + 实测通信延迟 + 半个发送周期。
    速度环：对象 1/(Js+b)，PI 零点放在 ωc/5，开环在 ωc 处相位约 −90°−11°−ωc·T。
    位置环：对象 1/(J s²)，PD 的 Kd/Kp = 1/ωz，ωz = ωc/3，再加小积分。
    """
    T = max(0.0, model.Td) + max(0.0, latency) + 0.5 / max(rate_hz, 1.0)
    # 速度环：90° + 11.3°(PI) + ωT = 180 − PM
    budget_v = math.radians(180 - pm_deg - 90 - 11.3)
    wv = budget_v / T if T > 0 else 2 * math.pi * 50
    wv = min(wv, 2 * math.pi * rate_hz / 10)
    J, b = model.J, model.b
    vkp = math.hypot(J * wv, b)
    vki = vkp * wv / 5
    # 位置环 PD：双积分 180° − PD 超前 atan(3) ≈ 71.6° + ωT = 180 − PM
    budget_p = math.radians(71.6 - pm_deg)
    wp = budget_p / T if T > 0 and budget_p > 0 else wv / 4
    wp = min(wp, wv / 2)
    wz = wp / 3
    mag = math.hypot(1, wp / wz)
    pkp = J * wp ** 2 / mag
    pkd = pkp / wz
    pki = pkp * wz / 10
    note = (f"依据：{model.text()}；总延迟估计 {T * 1000:.1f} ms（含通信 {latency * 1000:.1f} ms、"
            f"半周期 {500 / rate_hz:.1f} ms），目标相位裕度 {pm_deg:.0f}°。"
            "库仑摩擦未补偿；建议从建议值的 50% 起步。")
    return Suggestion(vkp, vki, wv / (2 * math.pi), pkp, pkd, pki, wp / (2 * math.pi), model.unit, note)


@dataclass
class StepStats:
    rise: float | None
    overshoot: float
    settle: float | None
    ss_err: float
    rms_err: float

    def text(self) -> str:
        rt = f"{self.rise * 1000:.1f} ms" if self.rise is not None else "—"
        st = f"{self.settle * 1000:.1f} ms" if self.settle is not None else "—"
        return (f"上升(10-90%) {rt}，超调 {self.overshoot:.1f}%，调节(2%) {st}，"
                f"稳态误差 {self.ss_err:+.4g}，跟踪 RMS {self.rms_err:.4g}")


def loop_stats(rec: Record, loop: str) -> StepStats | None:
    t = np.asarray(rec.t)
    r = np.asarray(rec.ref, float)
    y = np.asarray(rec.vel if loop == "vel" else rec.pos, float)
    ok = np.isfinite(r) & np.isfinite(y)
    t, r, y = t[ok], r[ok], y[ok]
    if len(t) < 10:
        return None
    e = r - y
    rms = float(np.sqrt(np.mean(e ** 2)))
    dr = np.abs(np.diff(r))
    k0 = int(np.argmax(dr)) + 1
    span = r[k0] - y[max(0, k0 - 1)]
    if dr.max() < 1e-12 or abs(span) < 1e-12:
        return StepStats(None, 0.0, None, float(e[-max(3, len(e) // 10):].mean()), rms)
    y0 = y[max(0, k0 - 1)]
    yn = (y[k0:] - y0) / span
    tt = t[k0:]
    i10 = np.argmax(yn >= 0.1) if np.any(yn >= 0.1) else None
    i90 = np.argmax(yn >= 0.9) if np.any(yn >= 0.9) else None
    rise = float(tt[i90] - tt[i10]) if i10 is not None and i90 is not None else None
    over = max(0.0, float(yn.max() - 1) * 100)
    out = np.nonzero(np.abs(yn - 1) > 0.02)[0]
    settle = None if len(out) and out[-1] == len(yn) - 1 else float(
        (tt[out[-1] + 1] if len(out) else tt[0]) - t[k0])
    tail = e[-max(3, len(e) // 10):]
    return StepStats(rise, over, settle, float(tail.mean()), rms)
