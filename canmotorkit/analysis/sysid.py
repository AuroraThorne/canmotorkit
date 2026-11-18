"""频响辨识、模型拟合与串级控制器建议（厂家单位）。

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
    coh: np.ndarray         # 0~1，电流→速度拟合质量
    coh_p: np.ndarray       # 0~1，电流→位置拟合质量


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


def _run_arr(run: ExciteRun):
    rec = run.snapshot_record() if hasattr(run, "snapshot_record") else run.rec
    return _arr(rec)


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
    t, u, v, p, seg = _run_arr(run)
    rows = []
    for i, (a, b, f) in enumerate(run.segs):
        settle = a + 1.0 / f if (b - a) * f >= 3 else a + 0.3 * (b - a)
        m = (seg == i) & (t >= settle)
        if m.sum() < 8:
            continue
        tu, U, rv, V = t[m], None, None, None
        U, r2u = _sine_fit(tu, u[m], f)
        V, r2v = _sine_fit(tu, v[m], f)
        Pp, r2p = _sine_fit(tu, p[m], f)
        if abs(U) < 1e-12:
            continue
        gv, gp = V / U, Pp / U
        rows.append((f, abs(gv), math.degrees(math.atan2(gv.imag, gv.real)), abs(gp),
                     math.degrees(math.atan2(gp.imag, gp.real)), min(r2u, r2v),
                     min(r2u, r2p)))
    if len(rows) < 2:
        return None
    a = np.array(rows)
    return Bode(a[:, 0], a[:, 1], np.unwrap(np.radians(a[:, 2])) * 180 / math.pi, a[:, 3],
                np.unwrap(np.radians(a[:, 4])) * 180 / math.pi, a[:, 5], a[:, 6])


def bode_broadband(run: ExciteRun, nper: int = 4) -> Bode | None:
    """扫频 / PRBS：重采样到均匀网格，用 H1 = Suv/Suu 估计（Welch 分段平均）。"""
    t, u, v, p, _ = _run_arr(run)
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
    Suu = Suv = Sup = Svv = Spp = 0
    for s0 in range(0, n - L + 1, step):
        U = np.fft.rfft(win * ug[s0:s0 + L])
        V = np.fft.rfft(win * vg[s0:s0 + L])
        Pq = np.fft.rfft(win * pg[s0:s0 + L])
        Suu = Suu + np.abs(U) ** 2
        Suv = Suv + np.conj(U) * V
        Sup = Sup + np.conj(U) * Pq
        Svv = Svv + np.abs(V) ** 2
        Spp = Spp + np.abs(Pq) ** 2
    f = np.fft.rfftfreq(L, 1 / fs)
    c = run.cfg
    m = (f >= c.f0) & (f <= c.f1) & (Suu > 1e-12 * np.max(Suu))
    if m.sum() < 3:
        return None
    Gv, Gp = Suv[m] / Suu[m], Sup[m] / Suu[m]
    coh_v = np.abs(Suv[m]) ** 2 / (Suu[m] * Svv[m] + 1e-30)
    coh_p = np.abs(Sup[m]) ** 2 / (Suu[m] * Spp[m] + 1e-30)
    # 对数分箱降点，约 20 点
    fm = f[m]
    edges = np.geomspace(fm[0], fm[-1] * 1.0001, 21)
    rows = []
    for a, b in zip(edges[:-1], edges[1:]):
        k = (fm >= a) & (fm < b)
        if k.any():
            gv, gp = Gv[k].mean(), Gp[k].mean()
            rows.append((fm[k].mean(), abs(gv), math.degrees(np.angle(gv)), abs(gp),
                         math.degrees(np.angle(gp)), float(coh_v[k].mean()),
                         float(coh_p[k].mean())))
    if len(rows) < 3:
        return None
    a = np.array(rows)
    return Bode(a[:, 0], a[:, 1], np.unwrap(np.radians(a[:, 2])) * 180 / math.pi, a[:, 3],
                np.unwrap(np.radians(a[:, 4])) * 180 / math.pi, a[:, 5], a[:, 6])


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
    start_ratio: float = 1.0
    phase_margin: float = 0.0
    vel_phase_margin: float = 0.0
    pos_phase_margin: float = 0.0
    single_pos_kp: float = 0.0
    single_pos_ki: float = 0.0
    single_pos_kd: float = 0.0
    single_pos_bw: float = 0.0
    single_pos_phase_margin: float = 0.0

    def text(self) -> str:
        u = self.unit
        return (f"速度环 PI（带宽 {self.vel_bw:.1f} Hz）：Kp = {self.vel_kp:.4g} {u}/(rad/s)，"
                f"Ki = {self.vel_ki:.4g} {u}/rad\n"
                f"串级位置环外环 PID（带宽 {self.pos_bw:.1f} Hz）："
                f"Kp = {self.pos_kp:.4g} (rad/s)/rad，"
                f"Kd = {self.pos_kd:.4g} (rad/s)/(rad/s)，"
                f"Ki = {self.pos_ki:.4g} (rad/s)/(rad·s)\n"
                f"实际相位裕度：速度内环 {self.vel_phase_margin:.4g}°，"
                f"位置外环 {self.pos_phase_margin:.4g}°\n"
                + self.note.replace("50%", f"{self.start_ratio * 100:.0f}%"))


def _velocity_open_loop(model: Model, kp: float, ki: float, w: float,
                        delay: float) -> complex:
    s = 1j * w
    plant = np.exp(-s * delay) / (model.J * s + model.b)
    return (kp + ki / s) * plant


def _velocity_closed_loop(model: Model, kp: float, ki: float, w: float, delay: float) -> complex:
    """返回速度 PI 闭环（速度参考到速度反馈）的频响。"""
    s = 1j * w
    plant = np.exp(-s * delay) / (model.J * s + model.b)
    controller = kp + ki / s
    loop = controller * plant
    return loop / (1.0 + loop)


def _position_open_loop(model: Model, kp: float, ki: float, kd: float,
                        w: float, delay: float) -> complex:
    """位置单环开环：位置 PID 直接输出电流/扭矩。"""
    s = 1j * w
    plant = np.exp(-s * delay) / ((model.J * s + model.b) * s)
    return (kp + ki / s + kd * s) * plant


def _position_closed_loop(model: Model, kp: float, ki: float, kd: float,
                          w: float, delay: float) -> complex:
    loop = _position_open_loop(model, kp, ki, kd, w, delay)
    return loop / (1.0 + loop)


def _phase_margin(loop_fn, rate_hz: float) -> float:
    """从开环 0 dB 交越点计算实际相位裕度。"""
    fmax = max(1.0, rate_hz / 2.0)
    ws = np.geomspace(2.0 * math.pi * 1.0e-3, 2.0 * math.pi * fmax, 2400)
    vals = np.asarray([loop_fn(float(w)) for w in ws], dtype=complex)
    mag = np.abs(vals)
    log_mag = np.log(np.maximum(mag, 1.0e-30))
    phase = np.unwrap(np.angle(vals))
    crossings = np.flatnonzero((log_mag[:-1] >= 0.0) & (log_mag[1:] < 0.0))
    if len(crossings):
        i = int(crossings[0])
    else:
        i = int(np.argmin(np.abs(log_mag)))
        return float(180.0 + math.degrees(phase[i]))
    a, b = log_mag[i], log_mag[i + 1]
    ratio = 0.0 if abs(b - a) < 1.0e-12 else -a / (b - a)
    ratio = max(0.0, min(1.0, ratio))
    phase_at_cross = phase[i] + ratio * (phase[i + 1] - phase[i])
    return float(180.0 + math.degrees(phase_at_cross))


def _closed_loop_bandwidth_from(loop_fn, target_w: float, rate_hz: float) -> float:
    low = max(1.0e-3, target_w / 1000.0)
    high = max(target_w * 10.0, 2.0 * math.pi * rate_hz / 2.0)
    ws = np.geomspace(low, high, 1600)
    mag = np.abs([loop_fn(float(w)) for w in ws])
    hit = np.flatnonzero(mag <= 1.0 / math.sqrt(2.0))
    if not len(hit):
        return target_w
    i = int(hit[0])
    if i == 0:
        return float(ws[0])
    a, b = math.log(ws[i - 1]), math.log(ws[i])
    ma, mb = mag[i - 1], mag[i]
    if abs(mb - ma) < 1.0e-12:
        return float(ws[i])
    ratio = ((1.0 / math.sqrt(2.0)) - ma) / (mb - ma)
    return float(math.exp(a + max(0.0, min(1.0, ratio)) * (b - a)))


def _single_position_gains(model: Model, crossover_w: float, delay: float,
                           pm_deg: float) -> tuple[float, float, float]:
    """按目标交越频率设计位置单环 PD，Ki 保持为 0。

    位置对象自身含一个积分环节，PD 已能实现阶跃无静差；先不加入
    积分项，避免摩擦和输出限幅造成积分累积。
    """
    s = 1j * crossover_w
    plant = np.exp(-s * delay) / ((model.J * s + model.b) * s)
    phase_p = math.atan2(plant.imag, plant.real)
    desired_c_phase = math.radians(-180.0 + pm_deg) - phase_p
    desired_c_phase = max(math.radians(1.0), min(math.radians(89.0), desired_c_phase))
    lead = math.tan(desired_c_phase)
    kp = 1.0 / (abs(plant) * math.sqrt(1.0 + lead * lead))
    kd = kp * lead / crossover_w
    return float(kp), 0.0, float(kd)


def _single_position_pid(model: Model, delay: float, rate_hz: float,
                         target_bw_hz: float, pm_deg: float) -> tuple[float, float, float, float, float]:
    target_w = 2.0 * math.pi * max(0.01, target_bw_hz)
    lo = 2.0 * math.pi * 0.01
    hi = 2.0 * math.pi * rate_hz / 10.0

    def actual(crossover_w):
        gains = _single_position_gains(model, crossover_w, delay, pm_deg)
        return _closed_loop_bandwidth_from(
            lambda w: _position_closed_loop(model, *gains, w, delay),
            crossover_w, rate_hz)

    if target_w <= actual(lo):
        crossover = lo
    elif target_w >= actual(hi):
        crossover = hi
    else:
        for _ in range(48):
            mid = (lo + hi) / 2.0
            if actual(mid) < target_w:
                lo = mid
            else:
                hi = mid
        crossover = (lo + hi) / 2.0
    kp, ki, kd = _single_position_gains(model, crossover, delay, pm_deg)
    bw = _closed_loop_bandwidth_from(
        lambda w: _position_closed_loop(model, kp, ki, kd, w, delay),
        crossover, rate_hz)
    phase = _phase_margin(
        lambda w: _position_open_loop(model, kp, ki, kd, w, delay), rate_hz)
    return kp, ki, kd, bw / (2.0 * math.pi), phase


def _closed_loop_bandwidth(model: Model, kp: float, ki: float, delay: float,
                           target_w: float, rate_hz: float) -> float:
    """按 -3 dB 从内环模型计算实际速度闭环带宽。"""
    low = max(1.0e-3, target_w / 1000.0)
    high = max(target_w * 10.0, 2.0 * math.pi * rate_hz / 2.0)
    ws = np.geomspace(low, high, 1600)
    mag = np.abs([_velocity_closed_loop(model, kp, ki, w, delay) for w in ws])
    hit = np.flatnonzero(mag <= 1.0 / math.sqrt(2.0))
    if not len(hit):
        return target_w
    i = int(hit[0])
    if i == 0:
        return float(ws[0])
    a, b = math.log(ws[i - 1]), math.log(ws[i])
    ma, mb = mag[i - 1], mag[i]
    if abs(mb - ma) < 1.0e-12:
        return float(ws[i])
    ratio = ((1.0 / math.sqrt(2.0)) - ma) / (mb - ma)
    return float(math.exp(a + max(0.0, min(1.0, ratio)) * (b - a)))


def _velocity_gains(model: Model, crossover_w: float) -> tuple[float, float]:
    return (math.hypot(model.J * crossover_w, model.b),
            math.hypot(model.J * crossover_w, model.b) * crossover_w / 5.0)


def _crossover_for_velocity_bandwidth(model: Model, target_w: float,
                                      delay: float, rate_hz: float) -> float:
    """反求 PI 交越频率，使速度闭环 -3 dB 带宽接近期望值。"""
    lo = 2.0 * math.pi * 0.01
    hi = 2.0 * math.pi * rate_hz / 10.0

    def actual(crossover_w):
        kp, ki = _velocity_gains(model, crossover_w)
        return _closed_loop_bandwidth(model, kp, ki, delay, crossover_w, rate_hz)

    if target_w <= actual(lo):
        return lo
    if target_w >= actual(hi):
        return hi
    for _ in range(48):
        mid = (lo + hi) / 2.0
        if actual(mid) < target_w:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def _cascade_position_pid(model: Model, vel_kp: float, vel_ki: float,
                          vel_bw: float, delay: float, rate_hz: float,
                          pm_deg: float) -> tuple[float, float, float, float, float]:
    """基于实际速度闭环模型计算串级位置外环 PI/PID 建议。

    外环对象是 H_v(s)/s，其中 H_v 是内环速度闭环。先用外环 PI
    零点放在目标交越频率的 1/5 处，再按实际对象幅值归一化 Kp；
    Kd 初始为 0，避免位置反馈微分放大编码器噪声。
    """
    inner_bw = _closed_loop_bandwidth(model, vel_kp, vel_ki, delay, vel_bw, rate_hz)
    max_w = min(inner_bw / 3.0, 2.0 * math.pi * rate_hz / 20.0)
    if max_w <= 0.0:
        return 1.0, 0.0, 0.0, max(0.01, vel_bw / (2.0 * math.pi * 4.0)), 0.0
    min_w = max(max_w / 1000.0, 2.0 * math.pi * 0.05)
    ws = np.geomspace(min_w, max_w, 700)
    chosen = None
    fallback = None
    zero_ratio = 5.0
    for w in ws:
        hv = _velocity_closed_loop(model, vel_kp, vel_ki, float(w), delay)
        outer_plant = hv / (1j * w)
        normalized = 1.0 + 1.0 / (1j * zero_ratio)
        kp = 1.0 / max(abs(outer_plant * normalized), 1.0e-12)
        ki = kp * w / zero_ratio
        loop = (kp + ki / (1j * w)) * outer_plant
        pm = 180.0 + math.degrees(math.atan2(loop.imag, loop.real))
        candidate = (kp, 0.0, ki, w / (2.0 * math.pi), pm)
        fallback = candidate
        if pm >= pm_deg:
            chosen = candidate
    kp, kd, ki, bw, phase_margin = chosen or fallback
    return float(kp), float(kd), float(ki), float(bw), float(phase_margin)


def suggest(model: Model, latency: float = 0.003, rate_hz: float = 500.0,
            pm_deg: float = 55.0, vel_bw_hz: float | None = None,
            start_ratio: float = 1.0,
            single_pos_bw_hz: float | None = None) -> Suggestion:
    """按相位裕度约束生成内环速度 PI 与串级位置外环 PID 建议。

    总延迟 = 模型延迟 + 实测通信延迟 + 半个发送周期。
    速度环：对象 1/(Js+b)，PI 零点放在 ωc/5，开环在 ωc 处相位约 −90°−11°−ωc·T。
    位置环：先组合速度 PI 与电机对象得到 H_v(s)，外环对象为 H_v(s)/s，
    再按目标相位裕度计算外环 PI；Kd 初始为 0。
    """
    T = max(0.0, model.Td) + max(0.0, latency) + 0.5 / max(rate_hz, 1.0)
    # 速度环：目标带宽由用户指定时优先使用，否则按延迟和相位裕度估计。
    if vel_bw_hz is None:
        budget_v = math.radians(180 - pm_deg - 90 - 11.3)
        wv = budget_v / T if T > 0 else 2 * math.pi * 50
        requested = None
    else:
        requested = max(0.01, float(vel_bw_hz))
        wv = _crossover_for_velocity_bandwidth(
            model, 2.0 * math.pi * requested, T, rate_hz)
    max_wv = 2 * math.pi * rate_hz / 10
    wv = min(wv, max_wv)
    J, b = model.J, model.b
    vkp, vki = _velocity_gains(model, wv)
    inner_bw = _closed_loop_bandwidth(model, vkp, vki, T, wv, rate_hz)
    pkp, pkd, pki, pos_bw, _ = _cascade_position_pid(
        model, vkp, vki, inner_bw, T, rate_hz, pm_deg)
    single_target = single_pos_bw_hz if single_pos_bw_hz is not None else pos_bw
    skp, ski, skd, single_bw, single_pm = _single_position_pid(
        model, T, rate_hz, single_target, pm_deg)
    vel_pm = _phase_margin(lambda w: _velocity_open_loop(model, vkp, vki, w, T), rate_hz)
    pos_pm = _phase_margin(
        lambda w: (pkp + pki / (1j * w) + pkd * 1j * w)
        * _velocity_closed_loop(model, vkp, vki, w, T) / (1j * w), rate_hz)
    actual_target = inner_bw / (2 * math.pi)
    requested_text = (f"速度环期望带宽 {requested:.1f} Hz；"
                      if requested is not None else "")
    note = (f"依据：{model.text()}；已组合内环速度 PI 的实际闭环模型，"
            f"内环 -3 dB 带宽约 {inner_bw / (2 * math.pi):.1f} Hz；"
            + requested_text +
            f"总延迟估计 {T * 1000:.1f} ms（含通信 {latency * 1000:.1f} ms、"
            f"半周期 {500 / rate_hz:.1f} ms），目标相位裕度 {pm_deg:.0f}°。"
            "库仑摩擦未补偿；内外环建议均从计算值的 50% 起步。")
    ratio = max(0.0, min(1.0, start_ratio))
    return Suggestion(vkp, vki, actual_target, pkp, pkd, pki,
                      pos_bw, model.unit, note, ratio, pos_pm, vel_pm, pos_pm,
                      skp, ski, skd, single_bw, single_pm)


def predicted_closed_loop(model: Model, suggestion: Suggestion,
                          latency: float = 0.003, rate_hz: float = 500.0,
                          fmin: float = 0.05, fmax: float | None = None,
                          count: int = 500) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """计算速度、串级位置、单环位置闭环幅频曲线（dB）。"""
    delay = max(0.0, model.Td) + max(0.0, latency) + 0.5 / max(rate_hz, 1.0)
    max_bw = max(suggestion.vel_bw, suggestion.pos_bw, suggestion.single_pos_bw, 0.1)
    if fmax is None:
        fmax = min(max(100.0, max_bw * 20.0), rate_hz / 2.0)
    fmax = max(fmin * 10.0, fmax)
    f = np.geomspace(max(1.0e-3, fmin), fmax, max(32, int(count)))
    inner = []
    position = []
    single_position = []
    for hz in f:
        w = 2.0 * math.pi * float(hz)
        hv = _velocity_closed_loop(model, suggestion.vel_kp, suggestion.vel_ki, w, delay)
        s = 1j * w
        outer_plant = hv / s
        outer_controller = suggestion.pos_kp + suggestion.pos_ki / s + suggestion.pos_kd * s
        hp = outer_controller * outer_plant / (1.0 + outer_controller * outer_plant)
        hs = _position_closed_loop(
            model, suggestion.single_pos_kp, suggestion.single_pos_ki,
            suggestion.single_pos_kd, w, delay)
        inner.append(abs(hv))
        position.append(abs(hp))
        single_position.append(abs(hs))
    db = lambda x: 20.0 * np.log10(np.maximum(np.asarray(x), 1.0e-12))
    return f, db(inner), db(position), db(single_position)


@dataclass
class StepStats:
    rise: float | None
    overshoot: float
    settle: float | None
    ss_err: float
    rms_err: float

    def text(self, unit: str = "") -> str:
        rt = f"{self.rise * 1000:.1f} ms" if self.rise is not None else "—"
        st = f"{self.settle * 1000:.1f} ms" if self.settle is not None else "—"
        suffix = f" {unit}" if unit else ""
        rows = (
            ("上升(10-90%)", rt),
            ("超调", f"{self.overshoot:.1f}%"),
            ("调节(2%)", st),
            ("稳态误差", f"{self.ss_err:+.4g}{suffix}"),
            ("跟踪 RMS", f"{self.rms_err:.4g}{suffix}"),
        )
        return "\n".join(f"{label:<12}：{value}" for label, value in rows)


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
