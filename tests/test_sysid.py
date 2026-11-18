"""辨识算法离线测试：用已知模型生成数据，检查能否恢复 J、b。"""
import math
from types import SimpleNamespace

from canmotorkit.analysis import sysid
from canmotorkit.analysis.runs import ExciteConfig, ExciteRun, LoopConfig, LoopRun


def simulate(run, J=0.01, b=0.05, fc=0.0, dt=0.002, delay_steps=1):
    run.start(0.0, 0.0, 0.0)
    v = p = 0.0
    hist = [0.0] * (delay_steps + 1)
    t = 0.0
    while True:
        fb = SimpleNamespace(vel=v, out=0.0)
        u = run.output(t, fb, p)
        if u is None:
            break
        hist.append(u)
        ua = hist.pop(0)
        s = math.copysign(1, v) if abs(v) > 1e-4 else 0.0
        v += (ua - b * v - fc * s) / J * dt
        p += v * dt
        run.record(t, u, SimpleNamespace(vel=v, out=ua), p)
        t += dt
    return run


def test_stepsine_recovers_model():
    run = simulate(ExciteRun(ExciteConfig(kind="stepsine", amp=0.5, f0=1, f1=20, n_freq=8)))
    bode = sysid.bode_from_run(run)
    assert bode is not None and len(bode.f) == 8
    m = sysid.fit_model(bode)
    assert abs(m.J - 0.01) / 0.01 < 0.1, m
    assert abs(m.b - 0.05) / 0.05 < 0.3, m
    assert 0.001 < m.Td < 0.01
    assert len(bode.coh) == len(bode.coh_p) == len(bode.f)
    # 位置/电流 = 速度/电流 / ω
    w = 2 * math.pi * bode.f
    assert all(abs(bode.mag_p * w / bode.mag_v - 1) < 0.1)


def test_chirp_and_prbs_bode():
    for kind in ("chirp", "prbs"):
        run = simulate(ExciteRun(ExciteConfig(kind=kind, amp=0.5, f0=1, f1=20, duration=6)))
        bode = sysid.bode_from_run(run)
        assert bode is not None, kind
        m = sysid.fit_model(bode)
        assert abs(m.J - 0.01) / 0.01 < 0.3, (kind, m)


def test_step_model_with_friction():
    cfg = ExciteConfig(kind="step", amp=1.0, step_on=0.3, step_delay=0.1, step_tail=0.4)
    run = simulate(ExciteRun(cfg), fc=0.2)
    m = sysid.step_model(run.rec, 1.0, 0.1, 0.3)
    assert m is not None
    assert abs(m.J - 0.01) / 0.01 < 0.25 and abs(m.Fc - 0.2) < 0.08, m


def test_suggest_and_closed_loop_stable():
    model = sysid.Model(J=0.01, b=0.05, Fc=0.0, Td=0.002)
    s = sysid.suggest(model, latency=0.002, rate_hz=500)
    assert s.vel_kp > 0 and s.vel_ki > 0 and s.pos_kp > 0 and s.pos_ki > 0
    assert "单环位置" not in s.text()
    vel = LoopRun(LoopConfig(loop="vel", kp=s.vel_kp, ki=s.vel_ki,
                             amp=2.0, duration=2.0, out_max=10))
    simulate(vel, delay_steps=2)
    vel_stats = sysid.loop_stats(vel.rec, "vel")
    assert vel_stats is not None and vel_stats.overshoot < 40 and abs(vel_stats.ss_err) < 0.1

    pos = LoopRun(LoopConfig(loop="pos", kp=s.vel_kp, ki=s.vel_ki,
                             pos_kp=s.pos_kp, pos_ki=s.pos_ki, pos_kd=s.pos_kd,
                             amp=0.5, duration=2.0, out_max=10, pos_out_max=10))
    simulate(pos, delay_steps=2)
    pos_stats = sysid.loop_stats(pos.rec, "pos")
    assert pos_stats is not None and pos_stats.overshoot < 40 and abs(pos_stats.ss_err) < 0.05, pos_stats


def test_suggestion_uses_custom_start_ratio():
    model = sysid.Model(J=0.01, b=0.05, Fc=0.0, Td=0.002)
    s = sysid.suggest(model, latency=0.002, rate_hz=500, start_ratio=0.75)
    assert s.start_ratio == 0.75
    assert "75%" in s.text()


def test_suggestion_target_is_closed_loop_bandwidth():
    model = sysid.Model(J=0.01, b=0.05, Fc=0.0, Td=0.002)
    s = sysid.suggest(model, latency=0.002, rate_hz=500, vel_bw_hz=5.0)
    assert 4.5 < s.vel_bw < 5.5, s.vel_bw


def test_single_position_suggestion_and_prediction():
    model = sysid.Model(J=0.01, b=0.05, Fc=0.0, Td=0.002)
    s = sysid.suggest(model, latency=0.002, rate_hz=500,
                      vel_bw_hz=5.0, single_pos_bw_hz=2.0)
    assert s.single_pos_kp > 0
    assert s.single_pos_ki == 0.0
    assert s.single_pos_kd >= 0
    assert 1.8 < s.single_pos_bw < 2.2
    assert 0 < s.single_pos_phase_margin < 180

    run = LoopRun(LoopConfig(
        loop="pos_single", kp=s.single_pos_kp, ki=s.single_pos_ki,
        kd=s.single_pos_kd, amp=0.2, duration=1.0, out_max=10.0,
    ))
    simulate(run, delay_steps=2)
    assert run.legacy_position
    assert len(run.rec.t) > 10

    curves = sysid.predicted_closed_loop(model, s, latency=0.002, rate_hz=500)
    assert len(curves) == 4
    assert all(len(curve) == len(curves[0]) for curve in curves)
