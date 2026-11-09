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
    assert s.vel_kp > 0 and s.pos_kp > 0 and s.pos_kd > 0
    for loop, kp, ki, kd, amp in (("vel", s.vel_kp, s.vel_ki, 0.0, 2.0),
                                  ("pos", s.pos_kp, s.pos_ki, s.pos_kd, 0.5)):
        run = LoopRun(LoopConfig(loop=loop, kp=kp, ki=ki, kd=kd, amp=amp, duration=2.0, out_max=10))
        simulate(run, delay_steps=2)
        st = sysid.loop_stats(run.rec, loop)
        assert st is not None and st.overshoot < 40 and abs(st.ss_err) < 0.05 * amp, (loop, st)
