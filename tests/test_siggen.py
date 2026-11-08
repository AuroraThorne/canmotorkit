from hm.siggen import SignalGen, step_metrics


def test_step_metrics_ignores_small_pre_adjust():
    # 开始时指令从 0.300 微调到 0.304（=当前位置），然后在 t=0.5 阶跃 +0.5
    t = [i * 0.005 for i in range(600)]
    cmd = [0.300] * 5 + [0.304 if ti < 0.5 else 0.804 for ti in t[5:]]
    fb, y = [], 0.304
    for c in cmd:
        y += (c - y) * 0.05
        fb.append(y)
    m = step_metrics(t, cmd, fb)
    assert m is not None
    assert abs(m.target - 0.804) < 1e-9
    assert m.rise_time is not None and m.overshoot_pct < 1.0
    assert abs(m.ss_error) < 1e-3


def test_siggen_shapes():
    g = SignalGen("step", amp=1.0, offset=2.0, duration=1.0, delay=0.5)
    assert g.value(0.1) == 2.0 and g.value(0.6) == 3.0 and g.value(1.5) is None
    s = SignalGen("sine", amp=1.0, freq=1.0, duration=2.0)
    assert abs(s.value(0.25) - 1.0) < 1e-9
    c = SignalGen("chirp", amp=1.0, freq=1.0, freq_end=5.0, duration=2.0)
    assert all(abs(c.value(i * 0.01)) <= 1.0 for i in range(200))
