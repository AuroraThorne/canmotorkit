import math

from canmotorkit.control.safety import MultiTurn, SafetyLimits, Supervisor


def test_multiturn_unwrap():
    m = MultiTurn()
    xs = [3.0, 3.1, -3.1, -3.0, 3.1, 3.0]   # 正向跨过 π 再反向回来
    ys = [m.update(x) for x in xs]
    assert abs(ys[2] - (2 * math.pi - 3.1)) < 1e-9
    assert abs(ys[4] - 3.1) < 1e-9


def test_output_clamp_and_trip_latch():
    s = Supervisor(SafetyLimits(vel_max=5, out_max=1.0, temp_max=0, fb_timeout=0))
    s.on_feedback(0.0, 0.0, 1.0, 30)
    assert s.filter_output(0.0, 3.0) == 1.0
    assert s.filter_output(0.0, -0.4) == -0.4
    s.on_feedback(0.01, 0.0, 6.0, 30)
    assert s.tripped and "速度" in s.reason
    assert s.filter_output(0.01, 0.5) == 0.0
    assert not s.reset()                       # 仍超速，拒绝解除
    s.on_feedback(0.02, 0.0, 0.5, 30)
    assert s.tripped                           # 锁存，不自动恢复
    assert s.reset() and not s.tripped


def test_position_window_multiturn():
    s = Supervisor(SafetyLimits(vel_max=0, pos_enabled=True, pos_min=-4, pos_max=4, temp_max=0,
                                fb_timeout=0))
    for x in (2.0, 3.0, -3.0, -2.5):        # 累计 ≈ 3.78
        s.on_feedback(0, x, 0, 0)
    assert not s.tripped and abs(s.pos - (2 * math.pi - 2.5)) < 1e-9
    s.on_feedback(0, -2.0, 0, 0)              # 累计 ≈ 4.28
    assert s.tripped and "位置" in s.reason


def test_feedback_timeout_only_when_output():
    s = Supervisor(SafetyLimits(vel_max=0, temp_max=0, fb_timeout=0.1))
    s.on_feedback(0.0, 0, 0, 0)
    assert s.filter_output(1.0, 0.0) == 0.0 and not s.tripped
    assert s.filter_output(1.0, 0.5) == 0.0 and s.tripped and "超时" in s.reason
