"""用 SimBus 对 Worker 做无界面集成测试。"""
import time

import pytest
from PySide6.QtCore import QCoreApplication

from canmotorkit.drivers import dm_protocol as P
from canmotorkit.core.bus import SimBus
from canmotorkit.analysis.siggen import SignalGen, step_metrics
from canmotorkit.core.worker import Worker


@pytest.fixture(scope="module")
def app():
    return QCoreApplication.instance() or QCoreApplication([])


def pump(app, sec, until=None):
    end = time.perf_counter() + sec
    while time.perf_counter() < end:
        app.processEvents()
        if until and until():
            return True
        time.sleep(0.005)
    return bool(until and until())


def make_rig(app, motor_id=1):
    bus = SimBus()
    bus.open()
    w = Worker(bus, motor_id, 0)
    rec = {"samples": [], "regs": [], "scan": None, "tx_mit": 0, "tx_cc": 0, "sigdone": 0,
           "msgs": []}

    def on_frames(fs):
        for f in fs:
            if not f.tx:
                continue
            if f.can_id == motor_id and not P.is_special(f.data):
                rec["tx_mit"] += 1
            elif f.can_id == P.CONFIG_ID and f.data[2] == P.CMD_STATE:
                rec["tx_cc"] += 1

    w.samples.connect(lambda s: rec["samples"].extend(s))
    w.reg_done.connect(lambda k, r, v, e: rec["regs"].append((k, r, v, e)))
    w.scan_done.connect(lambda f: rec.__setitem__("scan", f))
    w.frames.connect(on_frames)
    w.message.connect(rec["msgs"].append)
    w.signal_finished.connect(lambda: rec.__setitem__("sigdone", rec["sigdone"] + 1))
    w.start()
    return w, bus, rec


@pytest.fixture
def rig(app):
    w, bus, rec = make_rig(app)
    yield w, bus, rec
    w.stop()
    w.wait(2000)
    bus.close()


def reset_counts(rec):
    rec["tx_mit"] = rec["tx_cc"] = 0


def test_auto_limits_and_registers(app, rig):
    w, bus, rec = rig
    assert pump(app, 1.0, lambda: w.limits_ok)
    assert abs(w.lim.p_max - 3.14159) < 1e-4 and w.lim.t_max == 54.0
    assert pump(app, 1.0, lambda: len(rec["samples"]) > 10)   # 0xCC 查询
    assert all(s.st.err == 0 and not s.mit for s in rec["samples"])
    rec["regs"].clear()
    w.read_reg(0x3C)
    w.read_reg(0x10)
    w.write_reg(0x09, 100)
    w.write_reg(0x3C, 1.0)        # 只读
    w.save()
    assert pump(app, 2.0, lambda: len(rec["regs"]) >= 5)
    d = {(k, r): (v, e) for k, r, v, e in rec["regs"]}
    assert abs(d[("read", 0x3C)][0] - 24.0) < 0.2
    assert d[("read", 0x10)][0] == 21
    assert d[("write", 0x09)][0] == 100
    assert d[("write", 0x3C)][1]
    assert d[("save", 0)] == (True, "")


def test_enable_switches_query_to_mit(app, rig):
    w, bus, rec = rig
    assert pump(app, 1.0, lambda: w.limits_ok)
    w.set_target(P.MitCommand(p=1.0, kp=5, kd=0.2))
    pump(app, 0.2)
    reset_counts(rec)
    pump(app, 0.5)
    assert rec["tx_mit"] == 0 and rec["tx_cc"] > 50          # 未使能：只发查询
    w.enable()
    assert pump(app, 0.5, lambda: w.enabled)
    pump(app, 0.1)
    reset_counts(rec)
    pump(app, 0.5)
    assert rec["tx_mit"] > 50 and rec["tx_cc"] == 0          # 已使能：只发 MIT
    assert w.get_target().p == 1.0                           # 使能不改目标
    assert abs(bus.motors[0].cmd.p - 1.0) < 0.01
    w.write_reg(0x09, 5)                                     # 使能中拒绝写
    assert pump(app, 0.5, lambda: any(k == "write" and e for k, _, _, e in rec["regs"]))
    w.disable()
    assert pump(app, 0.5, lambda: w.motor_code == 0)
    pump(app, 0.1)
    reset_counts(rec)
    pump(app, 0.4)
    assert rec["tx_mit"] == 0 and rec["tx_cc"] > 30


def test_rate_shared(app, rig):
    w, bus, rec = rig
    assert pump(app, 1.0, lambda: w.limits_ok)
    w.rate_hz = 500
    pump(app, 0.2)
    reset_counts(rec)
    pump(app, 1.0)
    assert 400 < rec["tx_cc"] < 560
    w.enable()
    assert pump(app, 0.5, lambda: w.enabled)
    pump(app, 0.1)
    reset_counts(rec)
    pump(app, 1.0)
    assert 400 < rec["tx_mit"] < 560
    w.disable()


def test_step_and_estop(app, rig):
    w, bus, rec = rig
    assert pump(app, 1.0, lambda: w.limits_ok and w.last_status is not None)
    p0 = w.last_status.pos
    w.set_target(P.MitCommand(p=p0, kp=20.0, kd=1.0))
    w.enable()
    assert pump(app, 0.5, lambda: w.enabled and rec["tx_mit"] > 10)
    rec["samples"].clear()
    w.start_signal(SignalGen("step", amp=0.5, offset=p0, duration=1.5, delay=0.2), "p")
    assert pump(app, 3.0, lambda: rec["sigdone"] >= 1)
    s = [x for x in rec["samples"] if x.mit]
    m = step_metrics([x.t for x in s], [x.cmd.p for x in s], [x.st.pos for x in s])
    assert m is not None and abs(m.ss_error) < 0.02, m
    w.estop()
    pump(app, 0.3)
    m0 = bus.motors[0]
    assert m0.enabled and w.safety.tripped                     # 急停不失能
    assert m0.cmd.kp == 0 and m0.cmd.kd == 0 and abs(m0.cmd.t) < 0.05
    w.set_target(P.MitCommand(t=0.5))
    pump(app, 0.2)
    assert abs(m0.cmd.t) < 0.05                                # 锁存中仍为 0
    w.disable()
    assert pump(app, 0.5, lambda: w.motor_code == 0)


def test_overspeed_trips_zero_torque(app, rig):
    from canmotorkit.control.safety import SafetyLimits
    w, bus, rec = rig
    assert pump(app, 1.0, lambda: w.limits_ok)
    w.set_safety_limits(SafetyLimits(vel_max=3.0, out_max=5.0, temp_max=0))
    w.set_target(P.MitCommand(t=2.0))
    w.enable()
    assert pump(app, 3.0, lambda: w.safety.tripped)
    assert "速度" in w.safety.reason
    pump(app, 0.1)
    assert abs(bus.motors[0].cmd.t) < 0.05 and bus.motors[0].enabled
    w.disable()


def test_no_limits_refuses_enable(app):
    w, bus, rec = make_rig(app, motor_id=9)   # 不存在的电机
    try:
        pump(app, 1.5)
        assert not w.limits_ok
        w.enable()
        pump(app, 0.3)
        assert any("禁止使能" in m for m in rec["msgs"])
        assert rec["tx_mit"] == 0
    finally:
        w.stop()
        w.wait(2000)
        bus.close()


def test_already_enabled_without_limits_sends_zero_torque(app):
    bus = SimBus()
    bus.open()
    m = bus.motors[0]
    m.enabled = True                          # 模拟上次未失能就关闭了软件
    m.cmd = P.MitCommand(t=3.0)
    w = Worker(bus, 1, 0)
    w._reqs.clear()                           # 模拟量程未读到
    w.set_target(P.MitCommand(t=5.0))
    w.start()
    try:
        assert pump(app, 1.0, lambda: w.enabled)
        pump(app, 0.2)
        assert not w.limits_ok
        c = m.cmd
        assert abs(c.t) < 0.05 and c.kp == 0 and c.kd == 0
    finally:
        w.stop()
        w.wait(2000)
        bus.close()


def test_scan(app, rig):
    w, bus, rec = rig
    w.start_scan(range(1, 20), window=0.01)
    assert pump(app, 3.0, lambda: rec["scan"] is not None)
    assert [m for m, _, _ in rec["scan"]] == [1, 5]
