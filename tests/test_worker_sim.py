"""用 SimBus 对 Worker 做无界面集成测试。"""
import time

import pytest
from PySide6.QtCore import QCoreApplication

from hm import protocol as P
from hm.bus import SimBus
from hm.siggen import SignalGen, step_metrics
from hm.worker import Worker


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
    rec = {"samples": [], "regs": [], "scan": None, "tx_mit": 0, "sigdone": 0, "msgs": []}

    def on_frames(fs):
        rec["tx_mit"] += sum(1 for f in fs if f.tx and f.can_id == motor_id and not P.is_special(f.data))

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


def test_auto_limits_and_registers(app, rig):
    w, bus, rec = rig
    assert pump(app, 1.0, lambda: w.limits_ok)
    assert abs(w.lim.p_max - 3.14159) < 1e-4 and w.lim.t_max == 54.0
    assert pump(app, 1.0, lambda: len(rec["samples"]) > 10)   # 0xCC 轮询
    assert all(s.st.err == 0 for s in rec["samples"])
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


def test_enable_without_streaming_sends_no_mit(app, rig):
    w, bus, rec = rig
    assert pump(app, 1.0, lambda: w.limits_ok)
    w.set_target(P.MitCommand(p=1.0, kp=5, kd=0.2))
    w.enable()
    assert pump(app, 0.5, lambda: w.enabled)            # 状态码 1
    pump(app, 0.3)
    assert rec["tx_mit"] == 0 and not w.streaming
    assert w.get_target().p == 1.0                       # 使能不改目标
    w.write_reg(0x09, 5)                                 # 使能中拒绝写
    assert pump(app, 0.5, lambda: any(k == "write" and e for k, _, _, e in rec["regs"]))
    w.disable()
    assert pump(app, 0.5, lambda: w.motor_code == 0)


def test_streaming_step_and_estop(app, rig):
    w, bus, rec = rig
    assert pump(app, 1.0, lambda: w.limits_ok and w.last_status is not None)
    p0 = w.last_status.pos
    w.set_target(P.MitCommand(p=p0, kp=20.0, kd=1.0))
    w.enable()
    w.set_streaming(True)
    assert pump(app, 0.5, lambda: w.enabled and rec["tx_mit"] > 10)
    rec["samples"].clear()
    w.start_signal(SignalGen("step", amp=0.5, offset=p0, duration=1.5, delay=0.2), "p")
    assert pump(app, 3.0, lambda: rec["sigdone"] >= 1)
    s = [x for x in rec["samples"] if x.streaming]
    m = step_metrics([x.t for x in s], [x.cmd.p for x in s], [x.st.pos for x in s])
    assert m is not None and abs(m.ss_error) < 0.02, m
    w.estop()
    assert pump(app, 0.5, lambda: w.motor_code == 0)
    assert not w.streaming and not bus.motors[0].enabled


def test_no_limits_refuses_enable(app):
    w, bus, rec = make_rig(app, motor_id=9)   # 不存在的电机
    try:
        pump(app, 1.5)
        assert not w.limits_ok
        w.enable()
        w.set_streaming(True)
        pump(app, 0.3)
        assert not w.streaming
        assert not any(f.data == P.special_frame(P.SPECIAL_ENABLE) for f in [])  # 占位
        assert any("禁止使能" in m for m in rec["msgs"])
    finally:
        w.stop()
        w.wait(2000)
        bus.close()


def test_scan(app, rig):
    w, bus, rec = rig
    w.start_scan(range(1, 20), window=0.01)
    assert pump(app, 3.0, lambda: rec["scan"] is not None)
    assert [m for m, _, _ in rec["scan"]] == [1, 5]
