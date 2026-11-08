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


@pytest.fixture
def rig(app):
    bus = SimBus()
    bus.open()
    w = Worker(bus, 1, 0)
    rec = {"samples": [], "regs": [], "scan": None, "frames": 0, "sigdone": 0}
    w.samples.connect(lambda s: rec["samples"].extend(s))
    w.reg_done.connect(lambda k, r, v, e: rec["regs"].append((k, r, v, e)))
    w.scan_done.connect(lambda f: rec.__setitem__("scan", f))
    w.frames.connect(lambda f: rec.__setitem__("frames", rec["frames"] + len(f)))
    w.signal_finished.connect(lambda: rec.__setitem__("sigdone", rec["sigdone"] + 1))
    w.start()
    yield w, bus, rec
    w.stop()
    w.wait(2000)
    bus.close()


def test_poll_and_registers(app, rig):
    w, bus, rec = rig
    assert pump(app, 1.0, lambda: len(rec["samples"]) > 10)   # 失能状态 0xCC 轮询
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
    assert rec["frames"] > 0


def test_enable_step_and_save_refused(app, rig):
    w, bus, rec = rig
    pump(app, 0.3)
    w.enable()
    pump(app, 0.2)
    tgt = w.get_target()
    tgt.kp, tgt.kd = 20.0, 1.0
    w.set_target(tgt)
    rec["samples"].clear()
    w.start_signal(SignalGen("step", amp=0.5, offset=tgt.p, duration=1.5, delay=0.2), "p")
    assert pump(app, 3.0, lambda: rec["sigdone"] >= 1)
    s = [x for x in rec["samples"] if x.enabled]
    m = step_metrics([x.t for x in s], [x.cmd.p for x in s], [x.st.pos for x in s])
    assert m is not None and abs(m.ss_error) < 0.02, m
    w.save()
    pump(app, 0.3, lambda: any(k == "save" for k, *_ in rec["regs"]))
    assert any(k == "save" and e for k, _, _, e in rec["regs"])   # 使能中拒绝保存
    w.estop()
    pump(app, 0.2)
    assert not w.enabled and not bus.motors[0].enabled


def test_scan(app, rig):
    w, bus, rec = rig
    w.start_scan(range(1, 20), window=0.01)
    assert pump(app, 3.0, lambda: rec["scan"] is not None)
    assert [m for m, _, _ in rec["scan"]] == [1, 5]
