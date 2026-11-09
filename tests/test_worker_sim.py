"""用演示台（DemoBus）对 Worker 做无界面集成测试，覆盖达妙/HM 与大疆。"""
import time

import pytest
from PySide6.QtCore import QCoreApplication

from canmotorkit.analysis import sysid
from canmotorkit.analysis.runs import ExciteConfig, ExciteRun, LoopConfig, LoopRun
from canmotorkit.control.safety import SafetyLimits
from canmotorkit.core.worker import Worker
from canmotorkit.drivers import dm_protocol as P
from canmotorkit.drivers.catalog import make_driver
from canmotorkit.sim.demo import DemoBus, DjiSimMotor, DmSimMotor


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


class Rig:
    def __init__(self, app, driver=None):
        self.bus = DemoBus()
        self.bus.open()
        self.w = Worker(self.bus, driver)
        self.rec = {"samples": [], "regs": [], "scan": None, "msgs": [], "runs": [], "tx": {}}
        w, r = self.w, self.rec
        w.samples.connect(lambda s: r["samples"].extend(s))
        w.reg_done.connect(lambda k, rid, v, e: r["regs"].append((k, rid, v, e)))
        w.scan_done.connect(lambda f: r.__setitem__("scan", f))
        w.message.connect(r["msgs"].append)
        w.run_finished.connect(r["runs"].append)
        w.frames.connect(self._frames)
        w.start()

    def _frames(self, fs):
        for f in fs:
            if f.tx:
                self.rec["tx"][f.can_id] = self.rec["tx"].get(f.can_id, 0) + 1

    def close(self):
        self.w.stop()
        self.w.wait(2000)
        self.bus.close()


@pytest.fixture
def hm(app):
    r = Rig(app, make_driver("dm", "HM9816G9", 1, 0))
    r.w.set_sending(True)
    assert pump(app, 1.5, lambda: r.w.limits_ok and r.w.last_fb is not None)
    yield r
    r.close()


def test_dm_limits_and_registers(app, hm):
    w, rec = hm.w, hm.rec
    assert abs(w.lim.p_max - 3.14159265) < 1e-6
    w.read_reg(0x3C)
    w.read_reg(0x0A)            # HM 没有 CTRL_MODE → 超时
    w.write_reg(0x09, 0)
    w.write_reg(0x14, 5.0)      # 本软件只读
    w.save()
    assert pump(app, 2.0, lambda: len(rec["regs"]) >= 5 + 3)
    d = {(k, r): (v, e) for k, r, v, e in rec["regs"]}
    assert abs(d[("read", 0x3C)][0] - 24.0) < 0.2
    assert d[("read", 0x0A)][1]
    assert d[("write", 0x14)][1]
    assert d[("save", 0)] == (True, "")


def test_dm_current_mode_estop_and_disable(app, hm):
    w, bus = hm.w, hm.bus
    m = bus.motor(DmSimMotor, 1)
    w.set_target_out(0.5)                       # 低于静摩擦 0.8，不会转
    w.enable()
    assert pump(app, 0.5, lambda: w.active)
    pump(app, 0.2)
    assert m.cmd.kp == 0 and m.cmd.kd == 0 and abs(m.cmd.t - 0.5) < 0.03
    assert abs(m.plant.vel) < 1e-6
    w.estop()
    pump(app, 0.2)
    assert m.enabled and w.safety.tripped and abs(m.cmd.t) < 0.03    # 急停不失能
    w.reset_safety()
    pump(app, 0.2)
    assert not w.safety.tripped and abs(m.cmd.t - 0.5) < 0.03
    w.disable()
    assert pump(app, 0.5, lambda: not m.enabled and w.motor_code == 0)
    n = hm.rec["tx"].get(P.CONFIG_ID, 0)
    pump(app, 0.3)
    assert hm.rec["tx"].get(P.CONFIG_ID, 0) > n + 20      # 失能后恢复 0xCC 查询


def test_dm_mit_mode(app, hm):
    w, m = hm.w, hm.bus.motor(DmSimMotor, 1)
    w.set_mode("mit")
    w.set_target_mit(P.MitCommand(p=0.5, kp=20, kd=1))
    w.enable()
    assert pump(app, 2.0, lambda: w.active and abs(w.last_fb.pos - 0.5) < 0.05)
    w.disable()


def test_overspeed_trip(app, hm):
    w, m = hm.w, hm.bus.motor(DmSimMotor, 1)
    w.set_safety_limits(SafetyLimits(vel_max=2.0, out_max=2.0, temp_max=0))
    w.set_target_out(1.5)
    w.enable()
    assert pump(app, 3.0, lambda: w.safety.tripped)
    assert "速度" in w.safety.reason
    pump(app, 0.1)
    assert m.enabled and abs(m.cmd.t) < 0.03
    w.disable()


def test_dji_current(app):
    r = Rig(app, make_driver("dji", "M3508 (C620)", 1))
    try:
        w, m = r.w, r.bus.motor(DjiSimMotor, 1)
        assert pump(app, 0.5, lambda: w.last_fb is not None)     # 广播反馈，无需发送
        assert r.rec["tx"] == {}
        w.set_target_out(2.0)
        pump(app, 0.05)
        w.enable()
        assert pump(app, 0.3, lambda: abs(m.cur - 1.0) < 0.01)    # 默认安全限幅 ±1
        w.set_safety_limits(SafetyLimits(vel_max=100, out_max=5, temp_max=0))
        w.enable()                                               # 软件使能 + 自动开发送
        assert pump(app, 0.5, lambda: abs(m.cur - 2.0) < 0.01)
        assert pump(app, 1.0, lambda: w.last_fb.vel > 1.0)
        assert abs(w.last_fb.out - 2.0) < 0.2
        w.disable()
        assert pump(app, 0.3, lambda: m.cur == 0.0)
    finally:
        r.close()


def test_scan_all_protocols(app):
    r = Rig(app)
    try:
        r.w.start_scan(range(1, 8), window=0.01, listen=0.2)
        assert pump(app, 4.0, lambda: r.rec["scan"] is not None)
        got = {(x["protocol"], x["model"], x["motor_id"]) for x in r.rec["scan"]}
        assert ("dm", "HM9816G9", 1) in got
        assert ("dm", "DM-J8009P", 3) in got
        dji = {x["fb_id"]: x for x in r.rec["scan"] if x["protocol"] == "dji"}
        assert set(dji) == {0x201, 0x202, 0x206}
        assert dji[0x202]["model"] == "M2006 (C610)"           # 无温度 → C610
        assert dji[0x206]["model"] in ("M3508 (C620)", "GM6020 (电流模式)")
        assert len(dji[0x206]["candidates"]) == 3                # C620/C610/GM6020 重叠
        dm8 = next(x for x in r.rec["scan"] if x["motor_id"] == 3 and x["protocol"] == "dm")
        assert dm8["master_id"] == 0x13
    finally:
        r.close()


def test_response_and_loop_on_demo(app):
    r = Rig(app, make_driver("dji", "M3508 (C620)", 1))
    try:
        w = r.w
        w.rate_hz = 500
        w.set_safety_limits(SafetyLimits(vel_max=40, out_max=5, temp_max=0, fb_timeout=0.1))
        w.enable()
        assert pump(app, 0.5, lambda: w.active)
        run = ExciteRun(ExciteConfig(kind="stepsine", amp=1.5, f0=2, f1=20, n_freq=5, cycles=4,
                                     min_seg=0.2, bias_vel=8.0, bias_kp=0.3))
        w.start_run(run)
        assert pump(app, 8.0, lambda: run.done), r.rec["msgs"]
        assert not run.reason, run.reason
        bode = sysid.bode_from_run(run)
        assert bode is not None
        model = sysid.fit_model(bode, "A")
        J_true = 0.002 / 0.3                 # 等效 J/k（A 单位）
        assert 0.4 < model.J / J_true < 2.0, model
        s = sysid.suggest(model, latency=0.002, rate_hz=500)
        loop = LoopRun(LoopConfig(loop="vel", kp=s.vel_kp * 0.5, ki=s.vel_ki * 0.5, amp=10.0,
                                  duration=1.5, out_max=5))
        w.start_run(loop)
        assert pump(app, 4.0, lambda: loop.done)
        st = sysid.loop_stats(loop.rec, "vel")
        assert st is not None and abs(st.ss_err) < 1.0, st
        w.disable()
    finally:
        r.close()
