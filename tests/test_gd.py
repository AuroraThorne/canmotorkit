"""GD 手册示例、位域、边界与标准/扩展帧隔离。"""
import math

import pytest

from canmotorkit.core.bus import Frame
from canmotorkit.core.bus import CanBus
from canmotorkit.core.worker import Worker
from canmotorkit.drivers import gd
from canmotorkit.drivers.catalog import make_driver, PROTOCOLS
from canmotorkit.drivers.dm_protocol import MitCommand
from canmotorkit.sim.demo import DemoBus


def test_documented_gd_feedback_and_torque():
    d = make_driver("gd", gd.MODEL, 1)
    assert gd.MODEL in PROTOCOLS["gd"][1]
    specific = make_driver("gd", gd.MODEL_5036, 1)
    assert specific.model == "GD 5036-02-CNE"
    assert specific.output_frames(1) == d.output_frames(1)
    assert d.extended and d.tx_id == 0x18001 and d.fb_id == 0x10100
    # 手册示例：0.02 圈、0.1 圈/s、0.03 Nm。
    data = bytes.fromhex("0E 00 C8 00 90 01 03 00")
    fb = d.parse(0x10100, data)
    assert fb.pos == pytest.approx(0.02 * math.tau)
    assert fb.vel == pytest.approx(0.1 * math.tau)
    assert fb.out == 0.03 and fb.code == 0 and fb.enabled is None
    assert math.isnan(fb.temp)
    assert d.output_frames(-1.23) == [(0x18001, bytes.fromhex("1B 0E 85 FF"))]
    assert d.stop_frames() == [(0x18001, bytes.fromhex("1B 0E 00 00"))]
    assert d.disable_frames() == [(0x18001, bytes.fromhex("01 0E"))]
    assert d.idle_frames() == [(0x18001, bytes.fromhex("00 0E"))]
    assert d.zero_frames() == [(0x18001, bytes.fromhex("03 03 03"))]
    assert not d.is_fault(0) and d.is_fault(1)
    assert "0.030 Nm" in d.describe(0x10100, data, False)
    assert not d.accepts(0x10200, data)
    assert d.parse(0x10100, b"\x03\x00") is None
    w = Worker(DemoBus(), d)
    assert w._on_rx(Frame(0, 0x10100, data)) is None
    assert w._on_rx(Frame(0, 0x10100, data, extended=True, rtr=True)) is None
    assert w._on_rx(Frame(0, 0x10100, data, extended=True)) is not None


def test_gd_mit_uses_little_endian_field_order_and_exact_zero():
    lo = MitCommand(gd.LIMITS.p_min, gd.LIMITS.v_min, -800, -200, -100)
    hi = MitCommand(gd.LIMITS.p_max, gd.LIMITS.v_max, 800, 200, 100)
    assert gd.pack_mit(lo) == bytes(8)
    assert gd.pack_mit(hi) == bytes([255]) * 8
    assert gd.pack_mit(MitCommand()) == bytes.fromhex("00 80 00 08 80 00 08 80")
    cmd = MitCommand(p=0, v=10 * math.tau, kp=800, kd=-200, t=-100)
    assert gd.pack_mit(cmd) == bytes.fromhex("00 80 FF 0F 00 FF 0F 00")
    assert gd.unpack_mit(gd.pack_mit(cmd)).v == pytest.approx(cmd.v)
    d = gd.GdDriver(motor_id=126)
    assert d.mit_frames(MitCommand()) == d.stop_frames()  # 不发非零的量化增益
    assert d.mit_frames(cmd)[0][0] == 0x5807E
    assert d.mit_frames(cmd)[1] == (0x1807E, bytes.fromhex("00 0E"))
    assert d.out_max == 100
    assert d.output_frames(1e6)[0][1] == bytes.fromhex("1B 0E 10 27")
    for invalid in (float("nan"), float("inf"), -float("inf")):
        with pytest.raises(ValueError):
            d.output_frames(invalid)
        with pytest.raises(ValueError):
            gd.pack_mit(MitCommand(kp=invalid))


@pytest.mark.parametrize("mid", [0, 127, -1])
def test_gd_rejects_invalid_ids(mid):
    with pytest.raises(ValueError):
        gd.GdDriver(motor_id=mid)


def test_gd_system_commands_match_document_and_decode_identity():
    d = gd.GdDriver()
    assert d.system_frame("save") == bytes.fromhex("03 03 02")
    assert d.system_frame("zero") == bytes.fromhex("03 03 03")
    assert d.system_frame("reboot") == bytes.fromhex("03 03 01")
    assert d.system_frame("change_id", 9) == bytes.fromhex("03 03 04 09")
    assert d.system_frame("periodic", 1000) == bytes.fromhex("03 00 05 0E E8 03 00 00")
    assert d.query_frame("model") == bytes.fromhex("00 07")
    assert d.parse_identity("firmware", bytes.fromhex("04 08 01 00")) == "0.1.8"
    partial = d.parse_identity("model", b'\x07\x075036_0')
    assert partial == dict(text="5036_0", complete=False, length=7)
    d.apply_identity("model", partial)
    assert d.model == gd.MODEL_5036_FAMILY
    d.apply_identity("model", d.parse_identity("model", b'\x07\x046036'))
    assert d.model == gd.MODEL_6036
    for data in (b'\x07', b'\x07\x00', b'\x07\x02\xff\xff'):
        with pytest.raises(ValueError):
            d.parse_identity("model", data)
    for value in (99, -1, 2**32, True):
        with pytest.raises(ValueError):
            d.system_frame("periodic", value)


def test_gd_internal_control_examples():
    d = gd.GdDriver()
    assert d.control_frames("velocity", dict(velocity=0.1 * math.tau)) == [(0x18001, bytes.fromhex("1C 0E 90 01"))]
    assert d.control_frames("position", dict(position=-0.5 * math.tau)) == [(0x18001, bytes.fromhex("1D 0E 78 EC"))]
    values = dict(position=0.5 * math.tau, velocity=0.5 * math.tau, torque=1)
    assert d.control_frames("position_velocity_torque", values) == [(0x18001, bytes.fromhex("1F 0E 88 13 D0 07 64 00"))]
    assert d.control_frames("brake", {}) == [(0x18001, bytes.fromhex("18 0E"))]
    assert d.control_frames("coast", {}) == [(0x18001, bytes.fromhex("01 0E"))]
    with pytest.raises(ValueError):
        d.control_frames("velocity", dict(velocity=100000))


class GdCaptureBus(CanBus):
    def __init__(self):
        self.tx = []

    def send(self, can_id, data, **kwargs):
        self.tx.append((can_id, data, kwargs))


def gd_rig():
    bus = GdCaptureBus()
    w = Worker(bus, gd.GdDriver(motor_id=6))
    w._gd_queue.clear()
    results = []
    w.gd_done.connect(lambda k, v, e: results.append((k, v, e)))
    return w, bus, results


def test_gd_id_change_checks_collision_and_verifies_new_id():
    w, bus, result = gd_rig()
    w._do_gd_request("change_id", 9)
    w._service_gd(0)
    assert bus.tx[-1][:2] == (0x18009, b'\x00\x0e')
    data = bytes.fromhex("0E 00 00 00 00 00 00 00")
    w._gd_rx(Frame(0, 0x10900, data, extended=True))
    assert "已有电机" in result[-1][2]
    assert not any(t[1][:3] == bytes.fromhex("03 03 04") for t in bus.tx)
    w._gd_cooldown = 0
    w._do_gd_request("change_id", 9)
    w._service_gd(0)
    w._service_gd(w._gd_pending['deadline'] + 0.001)
    assert bus.tx[-1][:2] == (0x18006, bytes.fromhex("03 03 04 09"))
    w._gd_rx(Frame(0, 0x10900, bytes.fromhex("03 00"), extended=True))
    assert w.driver.motor_id == 6  # 系统确认后仍需新 ID 状态
    w._gd_rx(Frame(0, 0x10900, data, extended=True))
    assert w.driver.motor_id == 9 and result[-1] == ("change_id", 9, "")
    assert bus.tx[-1][2]["extended"] is True


def test_gd_timeout_never_repeats_system_commands_and_recovers_queue():
    w, bus, result = gd_rig()
    w._do_gd_request("save")
    w._service_gd(0)
    w._service_gd(w._gd_pending['deadline'] + 0.001)
    assert len(bus.tx) == 1 and "超时" in result[-1][2]
    w._gd_cooldown = 0
    w._do_gd_request("hardware")
    w._service_gd(0)
    w._gd_rx(Frame(0, 0x10600, bytes.fromhex("05 08 01 00"), extended=True))
    assert result[-1] == ("hardware", "0.1.8", "")
    w._sw_en = True
    w._do_gd_request("zero")
    assert "停止输出" in result[-1][2]


def test_gd_direct_motion_zero_trip_and_watchdog():
    import time
    w, bus, result = gd_rig()
    w._sw_en = True
    w._do_set_mode("gd")
    w.safety.limits.vel_max = 2
    w.safety.limits.out_max = 0.5
    w._do_gd_control("position_velocity_torque", dict(position=0.1, velocity=5, torque=2))
    w._tick(time.perf_counter())
    frame = bus.tx[-1][1]
    import struct
    pos, vel, torque = struct.unpack_from("<hhh", frame, 2)
    assert abs(vel - 2 * 4000 / math.tau) <= 1 and torque == 50
    w._do_zero_output()
    w._tick(time.perf_counter())
    assert bus.tx[-1][1] == bytes.fromhex("1B 0E 00 00")
    assert w._gd_control is None
    w._do_gd_control("brake", {})
    w.safety.last_fb_t = time.perf_counter() - 1
    w._tick(time.perf_counter())
    assert w.safety.tripped
    assert w._gd_control is None
    assert bus.tx[-1][1] == bytes.fromhex("1B 0E 00 00")


def test_gd_watchdog_covers_missing_first_feedback_and_mit_zero_feedforward():
    import time
    w, bus, result = gd_rig()
    w._do_enable()
    w.mode = "mit"
    w.set_target_mit(MitCommand(p=1, kp=10))
    w._tick(time.perf_counter() + 1)
    assert w.safety.tripped and w.get_target() == MitCommand()
    assert bus.tx[-1][1] == bytes.fromhex("1B 0E 00 00")
