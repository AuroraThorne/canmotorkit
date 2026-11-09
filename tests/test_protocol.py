import math
import struct

import pytest

from canmotorkit.drivers import dm_protocol as P

G9 = P.DEFAULT_LIMITS


@pytest.mark.parametrize("x,lo,hi,bits", [
    (0.0, -12.5, 12.5, 16), (1.234, -12.5, 12.5, 16), (-45, -45, 45, 12),
    (45, -45, 45, 12), (250, 0, 500, 12), (0.5, 0, 5, 12), (-10.1, -54, 54, 12),
])
def test_float_uint_roundtrip(x, lo, hi, bits):
    u = P.float_to_uint(x, lo, hi, bits)
    assert 0 <= u <= (1 << bits) - 1
    assert abs(P.uint_to_float(u, lo, hi, bits) - x) <= P.lsb(lo, hi, bits)


def test_clamp_no_wrap():
    assert P.float_to_uint(100, -12.5, 12.5, 16) == 0xFFFF
    assert P.float_to_uint(-100, -12.5, 12.5, 16) == 0


def test_mit_roundtrip():
    cmd = P.MitCommand(p=1.5, v=-3.0, kp=20.0, kd=1.0, t=2.5)
    d = P.pack_mit(cmd, G9)
    assert len(d) == 8
    back = P.unpack_mit(d, G9)
    for f, lo, hi, bits in (("p", -12.5, 12.5, 16), ("v", -45, 45, 12), ("kp", 0, 500, 12),
                            ("kd", 0, 5, 12), ("t", -54, 54, 12)):
        assert abs(getattr(back, f) - getattr(cmd, f)) <= P.lsb(lo, hi, bits)


def test_mit_zero_command_bytes():
    # p=v=t=0 取量程中点，kp=kd=0 为 0
    d = P.pack_mit(P.MitCommand(), G9)
    assert d[0:2] == bytes((0x80, 0x00))          # 32767.5 -> 32768
    v = (d[2] << 4) | (d[3] >> 4)
    assert v == 2048
    assert (d[3] & 0x0F) == 0 and d[4] == 0 and d[5] == 0 and (d[6] >> 4) == 0


def test_status_roundtrip():
    d = P.pack_status(1, 3, 0.7, -2.0, 5.0, 40, 35, G9)
    s = P.unpack_status(d, G9)
    assert s.motor_id4 == 1 and s.err == 3
    assert math.isclose(s.pos, 0.7, abs_tol=P.lsb(-12.5, 12.5, 16))
    assert math.isclose(s.vel, -2.0, abs_tol=P.lsb(-45, 45, 12))
    assert math.isclose(s.torque, 5.0, abs_tol=P.lsb(-54, 54, 12))
    assert (s.t_mos, s.t_rotor) == (40, 35)


def test_special_frames():
    assert P.special_frame(P.SPECIAL_ENABLE) == bytes.fromhex("FFFFFFFFFFFFFFFC")
    assert P.special_frame(P.SPECIAL_DISABLE) == bytes.fromhex("FFFFFFFFFFFFFFFD")
    assert P.is_special(P.special_frame(P.SPECIAL_ZERO))
    assert not P.is_special(P.pack_mit(P.MitCommand(), G9))


def test_config_frames_match_manual():
    assert P.read_reg_frame(1, 0x3C) == bytes([0x01, 0x00, 0x33, 0x3C, 0, 0, 0, 0])
    assert P.write_reg_frame(0x101, 0x09, "u", 100) == bytes([0x01, 0x01, 0x55, 0x09, 100, 0, 0, 0])
    assert P.write_reg_frame(1, 0x01, "f", 0.45)[4:] == struct.pack("<f", 0.45)
    assert P.save_frame(1) == bytes([0x01, 0x00, 0xAA, 0x01, 0, 0, 0, 0])
    assert P.query_state_frame(5) == bytes([0x05, 0x00, 0xCC, 0, 0, 0, 0, 0])


def test_reply_classification():
    req = P.read_reg_frame(1, 0x3C)
    rep = bytes(req[:4]) + struct.pack("<f", 24.1)
    assert P.matches_request(rep, req)
    r = P.parse_config_reply(rep)
    assert r.cmd == P.CMD_READ and r.sub == 0x3C
    assert abs(P.decode_value("f", r.raw4) - 24.1) < 1e-5
    status = P.pack_status(1, 0, 0, 0, 0, 30, 30, G9)
    assert not P.matches_request(status, req)


def test_value_types():
    for typ, val in (("u", 4000000000), ("i", -5), ("f", 12.5)):
        assert P.decode_value(typ, P.encode_value(typ, val)) == pytest.approx(val)


def test_limits_override():
    lim = G9.with_maps(p_max=25.0, v_max=None, t_max=40.0)
    assert (lim.p_min, lim.p_max, lim.v_max, lim.t_min) == (-25.0, 25.0, 45.0, -40.0)


def test_status_code_text():
    assert P.code_text(0) == "失能" and P.code_text(1) == "已使能"
    assert not P.is_fault(1) and P.is_fault(8)
    assert "过压" in P.code_text(8)


def test_register_table_unique():
    assert len(P.REG_BY_RID) == len(P.REGISTERS)
    assert P.REG_BY_NAME["output_angle_multiturn"].rid == 0x51
