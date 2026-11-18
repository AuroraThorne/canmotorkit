from canmotorkit.core.slcan import encode_slcan_frame, parse_slcan_frame


def test_slcan_frame_roundtrip():
    raw = encode_slcan_frame(0x123, b"\x01\x02\xA0")
    assert raw == b"t12330102A0\r"
    assert parse_slcan_frame(raw[:-1]) == (0x123, b"\x01\x02\xA0")
    assert parse_slcan_frame(bytearray(raw[:-1])) == (0x123, b"\x01\x02\xA0")


def test_slcan_ignores_non_data_and_invalid_frames():
    assert parse_slcan_frame(b"O") is None
    assert parse_slcan_frame(b"r1230") is None
    assert parse_slcan_frame(b"t1239") is None
    assert parse_slcan_frame(b"t1232AA") is None


def test_slcan_extended_and_remote_frames():
    raw = encode_slcan_frame(0x1ABCDE, b"", extended=True, rtr=True, dlc=8)
    assert raw == b"R001ABCDE8\r"
    assert parse_slcan_frame(raw[:-1], with_flags=True) == (0x1ABCDE, b"", True, True, 8)
