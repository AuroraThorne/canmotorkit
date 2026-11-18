from canmotorkit.core.socketcan import SocketCanBus


def test_socketcan_classic_frame_roundtrip():
    raw = SocketCanBus.pack_frame(0x123, b"\x01\x02\x03")
    assert len(raw) == 16
    assert SocketCanBus.unpack_frame(raw) == (0x123, b"\x01\x02\x03")


def test_socketcan_rejects_error_frames():
    raw = bytearray(SocketCanBus.pack_frame(0x123, b""))
    raw[3] |= 0x20  # CAN_ERR_FLAG 的最高字节（本机小端布局）
    assert SocketCanBus.unpack_frame(bytes(raw)) is None


def test_socketcan_extended_and_remote_frames():
    raw = SocketCanBus.pack_frame(0x1ABCDE, b"", extended=True, rtr=True, dlc=8)
    assert SocketCanBus.unpack_frame(raw, with_flags=True) == (0x1ABCDE, b"", True, True, 8)
