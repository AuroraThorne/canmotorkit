"""PCANBasic.dll 的最小 ctypes 封装（仅经典 CAN）。"""
from __future__ import annotations

import ctypes as C
import sys

PCAN_NONEBUS = 0x00
# USB 通道 1..8 = 0x51..0x58，9..16 = 0x509..0x510
USB_CHANNELS = {f"PCAN_USBBUS{i}": (0x50 + i if i <= 8 else 0x500 + i) for i in range(1, 17)}

BAUDRATES = {
    "1M": 0x0014, "800K": 0x0016, "500K": 0x001C, "250K": 0x011C, "125K": 0x031C,
}

PCAN_ERROR_OK = 0x00000
PCAN_ERROR_QRCVEMPTY = 0x00020
PCAN_ERROR_BUSLIGHT = 0x00004
PCAN_ERROR_BUSHEAVY = 0x00008
PCAN_ERROR_BUSOFF = 0x00010

PCAN_MESSAGE_STANDARD = 0x00
PCAN_MESSAGE_RTR = 0x01
PCAN_MESSAGE_EXTENDED = 0x02
PCAN_MESSAGE_STATUS = 0x80
PCAN_MESSAGE_ERRFRAME = 0x40

PCAN_ATTACHED_CHANNELS_COUNT = 0x2A
PCAN_ATTACHED_CHANNELS = 0x2B
PCAN_CHANNEL_CONDITION = 0x0D
PCAN_CHANNEL_AVAILABLE = 0x01
PCAN_CHANNEL_OCCUPIED = 0x02


class TPCANMsg(C.Structure):
    _fields_ = [("ID", C.c_uint), ("MSGTYPE", C.c_ubyte), ("LEN", C.c_ubyte),
                ("DATA", C.c_ubyte * 8)]


class TPCANTimestamp(C.Structure):
    _fields_ = [("millis", C.c_uint), ("millis_overflow", C.c_ushort), ("micros", C.c_ushort)]


class TPCANChannelInformation(C.Structure):
    _fields_ = [("channel_handle", C.c_ushort), ("device_type", C.c_ubyte),
                ("controller_number", C.c_ubyte), ("device_features", C.c_uint),
                ("device_name", C.c_char * 33), ("device_id", C.c_uint),
                ("channel_condition", C.c_uint)]


class PcanError(RuntimeError):
    pass


class PCANBasic:
    def __init__(self):
        if sys.platform != "win32":
            raise PcanError("仅支持 Windows (PCANBasic.dll)")
        try:
            self.dll = C.windll.LoadLibrary("PCANBasic")
        except OSError as e:
            raise PcanError("无法加载 PCANBasic.dll，请安装 PEAK PCAN 驱动 (www.peak-system.com)") from e
        d = self.dll
        d.CAN_Initialize.argtypes = [C.c_ushort, C.c_ushort, C.c_ubyte, C.c_uint, C.c_ushort]
        d.CAN_Initialize.restype = C.c_uint
        d.CAN_Uninitialize.argtypes = [C.c_ushort]
        d.CAN_Uninitialize.restype = C.c_uint
        d.CAN_Reset.argtypes = [C.c_ushort]
        d.CAN_Reset.restype = C.c_uint
        d.CAN_GetStatus.argtypes = [C.c_ushort]
        d.CAN_GetStatus.restype = C.c_uint
        d.CAN_Read.argtypes = [C.c_ushort, C.POINTER(TPCANMsg), C.POINTER(TPCANTimestamp)]
        d.CAN_Read.restype = C.c_uint
        d.CAN_Write.argtypes = [C.c_ushort, C.POINTER(TPCANMsg)]
        d.CAN_Write.restype = C.c_uint
        d.CAN_GetValue.argtypes = [C.c_ushort, C.c_ubyte, C.c_void_p, C.c_uint]
        d.CAN_GetValue.restype = C.c_uint
        d.CAN_GetErrorText.argtypes = [C.c_uint, C.c_ushort, C.c_char_p]
        d.CAN_GetErrorText.restype = C.c_uint

    # ---- 基本 API ----
    def initialize(self, ch: int, baud: int) -> int:
        return self.dll.CAN_Initialize(ch, baud, 0, 0, 0)

    def uninitialize(self, ch: int) -> int:
        return self.dll.CAN_Uninitialize(ch)

    def reset(self, ch: int) -> int:
        return self.dll.CAN_Reset(ch)

    def get_status(self, ch: int) -> int:
        return self.dll.CAN_GetStatus(ch)

    def read(self, ch: int):
        msg, ts = TPCANMsg(), TPCANTimestamp()
        r = self.dll.CAN_Read(ch, C.byref(msg), C.byref(ts))
        return r, msg, ts

    def write(self, ch: int, can_id: int, data: bytes, extended: bool = False,
              rtr: bool = False, dlc: int | None = None) -> int:
        msg = TPCANMsg()
        msg.ID = can_id
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED if extended else PCAN_MESSAGE_STANDARD
        if rtr:
            msg.MSGTYPE |= PCAN_MESSAGE_RTR
        msg.LEN = len(data) if dlc is None else dlc
        for i, b in enumerate(data[:8]):
            msg.DATA[i] = b
        return self.dll.CAN_Write(ch, C.byref(msg))

    def error_text(self, err: int) -> str:
        buf = C.create_string_buffer(256)
        if self.dll.CAN_GetErrorText(err, 0x09, buf) == PCAN_ERROR_OK:   # 0x09 = English
            return buf.value.decode("ascii", "replace")
        return f"PCAN error 0x{err:X}"

    def attached_channels(self) -> list[tuple[str, int, str]]:
        """返回 [(通道名, 句柄, 描述)]；失败时返回空列表。"""
        cnt = C.c_uint(0)
        try:
            if self.dll.CAN_GetValue(PCAN_NONEBUS, PCAN_ATTACHED_CHANNELS_COUNT,
                                     C.byref(cnt), C.sizeof(cnt)) != PCAN_ERROR_OK or cnt.value == 0:
                return []
            arr = (TPCANChannelInformation * cnt.value)()
            if self.dll.CAN_GetValue(PCAN_NONEBUS, PCAN_ATTACHED_CHANNELS, arr, C.sizeof(arr)) != PCAN_ERROR_OK:
                return []
        except Exception:
            return []
        name_by_handle = {v: k for k, v in USB_CHANNELS.items()}
        out = []
        for info in arr:
            h = info.channel_handle
            name = name_by_handle.get(h, f"0x{h:X}")
            busy = " (已占用)" if info.channel_condition & PCAN_CHANNEL_OCCUPIED else ""
            desc = f"{info.device_name.decode(errors='replace')} #{info.device_id}{busy}"
            out.append((name, h, desc))
        return out
