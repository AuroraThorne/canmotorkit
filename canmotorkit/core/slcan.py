"""SLCAN 总线实现和 Canable 设备枚举。

Canable2 当前使用 USB CDC 串口暴露 SLCAN 协议。COM 端口号由 Windows
动态分配，因此只按 USB VID/PID 和设备描述识别，不保存固定 COM 号。
"""
from __future__ import annotations

import time

from .bus import BusError, CanBus, Frame


CANABLE_VID = 0x16D0
CANABLE_PID = 0x117E
DEFAULT_SERIAL_BAUDRATE = 115200

CAN_BITRATES = {
    "1M": 1_000_000,
    "800K": 800_000,
    "500K": 500_000,
    "250K": 250_000,
    "125K": 125_000,
}

# SLCAN 的 S0~S8 固定速率编码。当前项目只需要经典 CAN。
SLCAN_BITRATE_COMMANDS = {
    10_000: "S0",
    20_000: "S1",
    50_000: "S2",
    100_000: "S3",
    125_000: "S4",
    250_000: "S5",
    500_000: "S6",
    800_000: "S7",
    1_000_000: "S8",
}


def canable_ports() -> list[dict]:
    """枚举当前连接的 Canable SLCAN 设备，返回动态 COM 端口信息。"""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []

    out = []
    for port in list_ports.comports():
        text = " ".join(str(x or "") for x in
                         (port.description, port.product, port.manufacturer, port.hwid)).lower()
        known_id = port.vid == CANABLE_VID and port.pid == CANABLE_PID
        if not known_id and "canable" not in text:
            continue
        desc = port.description or port.product or "Canable SLCAN"
        out.append({
            "port": port.device,
            "description": desc,
            "vid": port.vid,
            "pid": port.pid,
            "serial_number": port.serial_number,
            "hwid": port.hwid,
        })
    return out


def encode_slcan_frame(can_id: int, data: bytes, extended: bool = False,
                       rtr: bool = False, dlc: int | None = None) -> bytes:
    """编码经典 CAN 数据/远程帧，支持标准/扩展 ID。"""
    limit = 0x1FFFFFFF if extended else 0x7FF
    if not 0 <= can_id <= limit:
        raise ValueError(f"SLCAN CAN ID 超出范围: 0x{can_id:X}")
    if len(data) > 8:
        raise ValueError("经典 CAN 数据长度不能超过 8 字节")
    wire_dlc = len(data) if dlc is None else dlc
    if not 0 <= wire_dlc <= 8:
        raise ValueError("DLC 必须在 0~8 之间")
    prefix = "R" if extended and rtr else "T" if extended else "r" if rtr else "t"
    id_width = 8 if extended else 3
    payload = "" if rtr else bytes(data).hex().upper()
    return f"{prefix}{can_id:0{id_width}X}{wire_dlc:X}{payload}\r".encode("ascii")


def parse_slcan_frame(line: bytes | bytearray | str, with_flags: bool = False):
    """解析标准/扩展数据帧和远程帧。"""
    if isinstance(line, (bytes, bytearray)):
        line = bytes(line).decode("ascii", "ignore")
    line = line.strip()
    if not line or line[0] not in "tTrR":
        return None
    extended = line[0] in "TR"
    rtr = line[0] in "rR"
    if not with_flags and (extended or rtr):
        return None
    id_width = 8 if extended else 3
    if len(line) < 2 + id_width:
        return None
    try:
        can_id = int(line[1:1 + id_width], 16)
        dlc = int(line[1 + id_width], 16)
        payload = line[2 + id_width:2 + id_width + dlc * 2]
        if dlc > 8 or (not rtr and len(payload) != dlc * 2):
            return None
        result = can_id, b"" if rtr else bytes.fromhex(payload), extended, rtr, dlc
        return result if with_flags else result[:2]
    except ValueError:
        return None


class SlcanBus(CanBus):
    """Windows/Linux 均可用的 SLCAN 串口总线，适配 Canable CDC 固件。"""

    def __init__(self, port: str, bitrate: int = 1_000_000,
                 serial_baudrate: int = DEFAULT_SERIAL_BAUDRATE):
        self.port = str(port)
        self.bitrate = int(bitrate)
        self.serial_baudrate = int(serial_baudrate)
        self.name = f"SLCAN {self.port}"
        self._serial = None
        self._rx = bytearray()

    def open(self) -> None:
        if self.bitrate not in SLCAN_BITRATE_COMMANDS:
            raise BusError(f"SLCAN 不支持 CAN 波特率 {self.bitrate:g} bit/s")
        if self._serial is not None:
            return
        try:
            import serial
            ser = serial.Serial(self.port, baudrate=self.serial_baudrate,
                                timeout=0, write_timeout=0.2)
            self._serial = ser
            self._rx.clear()
            ser.reset_input_buffer()
            self._command("C")
            self._command(SLCAN_BITRATE_COMMANDS[self.bitrate])
            self._command("O")
            ser.reset_input_buffer()
        except Exception as e:
            if self._serial is not None:
                self._serial.close()
                self._serial = None
            if type(e).__module__.startswith("serial"):
                raise BusError(f"打开 SLCAN {self.port} 失败: {e}") from e
            raise BusError(f"打开 SLCAN 失败: {e}") from e

    def close(self) -> None:
        ser, self._serial = self._serial, None
        if ser is None:
            return
        try:
            ser.write(b"C\r")
            ser.flush()
        except Exception:
            pass
        ser.close()

    def _command(self, command: str) -> None:
        self._serial.write((command + "\r").encode("ascii"))
        self._serial.flush()

    def send(self, can_id: int, data: bytes, extended: bool = False,
             rtr: bool = False, dlc: int | None = None) -> None:
        if self._serial is None:
            raise BusError("SLCAN 尚未打开")
        try:
            self._serial.write(encode_slcan_frame(can_id, data, extended, rtr, dlc))
            self._serial.flush()
        except Exception as e:
            raise BusError(f"SLCAN 发送失败: {e}") from e

    def recv_all(self) -> list[Frame]:
        if self._serial is None:
            return []
        try:
            chunk = self._serial.read(4096)
        except Exception as e:
            raise BusError(f"SLCAN 接收失败: {e}") from e
        if chunk:
            self._rx.extend(chunk)
        out = []
        batch_t = time.perf_counter()
        while b"\r" in self._rx:
            raw, _, rest = self._rx.partition(b"\r")
            self._rx = bytearray(rest)
            decoded = parse_slcan_frame(raw, with_flags=True)
            if decoded is not None:
                can_id, data, extended, rtr, dlc = decoded
                # 串口帧本身没有硬件时间戳；同一批 read() 取出的帧统一使用
                # 批次时间，避免循环解析多帧时人为制造微秒级帧间隔。
                out.append(Frame(batch_t, can_id, data,
                                 extended=extended, rtr=rtr, dlc=dlc))
        return out

    def status_text(self) -> str:
        return "OK" if self._serial is not None else "未打开"
