"""Linux SocketCAN 总线实现（经典 CAN）。

SocketCAN 的接口和波特率由 Linux 网络层管理，例如：
    sudo ip link set can0 up type can bitrate 1000000

本模块只负责收发经典 CAN 帧，不参与电机协议解析。
"""
from __future__ import annotations

import errno
import socket
import struct
import time

from .bus import BusError, CanBus, Frame


CAN_MTU = 16
CAN_FRAME = struct.Struct("=IB3x8s")
CAN_INTERFACE_TYPE = 280       # Linux ARPHRD_CAN
CAN_EFF_FLAG = getattr(socket, "CAN_EFF_FLAG", 0x80000000)
CAN_RTR_FLAG = getattr(socket, "CAN_RTR_FLAG", 0x40000000)
CAN_ERR_FLAG = getattr(socket, "CAN_ERR_FLAG", 0x20000000)


def socketcan_interfaces() -> list[str]:
    """返回本机可见的 SocketCAN 接口名；非 Linux/无 SocketCAN 时返回空列表。"""
    if not hasattr(socket, "AF_CAN"):
        return []
    try:
        names = []
        for _, name in socket.if_nameindex():
            type_path = f"/sys/class/net/{name}/type"
            try:
                with open(type_path, encoding="ascii") as f:
                    if int(f.read().strip()) != CAN_INTERFACE_TYPE:
                        continue
            except (OSError, ValueError):
                continue
            names.append(name)
        return sorted(names)
    except OSError:
        return []


class SocketCanBus(CanBus):
    """基于 Linux AF_CAN/CAN_RAW 的经典 CAN 总线。"""

    def __init__(self, interface: str):
        self.interface = str(interface)
        self.name = f"SocketCAN {self.interface}"
        self.sock: socket.socket | None = None

    @staticmethod
    def pack_frame(can_id: int, data: bytes) -> bytes:
        if not 0 <= can_id <= 0x7FF:
            raise ValueError(f"仅支持标准 11 位 CAN ID: 0x{can_id:X}")
        if len(data) > 8:
            raise ValueError("经典 CAN 数据长度不能超过 8 字节")
        return CAN_FRAME.pack(can_id, len(data), bytes(data).ljust(8, b"\x00"))

    @staticmethod
    def unpack_frame(raw: bytes) -> tuple[int, bytes] | None:
        if len(raw) < CAN_MTU:
            return None
        can_id, dlc, data = CAN_FRAME.unpack(raw[:CAN_MTU])
        if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
            return None
        return can_id, data[:min(dlc, 8)]

    def open(self) -> None:
        if not hasattr(socket, "AF_CAN"):
            raise BusError("SocketCAN 仅支持 Linux")
        if self.sock is not None:
            return
        sock = None
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            # 与 PCAN 默认行为保持一致，不把本机发送帧回显为接收帧。
            try:
                sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_LOOPBACK, 0)
            except (AttributeError, OSError):
                pass
            sock.bind((self.interface,))
            sock.setblocking(False)
            self.sock = sock
        except OSError as e:
            if sock is not None:
                sock.close()
            raise BusError(f"打开 SocketCAN {self.interface} 失败: {e}") from e

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def send(self, can_id: int, data: bytes) -> None:
        if self.sock is None:
            raise BusError("SocketCAN 尚未打开")
        try:
            n = self.sock.send(self.pack_frame(can_id, data))
        except OSError as e:
            raise BusError(f"SocketCAN 发送失败: {e}") from e
        if n != CAN_MTU:
            raise BusError(f"SocketCAN 发送长度异常: {n}")

    def recv_all(self) -> list[Frame]:
        if self.sock is None:
            return []
        out = []
        for _ in range(512):
            try:
                raw = self.sock.recv(CAN_MTU)
            except BlockingIOError:
                break
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise BusError(f"SocketCAN 接收失败: {e}") from e
            decoded = self.unpack_frame(raw)
            if decoded is None:
                continue
            can_id, data = decoded
            out.append(Frame(time.perf_counter(), can_id, data))
        return out

    def status_text(self) -> str:
        return "OK" if self.sock is not None else "未打开"
