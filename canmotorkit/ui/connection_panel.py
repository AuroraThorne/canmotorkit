"""① CAN 连接：PCAN / SLCAN / SocketCAN 通道（含「演示台」）/ 波特率 / 连接。"""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QComboBox, QFormLayout, QGroupBox, QHBoxLayout, QPushButton, QWidget

from canmotorkit.core.pcan_basic import BAUDRATES, USB_CHANNELS
from canmotorkit.core.slcan import CAN_BITRATES, canable_ports
from canmotorkit.core.socketcan import socketcan_interfaces

SIM_LABEL = "演示台（虚拟电机，无需硬件）"


class ConnectionPanel(QGroupBox):
    connect_requested = Signal(dict)
    disconnect_requested = Signal()

    def __init__(self, parent=None):
        super().__init__("① CAN 连接", parent)
        self.pcan_error = ""
        self.cb_channel = QComboBox()
        self.cb_channel.setToolTip("「演示台」包含 HM9816G9、达妙 J8009P、大疆 M3508/M2006/GM6020 虚拟电机")
        self.btn_refresh = QPushButton("刷新")
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 0, 0, 0)
        h.addWidget(self.cb_channel, 1)
        h.addWidget(self.btn_refresh)
        self.cb_baud = QComboBox()
        self.cb_baud.addItems(list(BAUDRATES))
        self.cb_baud.setToolTip("PCAN 由本程序设置；SocketCAN 由 Linux ip link 设置")
        self.btn_connect = QPushButton("连接")
        self.btn_connect.setMinimumHeight(30)
        f = QFormLayout(self)
        f.addRow("通道", row)
        f.addRow("波特率", self.cb_baud)
        f.addRow(self.btn_connect)
        self._connected = False
        self.btn_refresh.clicked.connect(self.refresh_channels)
        self.btn_connect.clicked.connect(self._on_click)
        self.cb_channel.currentIndexChanged.connect(self._channel_changed)
        self.refresh_channels()

    def refresh_channels(self):
        self.cb_channel.clear()
        attached = []
        try:
            from canmotorkit.core.pcan_basic import PCANBasic
            attached = PCANBasic().attached_channels()
            self.pcan_error = ""
        except Exception as e:
            self.pcan_error = str(e)
        for name, handle, desc in attached:
            self.cb_channel.addItem(f"{name}  {desc}", ("pcan", handle))
        if not attached:
            for i in range(1, 5):
                name = f"PCAN_USBBUS{i}"
                self.cb_channel.addItem(f"{name} (未检测到)", ("pcan", USB_CHANNELS[name]))
        socket_names = socketcan_interfaces()
        for name in socket_names:
            self.cb_channel.addItem(f"SocketCAN {name}（波特率由系统配置）", ("socketcan", name))
        canable_devices = canable_ports()
        for dev in canable_devices:
            self.cb_channel.addItem(f"SLCAN（{dev['port']}）", ("slcan", dev["port"]))
        self.cb_channel.addItem(SIM_LABEL, ("sim", 0))
        if not attached:
            real_index = next((i for i in range(self.cb_channel.count())
                               if self.cb_channel.itemData(i)[0] in ("socketcan", "slcan")), None)
            if real_index is None:
                self.select_sim()
            else:
                self.cb_channel.setCurrentIndex(real_index)
        self._channel_changed()

    def select_sim(self):
        self.cb_channel.setCurrentIndex(self.cb_channel.count() - 1)

    def params(self) -> dict:
        kind, handle = self.cb_channel.currentData()
        return {"kind": kind, "sim": kind == "sim", "channel": handle,
                "channel_name": self.cb_channel.currentText(),
                "baud": BAUDRATES[self.cb_baud.currentText()],
                "bitrate": CAN_BITRATES[self.cb_baud.currentText()]}

    def _channel_changed(self):
        data = self.cb_channel.currentData()
        is_socketcan = bool(data and data[0] == "socketcan")
        self.cb_baud.setEnabled(not is_socketcan)
        if is_socketcan:
            self.cb_baud.setToolTip("SocketCAN 波特率由 Linux 配置，例如 can0 使用 ip link set can0 ...")
        elif data and data[0] == "slcan":
            self.cb_baud.setToolTip("CANable 的串口默认 115200；这里设置 CAN 总线波特率")
        else:
            self.cb_baud.setToolTip("PCAN 由本程序设置；达妙/HM/大疆经典 CAN 通常为 1 Mbps")

    def _on_click(self):
        if self._connected:
            self.disconnect_requested.emit()
        else:
            self.connect_requested.emit(self.params())

    def set_connected(self, on: bool):
        self._connected = on
        self.btn_connect.setText("断开" if on else "连接")
        for w in (self.cb_channel, self.btn_refresh, self.cb_baud):
            w.setEnabled(not on)
