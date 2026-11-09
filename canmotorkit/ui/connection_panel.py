"""① PCAN 连接：通道（含「演示台」）/ 波特率 / 连接。"""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QComboBox, QFormLayout, QGroupBox, QHBoxLayout, QPushButton, QWidget

from canmotorkit.core.pcan_basic import BAUDRATES, USB_CHANNELS

SIM_LABEL = "演示台（虚拟电机，无需硬件）"


class ConnectionPanel(QGroupBox):
    connect_requested = Signal(dict)
    disconnect_requested = Signal()

    def __init__(self, parent=None):
        super().__init__("① PCAN 连接", parent)
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
        self.cb_baud.setToolTip("达妙/HM/大疆经典 CAN 均为 1 Mbps")
        self.btn_connect = QPushButton("连接")
        self.btn_connect.setMinimumHeight(30)
        f = QFormLayout(self)
        f.addRow("通道", row)
        f.addRow("波特率", self.cb_baud)
        f.addRow(self.btn_connect)
        self._connected = False
        self.btn_refresh.clicked.connect(self.refresh_channels)
        self.btn_connect.clicked.connect(self._on_click)
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
        self.cb_channel.addItem(SIM_LABEL, ("sim", 0))

    def select_sim(self):
        self.cb_channel.setCurrentIndex(self.cb_channel.count() - 1)

    def params(self) -> dict:
        kind, handle = self.cb_channel.currentData()
        return {"sim": kind == "sim", "channel": handle, "channel_name": self.cb_channel.currentText(),
                "baud": BAUDRATES[self.cb_baud.currentText()]}

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
