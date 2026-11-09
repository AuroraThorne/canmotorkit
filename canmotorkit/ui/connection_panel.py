"""连接面板：通道/波特率/电机 ID/Master ID/型号预设。"""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFormLayout, QGroupBox, QHBoxLayout,
                               QPushButton, QSpinBox, QWidget)

from canmotorkit.core.pcan_basic import BAUDRATES, USB_CHANNELS

SIM_LABEL = "演示台（虚拟电机，无需硬件）"


class ConnectionPanel(QGroupBox):
    connect_requested = Signal(dict)
    disconnect_requested = Signal()
    strict_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__("连接", parent)
        self.pcan_error = ""
        self.cb_channel = QComboBox()
        self.btn_refresh = QPushButton("刷新")
        ch_row = QWidget()
        h = QHBoxLayout(ch_row)
        h.setContentsMargins(0, 0, 0, 0)
        h.addWidget(self.cb_channel, 1)
        h.addWidget(self.btn_refresh)

        self.cb_baud = QComboBox()
        self.cb_baud.addItems(list(BAUDRATES))
        self.cb_baud.setToolTip("CAN 版电机固定 1 Mbps（仅 CAN FD 版支持改波特率）")
        self.sp_id = QSpinBox()
        self.sp_id.setRange(1, 127)
        self.sp_id.setValue(1)
        self.sp_id.setToolTip("电机接收 ID（控制帧帧 ID），出厂默认 1")
        self.sp_master = QSpinBox()
        self.sp_master.setRange(0, 0x7FE)
        self.sp_master.setValue(0)
        self.sp_master.setToolTip("电机应答帧 ID（MASTER_ID），出厂默认 0")
        self.chk_strict = QCheckBox("校验状态帧 ID 低 4 位")
        self.btn_connect = QPushButton("连接")
        self.btn_connect.setMinimumHeight(30)

        f = QFormLayout(self)
        f.addRow("通道", ch_row)
        f.addRow("波特率", self.cb_baud)
        f.addRow("电机 ID", self.sp_id)
        f.addRow("Master ID", self.sp_master)
        f.addRow(self.chk_strict)
        f.addRow(self.btn_connect)

        self._connected = False
        self.btn_refresh.clicked.connect(self.refresh_channels)
        self.btn_connect.clicked.connect(self._on_click)
        self.chk_strict.toggled.connect(self.strict_changed)
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
        return {
            "sim": kind == "sim",
            "channel": handle,
            "channel_name": self.cb_channel.currentText(),
            "baud": BAUDRATES[self.cb_baud.currentText()],
            "motor_id": self.sp_id.value(),
            "master_id": self.sp_master.value(),
            "strict": self.chk_strict.isChecked(),
        }

    def _on_click(self):
        if self._connected:
            self.disconnect_requested.emit()
        else:
            self.connect_requested.emit(self.params())

    def set_connected(self, on: bool):
        self._connected = on
        self.btn_connect.setText("断开" if on else "连接")
        for w in (self.cb_channel, self.btn_refresh, self.cb_baud, self.sp_id, self.sp_master):
            w.setEnabled(not on)

    def set_ids(self, motor_id: int, master_id: int):
        self.sp_id.setValue(motor_id)
        self.sp_master.setValue(master_id)
