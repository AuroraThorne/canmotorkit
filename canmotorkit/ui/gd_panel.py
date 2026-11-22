"""GD 专用查询/系统设置，以及电机内部位置/速度控制入口。"""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
                               QPushButton, QLabel, QLineEdit, QInputDialog,
                               QComboBox, QDoubleSpinBox, QMessageBox)
from canmotorkit.drivers import gd


class GdSettings(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = self.driver = None
        self.active = self.busy = False
        layout = QVBoxLayout(self)
        form = QFormLayout()
        self.fields = {}
        for kind, title in (("model", "电机返回型号"), ("firmware", "固件版本"), ("hardware", "硬件版本")):
            field = QLineEdit()
            field.setReadOnly(True)
            self.fields[kind] = field
            form.addRow(title, field)
        layout.addLayout(form)
        self.buttons = {}
        # 每行最多两个按钮，避免中间栏变窄后横向溢出。
        for pairs in ((("read", "读取型号 / 版本"), ("change_id", "修改ID")),
                      (("save", "保存到 Flash"), ("reboot", "软重启")),
                      (("zero", "重设零位"),)):
            row = QHBoxLayout()
            for kind, title in pairs:
                button = QPushButton(title)
                button.clicked.connect(lambda checked=False, k=kind: self.action(k))
                self.buttons[kind] = button
                row.addWidget(button, 1)
            layout.addLayout(row)
        period = QFormLayout()
        self.period_us = QLineEdit("1000")
        period.addRow("反馈周期（μs）", self.period_us)
        layout.addLayout(period)
        row = QHBoxLayout()
        for kind, title in (("periodic", "应用周期"), ("periodic_off", "关闭周期反馈")):
            button = QPushButton(title)
            button.clicked.connect(lambda checked=False, k=kind: self.action(k))
            self.buttons[kind] = button
            row.addWidget(button, 1)
        layout.addLayout(row)
        note = QLabel("周期最小 100 μs，0 表示关闭；设置后仍保留主动状态查询。")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.result = QLabel()
        self.result.setWordWrap(True)
        self.result.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.result)
        layout.addStretch(1)

    def bind(self, worker, driver):
        self.worker, self.driver = worker, driver
        self.active = self.busy = False
        self.result.clear()
        for field in self.fields.values():
            field.clear()
        if isinstance(driver, gd.GdDriver):
            for kind, value in driver.identity.items():
                self.show_identity(kind, value)
        self._enabled()

    def on_state(self, s):
        self.active = bool(s.get("active"))
        self.busy = bool(s.get("gd_busy") or s.get("scanning"))
        self._enabled()

    def _enabled(self):
        enabled = self.worker is not None and isinstance(self.driver, gd.GdDriver) and not self.active and not self.busy
        for button in self.buttons.values():
            button.setEnabled(enabled)
        self.period_us.setEnabled(enabled)

    def action(self, kind):
        if not self.worker or not isinstance(self.driver, gd.GdDriver) or self.active or self.busy:
            return
        if kind == "read":
            for query in gd.QUERIES:
                self.worker.gd_request(query)
            self.result.setText("正在读取型号和版本…")
            return
        value = None
        if kind == "change_id":
            dialog = QInputDialog(self)
            dialog.setWindowTitle("修改ID")
            dialog.setLabelText(f"新电机 ID（当前 {self.driver.motor_id}）")
            dialog.setInputMode(QInputDialog.IntInput)
            dialog.setIntRange(1, 126)
            dialog.setIntValue(self.driver.motor_id)
            dialog.setOkButtonText("写入")
            dialog.setCancelButtonText("取消")
            if not dialog.exec():
                return
            value = dialog.intValue()
        elif kind in ("periodic", "periodic_off"):
            try:
                value = 0 if kind == "periodic_off" else int(self.period_us.text().strip())
                self.driver.system_frame("periodic", value)
            except ValueError as e:
                self.result.setText(str(e))
                return
            kind = "periodic"
        elif kind in ("save", "zero", "reboot"):
            text = {"save": "将当前电机设置保存到 Flash？", "zero": "将当前位置重设为零位？",
                    "reboot": "软重启当前电机？重启后周期反馈可能需要重新配置。"}[kind]
            if QMessageBox.question(self, self.buttons[kind].text(), text) != QMessageBox.Yes:
                return
        self.worker.gd_request(kind, value)
        self.result.setText("正在处理…")

    def show_identity(self, kind, value):
        if kind == "model":
            self.fields[kind].setText(value["text"] + ("" if value["complete"] else "（返回不完整）"))
        else:
            self.fields[kind].setText(value)

    def on_done(self, kind, value, error):
        if not isinstance(self.driver, gd.GdDriver):
            return
        if error:
            self.result.setText(error)
        elif kind in gd.QUERIES:
            self.show_identity(kind, value)
            self.result.setText("型号 / 版本已更新")
        elif kind == "change_id":
            self.result.setText(f"ID 已改为 {value}，已收到新 ID 状态；如需掉电保持，请保存到 Flash。")
        elif kind == "periodic":
            self.result.setText("周期设置已发送（协议无独立确认），可在报文页观察反馈帧率。")
        elif kind == "reboot":
            self.result.setText("重启命令已发送（协议无重启应答），请核对反馈恢复。")
        else:
            self.result.setText("电机已确认：" + {"zero": "重设零位", "save": "保存设置"}.get(kind, kind))


class GdControl(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = self.driver = None
        self.enabled = False
        layout = QVBoxLayout(self)
        self.mode = QComboBox()
        for label, kind in (("速度", "velocity"), ("位置", "position"),
                            ("位置 + 速度上限 + 扭矩上限", "position_velocity_torque"),
                            ("自由停转", "coast"), ("阻尼制动", "brake")):
            self.mode.addItem(label, kind)
        layout.addWidget(self.mode)
        self.form = QFormLayout()
        self.fields = {}
        for key, label, lo, hi, unit in (
            ("position", "目标位置", -32768 * gd.TAU / 10000, 32767 * gd.TAU / 10000, "rad"),
            ("velocity", "速度 / 速度上限", -32768 * gd.TAU / 4000, 32767 * gd.TAU / 4000, "rad/s"),
            ("torque", "扭矩上限", 0, 100, "Nm")):
            spin = QDoubleSpinBox()
            spin.setDecimals(6)
            spin.setRange(lo, hi)
            spin.setSuffix(" " + unit)
            spin.setKeyboardTracking(False)
            self.fields[key] = spin
            self.form.addRow(label, spin)
        layout.addLayout(self.form)
        row = QHBoxLayout()
        self.apply = QPushButton("应用")
        self.zero = QPushButton("零扭矩")
        row.addWidget(self.apply, 1)
        row.addWidget(self.zero, 1)
        layout.addLayout(row)
        self.note = QLabel()
        self.note.setWordWrap(True)
        layout.addWidget(self.note)
        missing = QLabel("电压、电流、带加速度模式：待补齐厂家换算表后开放。")
        missing.setWordWrap(True)
        layout.addWidget(missing)
        layout.addStretch(1)
        self.mode.currentIndexChanged.connect(self._mode_changed)
        self.apply.clicked.connect(self._apply)
        self.zero.clicked.connect(lambda: self.worker and self.worker.zero_output())
        self._mode_changed()

    def bind(self, worker, driver):
        self.worker, self.driver = worker, driver
        self.reset()
        self.on_state({})

    def reset(self):
        for field in self.fields.values():
            field.setValue(0)

    def on_state(self, s):
        active = bool(s.get("active") and not s.get("tripped"))
        if self.enabled and not active:
            self.reset()
        self.enabled = active
        self.apply.setEnabled(active)
        self.zero.setEnabled(active)
        for field in self.fields.values():
            field.setEnabled(self.worker is not None and isinstance(self.driver, gd.GdDriver))

    def _mode_changed(self):
        kind = self.mode.currentData()
        self.form.setRowVisible(self.fields["position"], kind in ("position", "position_velocity_torque"))
        self.form.setRowVisible(self.fields["velocity"], kind in ("velocity", "position_velocity_torque"))
        self.form.setRowVisible(self.fields["torque"], kind == "position_velocity_torque")
        self.fields["velocity"].setMinimum(0 if kind == "position_velocity_torque" else -32768 * gd.TAU / 4000)
        self.note.setText({"velocity": "电机内部速度环；此命令不带扭矩上限。",
                           "position": "电机内部位置环；此命令不带速度、扭矩上限。",
                           "position_velocity_torque": "按设置的速度和扭矩上限到达目标位置，受左侧限幅保护。",
                           "coast": "三相断开，自由停转。", "brake": "三相接地，产生阻尼制动；急停始终使用零扭矩。"}[kind])
        if self.worker and self.enabled:
            self.worker.zero_output()

    def _apply(self):
        if self.worker and self.enabled:
            for field in self.fields.values():
                field.interpretText()
            self.worker.set_gd_control(self.mode.currentData(), {k: w.value() for k, w in self.fields.items()})
