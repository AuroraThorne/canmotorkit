"""安全限制 + 急停面板。急停/超限 = 零扭矩并锁存，不失能。"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
                               QLabel, QPushButton, QVBoxLayout)

from canmotorkit.control.safety import SafetyLimits

from .widgets import CheckBox


class _BlankableDoubleSpinBox(QDoubleSpinBox):
    """允许断开电机时显示空白，而不是 Qt 默认的 0。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._blank = False

    def set_blank(self, blank: bool):
        self._blank = bool(blank)
        self.lineEdit().setText("" if self._blank else self.textFromValue(self.value()))
        self.update()

    def textFromValue(self, value):
        if self._blank:
            return ""
        return super().textFromValue(value)


def _spin(lo, hi, val, dec, step, suffix):
    s = _BlankableDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(dec)
    s.setSingleStep(step)
    s.setValue(val)
    s.setSuffix(suffix)
    s.setKeyboardTracking(False)
    return s


class SafetyPanel(QGroupBox):
    limits_changed = Signal(float)

    def __init__(self, parent=None):
        super().__init__("安全限制 / 急停", parent)
        self.worker = None
        self.driver = None
        self._output_kind = ""
        d = SafetyLimits()
        self.btn_estop = QPushButton("急停（零扭矩）  [Esc / 空格]")
        self.btn_estop.setFocusPolicy(Qt.NoFocus)
        self.btn_estop.setMinimumHeight(40)
        self.btn_estop.setStyleSheet("QPushButton{background:#c62828;color:white;font-weight:bold;"
                                     "font-size:15px}QPushButton:disabled{background:#9e9e9e}")
        self.btn_estop.setToolTip("发送 kp=kd=t=0 并锁存，电机保持使能（失能会猛刹车）。\n"
                                  "需要失能请点击上方「失能」。")
        self.lbl_state = QLabel("安全：")
        self.lbl_state.setWordWrap(True)
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setMinimumHeight(24)
        self.btn_reset = QPushButton("解除锁存")
        self.btn_reset.setFocusPolicy(Qt.NoFocus)
        self.btn_reset.setToolTip("解除后会恢复发送已应用的参数，请先确认参数")

        self.sp_vel = _spin(0, 1000, d.vel_max, 2, 1, " rad/s")
        self.sp_vel.setPrefix("± ")
        self.sp_vel.setToolTip("0 = 不检查")
        self.sp_current = _spin(0, 1000, d.out_max, 3, 0.1, " A")
        self.sp_current.setPrefix("± ")
        self.sp_current.setToolTip("大疆电调电流指令限幅；超出则限幅")
        self.sp_torque = _spin(0, 1000, d.out_max, 3, 0.1, " Nm")
        self.sp_torque.setPrefix("± ")
        self.sp_torque.setToolTip("达妙/HM 扭矩指令限幅；超出则限幅")
        self.chk_pos = CheckBox("启用位置窗口（多圈累计）")
        self.sp_pmin = _spin(-1000, 0, d.pos_min, 3, 0.5, " rad")
        self.sp_pmax = _spin(0, 1000, d.pos_max, 3, 0.5, " rad")
        self.btn_pzero = QPushButton("以当前位置为窗口零点")
        self.btn_pzero.setFocusPolicy(Qt.NoFocus)
        self.sp_temp = _spin(0, 150, d.temp_max, 0, 5, " ℃")
        self.sp_temp.setToolTip("0 = 不检查")
        self.sp_to = _spin(0, 1000, d.fb_timeout * 1000, 0, 10, " ms")
        self.sp_to.setToolTip("输出非零时连续无反馈的超时；0 = 不检查")
        f = QFormLayout()
        f.addRow("速度上限", self.sp_vel)
        f.addRow("电流限幅", self.sp_current)
        f.addRow("扭矩限幅", self.sp_torque)
        f.addRow(self.chk_pos)
        row = QHBoxLayout()
        row.addWidget(QLabel("Min"))
        row.addWidget(self.sp_pmin)
        row.addSpacing(8)
        row.addWidget(QLabel("Max"))
        row.addWidget(self.sp_pmax)
        f.addRow("位置窗口", row)
        f.addRow(self.btn_pzero)
        f.addRow("温度上限", self.sp_temp)
        f.addRow("反馈超时", self.sp_to)

        lay = QVBoxLayout(self)
        lay.addWidget(self.btn_estop)
        r2 = QHBoxLayout()
        r2.addWidget(self.lbl_state, 1)
        r2.addWidget(self.btn_reset)
        lay.addLayout(r2)
        lay.addLayout(f)

        self.btn_estop.clicked.connect(self.estop)
        self.btn_reset.clicked.connect(lambda: self.worker and self.worker.reset_safety())
        self.btn_pzero.clicked.connect(lambda: self.worker and self.worker.zero_safety_position())
        for w in (self.sp_vel, self.sp_current, self.sp_torque, self.sp_pmin, self.sp_pmax,
                  self.sp_temp, self.sp_to):
            w.valueChanged.connect(self._push)
        self.chk_pos.toggled.connect(self._push)
        self.set_driver(None)
        self.set_bound(None)

    def limits(self) -> SafetyLimits:
        return SafetyLimits(vel_max=self.sp_vel.value(), pos_enabled=self.chk_pos.isChecked(),
                            pos_min=self.sp_pmin.value(), pos_max=self.sp_pmax.value(),
                            out_max=(self.sp_current if self._output_kind == "current" else self.sp_torque).value()
                            if self._output_kind else 0.0,
                            temp_max=self.sp_temp.value(),
                            fb_timeout=self.sp_to.value() / 1000.0)

    def set_bound(self, worker):
        self.worker = worker
        on = worker is not None
        for w in (self.btn_estop, self.btn_reset, self.btn_pzero):
            w.setEnabled(on)
        if on:
            self._push()
        else:
            self.lbl_state.setText("安全：")
            self.lbl_state.setStyleSheet("")
        self._update_buttons()

    def set_driver(self, d):
        self.driver = d
        self._output_kind = "" if d is None else ("torque" if d.out_name == "扭矩" else "current")
        if d is None:
            for spin in (self.sp_vel, self.sp_current, self.sp_torque, self.sp_pmin,
                         self.sp_pmax, self.sp_temp, self.sp_to):
                self._clear_spin(spin)
            self.chk_pos.blockSignals(True)
            self.chk_pos.setChecked(False)
            self.chk_pos.setEnabled(False)
            self.chk_pos.blockSignals(False)
            self._update_buttons()
            self._push()
            return

        defaults = SafetyLimits()
        self._fill_spin(self.sp_vel, defaults.vel_max, "± ", " rad/s")
        self._fill_spin(self.sp_current, defaults.out_max, "± ", " A",
                        active=self._output_kind == "current")
        self._fill_spin(self.sp_torque, defaults.out_max, "± ", " Nm",
                        active=self._output_kind == "torque")
        self.chk_pos.blockSignals(True)
        self.chk_pos.setChecked(defaults.pos_enabled)
        self.chk_pos.setEnabled(True)
        self.chk_pos.blockSignals(False)
        self._fill_spin(self.sp_pmin, defaults.pos_min, "", " rad")
        self._fill_spin(self.sp_pmax, defaults.pos_max, "", " rad")
        self._fill_spin(self.sp_temp, defaults.temp_max, "", " ℃")
        self._fill_spin(self.sp_to, defaults.fb_timeout * 1000, "", " ms")
        self._update_buttons()
        self._push()

    @staticmethod
    def _clear_spin(spin):
        spin.blockSignals(True)
        spin.setEnabled(False)
        spin.setPrefix("")
        spin.setSuffix("")
        spin.setValue(0)
        spin.set_blank(True)
        spin.blockSignals(False)

    @staticmethod
    def _fill_spin(spin, value, prefix, suffix, active=True):
        spin.blockSignals(True)
        spin.setPrefix(prefix if active else "")
        spin.setSuffix(suffix if active else "")
        spin.setEnabled(active)
        if active:
            spin.set_blank(False)
            spin.setValue(value)
        else:
            spin.setValue(0)
            spin.set_blank(True)
        spin.blockSignals(False)

    def _update_buttons(self):
        on = self.worker is not None and self.driver is not None
        self.btn_estop.setEnabled(on)
        self.btn_pzero.setEnabled(on)
        if not on:
            self.btn_reset.setEnabled(False)

    def estop(self):
        if self.worker:
            self.worker.estop()

    def _push(self):
        lim = self.limits()
        if self.worker:
            self.worker.set_safety_limits(lim)
        self.limits_changed.emit(lim.out_max if self.driver is not None else 0.0)

    def on_state(self, s: dict):
        if self.worker is None:
            return
        if s.get("tripped"):
            self.lbl_state.setText("⛔ 已触发零扭矩：" + s.get("trip_reason", ""))
            self.lbl_state.setStyleSheet("background:#c62828;color:white;font-weight:bold;padding:2px")
            self.btn_reset.setEnabled(True)
        else:
            self.lbl_state.setText("安全：正常")
            self.lbl_state.setStyleSheet("background:#e8f5e9;color:#2e7d32;padding:2px")
            self.btn_reset.setEnabled(False)
