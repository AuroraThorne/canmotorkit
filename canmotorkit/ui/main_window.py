"""主窗口：左侧连接+控制，右侧标签页，底部状态栏。"""
from __future__ import annotations

from PySide6.QtCore import QEvent, QObject, Qt
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (QAbstractSpinBox, QApplication, QLabel, QLineEdit, QMainWindow,
                               QMessageBox, QPlainTextEdit, QScrollArea, QSplitter, QTabWidget,
                               QTableWidget, QVBoxLayout, QWidget)

from canmotorkit.drivers import dm_protocol as P
from canmotorkit.core.bus import BusError, PcanBus, SimBus
from canmotorkit.core.worker import Worker

from .connection_panel import ConnectionPanel
from .control_tab import ControlPanel
from .plot_widget import PlotWidget
from .register_tab import RegisterTab
from .safety_panel import SafetyPanel
from .scan_log_tab import ScanLogTab
from .test_tab import TestTab

TEXT_INPUTS = (QLineEdit, QAbstractSpinBox, QPlainTextEdit, QTableWidget)


class _SpaceEstop(QObject):
    """空格键急停（焦点在文本输入控件时不触发，避免误伤输入）。"""

    def __init__(self, fn):
        super().__init__()
        self.fn = fn

    def eventFilter(self, obj, ev):
        if ev.type() == QEvent.KeyPress and ev.key() == Qt.Key_Space and not ev.isAutoRepeat():
            fw = QApplication.focusWidget()
            if not isinstance(fw, TEXT_INPUTS):
                self.fn()
                return True
        return False


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CanMotorKit")
        self.resize(1400, 860)
        self.bus = None
        self.worker: Worker | None = None

        self.conn = ConnectionPanel()
        self.control = ControlPanel()
        self.safety = SafetyPanel()
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(4, 4, 4, 4)
        ll.addWidget(self.conn)
        ll.addWidget(self.safety)
        ll.addWidget(self.control)
        scroll = QScrollArea()
        scroll.setWidget(left)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(470)

        self.plot = PlotWidget()
        self.regs = RegisterTab()
        self.test = TestTab()
        self.test.control = self.control
        self.scan = ScanLogTab()
        self.tabs = QTabWidget()
        self.tabs.addTab(self.plot, "实时曲线")
        self.tabs.addTab(self.regs, "参数寄存器")
        self.tabs.addTab(self.test, "测试信号")
        self.tabs.addTab(self.scan, "扫描 / 报文")

        sp = QSplitter(Qt.Horizontal)
        sp.addWidget(scroll)
        sp.addWidget(self.tabs)
        sp.setStretchFactor(1, 1)
        sp.setSizes([480, 920])
        self.setCentralWidget(sp)

        sb = self.statusBar()
        self.st_conn = QLabel("未连接")
        self.st_en = QLabel()
        self.st_link = QLabel()
        self.st_err = QLabel()
        self.st_bus = QLabel()
        for w in (self.st_conn, self.st_en, self.st_link, self.st_err, self.st_bus):
            w.setStyleSheet("padding:0 8px")
            sb.addPermanentWidget(w)

        # 信号
        self.conn.connect_requested.connect(self.connect_bus)
        self.conn.disconnect_requested.connect(self.disconnect_bus)
        self.conn.strict_changed.connect(lambda on: self.worker and setattr(self.worker, "strict_id", on))
        self.regs.ids_changed.connect(self.conn.set_ids)
        self.scan.use_motor.connect(self._use_motor)

        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self.safety.estop,
                  context=Qt.ApplicationShortcut)
        self._space = _SpaceEstop(self.safety.estop)
        QApplication.instance().installEventFilter(self._space)

        if self.conn.pcan_error:
            self.statusBar().showMessage(self.conn.pcan_error + "（仍可使用仿真总线）", 15000)

    # ---------- 连接 ----------
    def connect_bus(self, p: dict):
        try:
            bus = SimBus() if p["sim"] else PcanBus(p["channel"], p["baud"])
            bus.open()
        except (BusError, Exception) as e:
            QMessageBox.critical(self, "连接失败", str(e))
            return
        self.bus = bus
        w = Worker(bus, p["motor_id"], p["master_id"])
        w.strict_id = p["strict"]
        w.samples.connect(self._on_samples)
        w.frames.connect(self.scan.on_frames)
        w.reg_done.connect(self.regs.on_reg_done)
        w.scan_progress.connect(self.scan.on_scan_progress)
        w.scan_done.connect(self.scan.on_scan_done)
        w.state.connect(self._on_state)
        w.message.connect(self._on_message)
        w.limits_changed.connect(self.control.set_limits)
        w.signal_finished.connect(self.test.on_finished)
        self.worker = w
        self.control.set_limits(P.DEFAULT_LIMITS, False)
        for panel in (self.control, self.safety, self.regs, self.test, self.scan):
            panel.set_bound(w)
        self.conn.set_connected(True)
        self.plot.clear()
        w.start()
        self.st_conn.setText(f"{bus.name} | 电机 ID {w.motor_id} | Master 0x{w.master_id:X}")
        self._on_message(f"已连接 {p['channel_name']}")

    def disconnect_bus(self):
        w = self.worker
        if w is not None:
            w.stop()                # 若使能会先发失能
            if not w.wait(1500):
                w.terminate()
                w.wait(500)
            self.worker = None
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None
        for panel in (self.control, self.safety, self.regs, self.test, self.scan):
            panel.set_bound(None)
        self.conn.set_connected(False)
        self.st_conn.setText("未连接")
        for l in (self.st_en, self.st_link, self.st_err, self.st_bus):
            l.setText("")
            l.setStyleSheet("padding:0 8px")

    def closeEvent(self, ev):
        self.disconnect_bus()
        super().closeEvent(ev)

    # ---------- 回调 ----------
    def _on_samples(self, samples):
        self.plot.add_samples(samples)
        self.test.on_samples(samples)
        self.control.on_status(samples[-1].st)

    def _on_state(self, s: dict):
        self.safety.on_state(s)
        self.control.on_state(s)
        self.regs.on_state(s)
        self.scan.on_state(s)
        pad = "padding:0 8px;"
        self.st_en.setText("● 已使能" if s["enabled"] else "○ 未使能")
        self.st_en.setStyleSheet(pad + ("color:#2e7d32;font-weight:bold" if s["enabled"] else ""))
        if s["scanning"]:
            self.st_link.setText("扫描中…")
            self.st_link.setStyleSheet(pad)
        elif s["link_ok"]:
            self.st_link.setText("反馈正常")
            self.st_link.setStyleSheet(pad + "color:#2e7d32")
        else:
            self.st_link.setText("⚠ 反馈超时(>200ms)")
            self.st_link.setStyleSheet(pad + "color:white;background:#c62828")
        fault = P.is_fault(s["code"])
        self.st_err.setText(f"状态码 {s['code']} {P.code_text(s['code'])}" if s["code"] >= 0 else "")
        self.st_err.setStyleSheet(pad + ("color:white;background:#c62828" if fault else ""))
        self.st_bus.setText(f"总线: {s['bus']}")

    def _on_message(self, msg: str):
        self.statusBar().showMessage(msg, 8000)
        self.scan.append_text("# " + msg)

    def _use_motor(self, motor_id: int, master_id: int):
        if not self.worker:
            return
        if self.worker.enabled:
            QMessageBox.information(self, "切换电机", "请先失能")
            return
        self.worker.set_ids(motor_id, master_id)
        self.conn.set_ids(motor_id, master_id)
        self.plot.clear()
        self.st_conn.setText(f"{self.bus.name} | 电机 ID {motor_id} | Master 0x{master_id:X}")
        self._on_message(f"已切换到电机 ID {motor_id}")
