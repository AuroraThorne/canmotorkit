"""主窗口：左栏（① PCAN 连接 → ② 总线电机 → ③ 安全限制/急停），右侧顶部电机卡片 + 标签页。"""
from __future__ import annotations

from PySide6.QtCore import QEvent, QObject, Qt
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (QAbstractSpinBox, QApplication, QLabel, QLineEdit, QMainWindow,
                               QMessageBox, QPlainTextEdit, QScrollArea, QSizePolicy, QSplitter,
                               QTabWidget, QTableWidget, QVBoxLayout, QWidget)

from canmotorkit import APP_NAME
from canmotorkit.core.bus import PcanBus
from canmotorkit.core.worker import Worker
from canmotorkit.sim.demo import DemoBus

from .connection_panel import ConnectionPanel
from .control_tab import ControlPanel
from .loop_tab import LoopTab
from .motor_card import MotorCard
from .motor_list import MotorListPanel
from .plot_widget import PlotWidget
from .register_tab import RegisterTab
from .safety_panel import SafetyPanel
from .scan_log_tab import ScanLogTab
from .test_tab import TestTab

TEXT_INPUTS = (QLineEdit, QAbstractSpinBox, QPlainTextEdit, QTableWidget)


class _SpaceEstop(QObject):
    """空格急停（焦点在输入控件时不触发）。"""

    def __init__(self, fn):
        super().__init__()
        self.fn = fn

    def eventFilter(self, obj, ev):
        if ev.type() == QEvent.KeyPress and ev.key() == Qt.Key_Space and not ev.isAutoRepeat():
            if not isinstance(QApplication.focusWidget(), TEXT_INPUTS):
                self.fn()
                return True
        return False


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1500, 920)
        self.bus = None
        self.worker: Worker | None = None
        self.driver = None

        self.conn = ConnectionPanel()
        self.motors = MotorListPanel()
        self.safety = SafetyPanel()
        self.safety.setTitle("③ 安全限制 / 急停")
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(4, 4, 4, 4)
        ll.addWidget(self.conn)
        ll.addWidget(self.motors)
        ll.addWidget(self.safety)
        ll.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidget(left)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(200)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.card = MotorCard()
        self.control = ControlPanel()
        self.mini_plot = PlotWidget(capacity=60_000, compact=True)
        ctl = QSplitter(Qt.Horizontal)
        cs = QScrollArea()
        cs.setWidget(self.control)
        cs.setWidgetResizable(True)
        cs.setMinimumWidth(500)
        cs.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        ctl.addWidget(cs)
        ctl.addWidget(self.mini_plot)
        ctl.setStretchFactor(1, 1)
        self.plot = PlotWidget()
        self.test = TestTab()
        self.loop = LoopTab()
        self.regs = RegisterTab()
        self.scan = ScanLogTab()
        self.tabs = QTabWidget()
        self.tabs.addTab(ctl, "控制")
        self.tabs.addTab(self.plot, "曲线")
        self.tabs.addTab(self.test, "响应测试")
        self.tabs.addTab(self.loop, "闭环测试")
        self.tabs.addTab(self.regs, "电机参数")
        self.tabs.addTab(self.scan, "报文")

        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.addWidget(self.card)
        rl.addWidget(self.tabs, 1)
        # 允许右栏压缩到可用的工作宽度，但禁止被外层 splitter 折叠。
        right.setMinimumWidth(700)
        right.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        sp = QSplitter(Qt.Horizontal)
        sp.addWidget(scroll)
        sp.addWidget(right)
        sp.setOpaqueResize(True)
        sp.setChildrenCollapsible(False)
        sp.setCollapsible(0, False)
        sp.setCollapsible(1, False)
        sp.setStretchFactor(1, 1)
        sp.setSizes([480, 1020])
        self.setCentralWidget(sp)

        sb = self.statusBar()
        self.st_conn = QLabel("未连接")
        self.st_conn.setStyleSheet("padding:0 8px")
        sb.addPermanentWidget(self.st_conn)

        self.panels = (self.motors, self.safety, self.card, self.control, self.test, self.loop,
                       self.regs, self.scan)
        self.conn.connect_requested.connect(self.connect_bus)
        self.conn.disconnect_requested.connect(self.disconnect_bus)
        self.motors.motor_selected.connect(self.select_motor)
        self.test.suggestion_ready.connect(self._use_suggestion)
        self.regs.ids_changed.connect(lambda *_: self.select_motor(None))
        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self.safety.estop,
                  context=Qt.ApplicationShortcut)
        self._space = _SpaceEstop(self.safety.estop)
        QApplication.instance().installEventFilter(self._space)
        if self.conn.pcan_error:
            self.statusBar().showMessage(self.conn.pcan_error + "（仍可使用演示台）", 15000)

    # ---------- 连接 ----------
    def connect_bus(self, p: dict):
        try:
            bus = DemoBus() if p["sim"] else PcanBus(p["channel"], p["baud"])
            bus.open()
        except Exception as e:
            QMessageBox.critical(self, "连接失败", str(e))
            return
        self.bus = bus
        w = Worker(bus)
        w.samples.connect(self._on_samples)
        w.frames.connect(self.scan.on_frames)
        w.reg_done.connect(self.regs.on_reg_done)
        w.scan_progress.connect(self.motors.on_scan_progress)
        w.scan_done.connect(self.motors.on_scan_done)
        w.state.connect(self._on_state)
        w.message.connect(self._on_message)
        w.limits_changed.connect(self._on_limits)
        w.run_finished.connect(self.test.on_run_finished)
        w.run_finished.connect(self.loop.on_run_finished)
        self.worker = w
        for p_ in self.panels:
            p_.set_bound(w)
        self.conn.set_connected(True)
        w.start()
        self.st_conn.setText(f"{bus.name} 已连接")
        self._on_message(f"已连接 {p['channel_name']}，请扫描或添加电机")
        if p["sim"]:
            self.motors.sp_to.setValue(8)

    def disconnect_bus(self):
        w = self.worker
        if w is not None:
            w.stop()
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
        self.driver = None
        for p_ in self.panels:
            p_.set_bound(None)
        self.conn.set_connected(False)
        self.st_conn.setText("未连接")

    def closeEvent(self, ev):
        self.disconnect_bus()
        super().closeEvent(ev)

    # ---------- 电机选择 ----------
    def select_motor(self, drv):
        w = self.worker
        if w is None:
            return
        if w.active:
            QMessageBox.information(self, "切换电机", "请先失能/停止当前电机")
            return
        self.driver = drv
        w.set_driver(drv)
        for p_ in (self.card, self.control, self.test, self.loop, self.regs, self.scan):
            p_.set_driver(drv)
        for pl in (self.plot, self.mini_plot):
            pl.clear()
            if drv is not None:
                pl.set_unit(drv.out_unit, drv.out_name)
        self.safety.set_unit(drv.out_unit if drv else "")
        if drv is not None:
            self.tabs.setCurrentIndex(0)

    # ---------- 回调 ----------
    def _on_samples(self, samples):
        self.plot.add_samples(samples)
        self.mini_plot.add_samples(samples)
        self.control.on_sample(samples[-1])

    def _on_state(self, s: dict):
        for p_ in (self.card, self.safety, self.motors, self.test, self.loop, self.regs):
            p_.on_state(s)
        run = self.worker._run_obj if self.worker else None
        target = ""
        if run is not None:
            cfg = getattr(run, "cfg", None)
            target = getattr(cfg, "loop", "") if cfg is not None and hasattr(cfg, "loop") else ""
        for pl in (self.plot, self.mini_plot):
            pl.set_ref_target(target)

    def _on_limits(self, lim, ok: bool):
        self.control.set_limits(lim, ok)
        self.regs.set_limits(lim, ok)

    def _on_message(self, msg: str):
        self.statusBar().showMessage(msg, 8000)
        self.scan.append_text("# " + msg)

    def _use_suggestion(self, s):
        self.loop.set_suggestion(s)
        self.tabs.setCurrentWidget(self.loop)
