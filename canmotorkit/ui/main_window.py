"""主窗口：左栏（① PCAN 连接 → ② 总线电机 → ③ 安全限制/急停），右侧顶部电机卡片 + 标签页。"""
from __future__ import annotations

import sys

from PySide6.QtCore import QEvent, QObject, QPoint, QSettings, QTimer, Qt
from PySide6.QtGui import QAction, QActionGroup, QKeySequence, QPalette, QShortcut
from PySide6.QtWidgets import (QAbstractSpinBox, QApplication, QLabel, QLineEdit, QMainWindow,
                               QHBoxLayout, QMenu, QMessageBox, QPlainTextEdit, QScrollArea,
                               QSizePolicy, QSplitter, QTabWidget, QTableWidget, QPushButton,
                               QVBoxLayout, QWidget)

from canmotorkit import APP_NAME
from canmotorkit.core.bus import PcanBus
from canmotorkit.core.worker import Worker
from canmotorkit.core.slcan import SlcanBus
from canmotorkit.core.socketcan import SocketCanBus
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

MAIN_STYLE = """
QMainWindow {
    background: #171717;
    color: #f1f1f1;
}
QWidget {
    color: #f1f1f1;
}
QWidget#leftColumn, QScrollArea#leftScroll,
QScrollArea#leftScroll > QWidget > QWidget {
    background: #2b2b2b;
}
QWidget#rightColumn, QTabWidget#mainTabs,
QTabWidget#mainTabs::pane, QTabWidget#mainTabs QStackedWidget,
QTabWidget#mainTabs QStackedWidget > QWidget {
    background: #171717;
}
QScrollArea#controlScroll, QScrollArea#controlScroll > QWidget > QWidget,
QWidget#controlPage {
    background: #171717;
}
QWidget#controlPage QTabWidget,
QWidget#controlPage QTabWidget::pane,
QWidget#controlPage QTabWidget QStackedWidget,
QWidget#controlPage QTabWidget QStackedWidget > QWidget {
    background: #242424;
}
QScrollArea#loopScroll, QScrollArea#loopScroll > QWidget > QWidget,
QSplitter#loopSplitter > QWidget,
QSplitter#testSplitter > QWidget {
    background: #171717;
}
QSplitter#mainSplitter::handle {
    background: transparent;
}
QSplitter#loopSplitter::handle {
    background: transparent;
}
QSplitter#mainSplitter::handle:hover,
QSplitter#loopSplitter::handle:hover {
    background: transparent;
}
QGroupBox, QFrame {
    background: #242424;
    border: 1px solid #444444;
    border-radius: 3px;
}
QGroupBox {
    padding-top: 8px;
}
QGroupBox::title {
    color: #f1f1f1;
    subcontrol-origin: padding;
    subcontrol-position: top left;
    padding: 0 4px;
    background: #242424;
}
QTabWidget::pane {
    background: #202020;
    border: 1px solid #444444;
}
QProgressBar {
    background: #202020;
    color: #f1f1f1;
    border: 1px solid #555555;
    text-align: center;
}
QProgressBar::chunk {
    background: #202020;
}
QPushButton, QToolButton {
    background: #3a3a3a;
    color: #f1f1f1;
    border: 1px solid #555555;
    border-radius: 3px;
    padding: 4px 9px;
}
QPushButton:hover, QToolButton:hover {
    background: #4a4a4a;
    border-color: #737373;
}
QPushButton:pressed, QToolButton:pressed {
    background: #303030;
}
QPushButton:disabled, QToolButton:disabled {
    background: #292929;
    color: #777777;
    border-color: #3b3b3b;
}
QLineEdit, QPlainTextEdit, QAbstractSpinBox, QComboBox, QTableWidget {
    background: #202020;
    color: #f1f1f1;
    border: 1px solid #555555;
    selection-background-color: #356a9a;
    selection-color: #ffffff;
}
QTableWidget#scanLogTable {
    background: #202020;
    alternate-background-color: #2b2b2b;
    color: #d4d4d4;
    gridline-color: #3a3a3a;
    selection-background-color: #4a4a4a;
    selection-color: #f1f1f1;
}
QTableWidget#scanLogTable::item:selected {
    background: #4a4a4a;
    color: #f1f1f1;
}
QHeaderView::section {
    background: #303030;
    color: #f1f1f1;
    border: 1px solid #555555;
    padding: 4px 6px;
}
QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {
    width: 0px;
    height: 0px;
    border: none;
    background: transparent;
}
QAbstractSpinBox::up-arrow, QAbstractSpinBox::down-arrow {
    width: 0px;
    height: 0px;
}
QComboBox QAbstractItemView {
    background: #242424;
    color: #f1f1f1;
    selection-background-color: #356a9a;
}
QMenu {
    background: #242424;
    color: #f1f1f1;
    border: 1px solid #555555;
    padding: 3px;
}
QMenu::item {
    padding: 5px 28px 5px 8px;
}
QMenu::item:selected {
    background: #356a9a;
    color: #ffffff;
}
QMenu::item:disabled {
    color: #777777;
}
QMenu::separator {
    height: 1px;
    background: #444444;
    margin: 3px 5px;
}
QToolTip {
    background: #242424;
    color: #f1f1f1;
    border: 1px solid #777777;
    padding: 3px;
}
QStatusBar {
    background: #242424;
    color: #f1f1f1;
}
QLabel[themeRole="muted"] {
    color: #aaaaaa;
}
QLabel[themeRole="note"] {
    color: #d6b06a;
}
QCheckBox, QRadioButton {
    color: #f1f1f1;
}
QCheckBox::indicator {
    width: 14px;
    height: 14px;
    background: #202020;
    border: 1px solid #777777;
    border-radius: 2px;
}
QCheckBox::indicator:hover {
    border-color: #bdbdbd;
}
QCheckBox::indicator:checked {
    background: #202020;
    border-color: #d0d0d0;
}
QCheckBox::indicator:disabled {
    background: #292929;
    border-color: #4a4a4a;
}
QTabBar::tab {
    background: #2b2b2b;
    color: #c8c8c8;
    border: 1px solid #444444;
    padding: 5px 10px;
}
QTabBar::tab:selected {
    background: #3e3e3e;
    color: #ffffff;
}
QScrollBar:vertical, QScrollBar:horizontal {
    background: #202020;
}
QScrollBar:vertical {
    width: 16px;
    margin: 16px 0 16px 0;
}
QScrollBar:horizontal {
    height: 16px;
    margin: 0 16px 0 16px;
}
QScrollBar::handle:vertical, QScrollBar::handle:horizontal {
    background: #555555;
    min-height: 24px;
    min-width: 24px;
}
QScrollBar::sub-line:vertical, QScrollBar::add-line:vertical,
QScrollBar::sub-line:horizontal, QScrollBar::add-line:horizontal {
    subcontrol-origin: margin;
}
QScrollBar::sub-line:vertical, QScrollBar::add-line:vertical {
    height: 16px;
}
QScrollBar::sub-line:horizontal, QScrollBar::add-line:horizontal {
    width: 16px;
}
QScrollBar::sub-line:vertical {
    subcontrol-position: top;
}
QScrollBar::add-line:vertical {
    subcontrol-position: bottom;
}
QScrollBar::sub-line:horizontal {
    subcontrol-position: left;
}
QScrollBar::add-line:horizontal {
    subcontrol-position: right;
}
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical,
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {
    background: transparent;
}
"""


def _light_style() -> str:
    """Derive the light theme from the existing dark theme palette."""
    replacements = (
        ("#171717", "#f4f4f4"),
        ("#2b2b2b", "#e8e8e8"),
        ("#242424", "#ffffff"),
        ("#303030", "#dedede"),
        ("#444444", "#c8c8c8"),
        ("#3a3a3a", "#e2e2e2"),
        ("#555555", "#9e9e9e"),
        ("#4a4a4a", "#d0d0d0"),
        ("#626262", "#b8b8b8"),
        ("#292929", "#eeeeee"),
        ("#3b3b3b", "#c0c0c0"),
        ("#737373", "#777777"),
        ("#202020", "#ffffff"),
        ("#f1f1f1", "#202020"),
        ("#aaaaaa", "#5f6368"),
        ("#b8b8b8", "#5f6368"),
        ("#d6b06a", "#8a5a00"),
        ("#d4d4d4", "#303030"),
    )
    style = MAIN_STYLE
    tokens = []
    for index, (old, new) in enumerate(replacements):
        token = f"__THEME_COLOR_{index}__"
        style = style.replace(old, token)
        tokens.append((token, new))
    for token, new in tokens:
        style = style.replace(token, new)
    return style


LIGHT_STYLE = _light_style()
LIGHT_STYLE += """
/* Explicit light-theme overrides for native Qt states and scroll contents. */
QTabBar::tab {
    background: #e8e8e8;
    color: #202020;
    border: 1px solid #b8b8b8;
}
QTabBar::tab:selected {
    background: #d0d0d0;
    color: #101010;
    border-color: #8e8e8e;
}
QTabBar::tab:hover {
    background: #f7f7f7;
    color: #101010;
}
QTabBar::tab:disabled {
    background: #eeeeee;
    color: #9a9a9a;
}
QTabWidget::pane {
    background: #ffffff;
    border: 1px solid #b8b8b8;
}
QProgressBar {
    background: #ffffff;
    color: #202020;
    border: 1px solid #b8b8b8;
}
QProgressBar::chunk {
    background: #ffffff;
}
QWidget#controlPage QTabWidget,
QWidget#controlPage QTabWidget::pane,
QWidget#controlPage QTabWidget QStackedWidget,
QWidget#controlPage QTabWidget QStackedWidget > QWidget {
    background: #ffffff;
}
QScrollArea#loopScroll, QScrollArea#loopScroll > QWidget > QWidget,
QSplitter#loopSplitter > QWidget,
QSplitter#testSplitter > QWidget {
    background: #f4f4f4;
}
QScrollBar:vertical, QScrollBar:horizontal {
    background: #eeeeee;
}
QScrollBar::handle:vertical, QScrollBar::handle:horizontal {
    background: #a0a0a0;
}
"""


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
        self.setStyleSheet(MAIN_STYLE)
        self._settings = QSettings("CanMotorKit", "CanMotorKit")
        self._theme_mode = self._settings.value("theme_mode", "dark")
        self.resize(1500, 920)
        self.bus = None
        self.worker: Worker | None = None
        self.driver = None

        self.conn = ConnectionPanel()
        self.motors = MotorListPanel()
        self.safety = SafetyPanel()
        self.card = MotorCard()
        self.safety.set_output_controls(self.card.output_panel, self.card.service_panel, self.card.rate_panel)
        self.safety.setTitle("③ 安全限制 / 急停")
        left = QWidget()
        left.setObjectName("leftColumn")
        # The scroll area owns the minimum pane width. Let its content follow
        # the viewport instead of retaining the widest child size hint.
        left.setMinimumWidth(0)
        left.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        ll = QVBoxLayout(left)
        ll.setContentsMargins(4, 4, 4, 4)
        ll.addWidget(self.conn)
        ll.addWidget(self.motors)
        ll.addWidget(self.safety)
        ll.addStretch(1)
        scroll = QScrollArea()
        scroll.setObjectName("leftScroll")
        scroll.setWidget(left)
        scroll.setWidgetResizable(True)
        # 左栏保留至少 350 px，避免控件被压缩得难以阅读。
        scroll.setMinimumWidth(350)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.control = ControlPanel()
        self.safety.limits_changed.connect(self.control.set_output_limit)
        self.control.setObjectName("controlPage")
        self.mini_plot = PlotWidget(capacity=60_000, compact=True)
        # The curve pane may be compressed when the data pane is widened.
        self.mini_plot.setMinimumWidth(0)
        self.mini_plot.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        ctl = QSplitter(Qt.Horizontal)
        ctl.setObjectName("controlSplitter")
        cs = QScrollArea()
        cs.setObjectName("controlScroll")
        cs.setWidget(self.control)
        cs.setWidgetResizable(True)
        cs.setMinimumWidth(350)
        cs.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        ctl.addWidget(cs)
        ctl.addWidget(self.mini_plot)
        ctl.setChildrenCollapsible(False)
        ctl.setCollapsible(0, False)
        ctl.setCollapsible(1, True)
        ctl.setHandleWidth(6)
        ctl.setStretchFactor(1, 1)
        ctl.setSizes([350, 700])
        self.test = TestTab()
        self.loop = LoopTab()
        self.regs = RegisterTab()
        self.scan = ScanLogTab()
        self.tabs = QTabWidget()
        self.tabs.setObjectName("mainTabs")
        self.tabs.addTab(ctl, "控制")
        self.tabs.addTab(self.test, "响应测试")
        self.tabs.addTab(self.loop, "闭环测试")
        self.tabs.addTab(self.regs, "电机参数")
        self.tabs.addTab(self.scan, "报文")

        right = QWidget()
        right.setObjectName("rightColumn")
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        topbar = QHBoxLayout()
        topbar.setContentsMargins(0, 0, 4, 0)
        topbar.addWidget(self.card.lbl_title)
        topbar.addStretch(1)
        self.btn_settings = QPushButton()
        self.btn_settings.setText("设置")
        self.btn_settings.setToolTip("界面设置")
        self._theme_menu = QMenu(self.btn_settings)
        self._theme_actions = {}
        theme_group = QActionGroup(self._theme_menu)
        theme_group.setExclusive(True)
        for mode, label in (("dark", "深色模式"), ("light", "浅色模式"),
                            ("system", "跟随系统")):
            action = QAction(label, theme_group)
            action.setCheckable(True)
            action.triggered.connect(lambda _checked=False, value=mode: self._set_theme(value))
            self._theme_menu.addAction(action)
            self._theme_actions[mode] = action
        self.btn_settings.clicked.connect(self._show_theme_menu)
        topbar.addWidget(self.btn_settings)
        rl.addLayout(topbar)
        rl.addWidget(self.tabs, 1)
        # Keep the complete right section usable; its internal curve panes
        # are intentionally allowed to shrink with their splitters.
        right.setMinimumWidth(700)
        right.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        sp = QSplitter(Qt.Horizontal)
        sp.setObjectName("mainSplitter")
        sp.addWidget(scroll)
        sp.addWidget(right)
        # Keep the splitter's hit area at the pane border without showing a
        # second visible bar over the existing pane border.
        sp.setHandleWidth(6)
        sp.setOpaqueResize(True)
        sp.setChildrenCollapsible(False)
        sp.setCollapsible(0, False)
        sp.setCollapsible(1, False)
        sp.setStretchFactor(1, 1)
        sp.setSizes([450, 1050])
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
        self.test.limits_requested.connect(self._raise_test_limits)
        self.loop.limits_requested.connect(self._raise_test_limits)
        self.regs.ids_changed.connect(lambda *_: self.select_motor(None))
        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self.safety.estop,
                  context=Qt.ApplicationShortcut)
        self._space = _SpaceEstop(self.safety.estop)
        QApplication.instance().installEventFilter(self._space)
        QApplication.instance().installEventFilter(self)
        self._set_theme(self._theme_mode, persist=False)
        # 各页面的参数栏与曲线栏使用同一默认比例，并保持拖动比例同步。
        self._pane_splitters = (ctl, self.loop.columns, self.test.columns)
        # Initial ratio after the right pane is laid out at the default window
        # size. Hidden tabs are laid out lazily, so their sizes are applied
        # again when they become visible.
        self._pane_ratio = 350 / 1050
        self._syncing_panes = False
        for splitter in self._pane_splitters:
            splitter.splitterMoved.connect(
                lambda _pos, _index, source=splitter: self._pane_splitter_moved(source)
            )
        self.tabs.currentChanged.connect(
            lambda _index: QTimer.singleShot(0, self._apply_pane_ratio)
        )
        QTimer.singleShot(0, self._apply_pane_ratio)
        if self.conn.pcan_error:
            self.statusBar().showMessage(self.conn.pcan_error + "（仍可使用演示台）", 15000)

    # ---------- 连接 ----------
    @staticmethod
    def _system_is_dark() -> bool:
        return QApplication.palette().color(QPalette.ColorRole.Window).lightness() < 128

    def _show_theme_menu(self, _checked=False):
        width = self._theme_menu.sizeHint().width()
        pos = self.btn_settings.mapToGlobal(
            QPoint(max(0, self.btn_settings.width() - width), self.btn_settings.height())
        )
        self._theme_menu.popup(pos)

    def _pane_splitter_moved(self, source):
        if self._syncing_panes:
            return
        sizes = source.sizes()
        total = sum(sizes)
        if total <= 0:
            return
        self._pane_ratio = max(0.0, min(1.0, sizes[0] / total))
        self._apply_pane_ratio()

    def _apply_pane_ratio(self):
        if getattr(self, "_syncing_panes", False):
            return
        self._syncing_panes = True
        try:
            for splitter in getattr(self, "_pane_splitters", ()):
                total = sum(splitter.sizes())
                if total <= 0:
                    continue
                left = round(total * self._pane_ratio)
                splitter.setSizes([left, total - left])
        finally:
            self._syncing_panes = False

    def _set_theme(self, mode: str, persist: bool = True):
        if mode not in ("dark", "light", "system"):
            mode = "dark"
        self._theme_mode = mode
        if persist:
            self._settings.setValue("theme_mode", mode)
        dark = mode == "dark" or (mode == "system" and self._system_is_dark())
        self.setStyleSheet(MAIN_STYLE if dark else LIGHT_STYLE)
        self._set_native_titlebar(dark)
        for key, action in self._theme_actions.items():
            action.setChecked(key == mode)
        self.motors.apply_theme(dark)
        self.control.apply_theme(dark)
        self.regs.apply_theme(dark)
        for panel in (self.mini_plot, self.loop, self.test, self.scan):
            panel.apply_theme(dark)

    def _set_native_titlebar(self, dark: bool):
        """同步 Windows 原生标题栏，避免浅色内容配深色标题栏。"""
        if sys.platform != "win32":
            return
        try:
            import ctypes

            hwnd = int(self.winId())
            dwm = ctypes.windll.dwmapi.DwmSetWindowAttribute

            def set_attr(attr: int, value: int):
                data = ctypes.c_int(value)
                dwm(hwnd, attr, ctypes.byref(data), ctypes.sizeof(data))

            def colorref(rgb):
                r, g, b = rgb
                return r | (g << 8) | (b << 16)

            set_attr(20, 1 if dark else 0)
            set_attr(35, colorref((23, 23, 23) if dark else (244, 244, 244)))
            set_attr(36, colorref((241, 241, 241) if dark else (32, 32, 32)))
        except (AttributeError, OSError, TypeError, ValueError):
            pass

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.MouseButtonPress:
            target = QApplication.widgetAt(event.globalPosition().toPoint())
            table = getattr(self.motors, "tbl", None)
            if table is not None and (target is None or not table.isAncestorOf(target)):
                self.motors.clear_temporary_selection()
        return super().eventFilter(obj, event)

    def connect_bus(self, p: dict):
        try:
            if p.get("kind") == "socketcan":
                bus = SocketCanBus(p["channel"])
            elif p.get("kind") == "slcan":
                bus = SlcanBus(p["channel"], p["bitrate"])
            elif p["sim"]:
                bus = DemoBus()
            else:
                bus = PcanBus(p["channel"], p["baud"])
            bus.open()
        except Exception as e:
            QMessageBox.critical(self, "连接失败", str(e))
            return
        self.bus = bus
        w = Worker(bus)
        w.samples.connect(self._on_samples)
        w.samples.connect(self.test.on_samples)
        w.frames.connect(self.scan.on_frames)
        w.reg_done.connect(self.regs.on_reg_done)
        w.gd_done.connect(self.regs.on_gd_done)
        w.gd_done.connect(self._on_gd_done)
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
        self._on_message(f"已连接 {p['channel_name']}，开始自动扫描")
        self.motors.scan_bus()

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
        self.safety.set_driver(None)
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
        if w.configuration_busy:
            self._on_message("GD 设置正在执行，请等待操作结束再切换电机")
            return
        if w.active:
            QMessageBox.information(self, "切换电机", "请先失能/停止当前电机")
            return
        self.driver = drv
        w.set_driver(drv)
        for p_ in (self.card, self.control, self.test, self.loop, self.regs, self.scan):
            p_.set_driver(drv)
        self.mini_plot.clear()
        if drv is not None:
            self.mini_plot.set_unit(drv.out_unit, drv.out_name)
            self.motors.set_debug_driver(drv)
        else:
            self.mini_plot.set_unit("Nm", "输出")
            self.motors.set_debug_driver(None)
        self.safety.set_driver(drv)
        lim = self.safety.limits()
        self.control.set_output_limit(lim.out_max if drv is not None and lim.out_enabled else None)
        if drv is None:
            self.tabs.setCurrentIndex(0)
        if drv is not None:
            self.tabs.setCurrentIndex(0)

    # ---------- 回调 ----------
    def _on_gd_done(self, kind, value, error):
        d = self.driver
        if error or d is None or d.protocol != "gd":
            return
        if kind in ("change_id", "model"):
            row = self.motors._debug_row
            if row is not None and 0 <= row < len(self.motors.entries):
                entry = self.motors.entries[row]
                entry.update(model=d.model, motor_id=d.motor_id, fb_id=d.fb_id,
                             candidates=[(d.model, d.motor_id)], gd_identity=dict(d.identity))
                self.motors._refresh()
                self.motors.set_debug_driver(d)
            self.card.lbl_title.setText(d.model)
            self.scan.set_driver(d)

    def _on_samples(self, samples):
        self.mini_plot.add_samples(samples)
        self.control.on_sample(samples[-1])
        self.loop.on_samples(samples)

    def _on_state(self, s: dict):
        for p_ in (self.card, self.control, self.safety, self.motors, self.test, self.loop, self.regs):
            p_.on_state(s)
        run = self.worker._run_obj if self.worker else None
        target = ""
        if run is not None:
            cfg = getattr(run, "cfg", None)
            target = getattr(cfg, "loop", "") if cfg is not None and hasattr(cfg, "loop") else ""
        self.mini_plot.set_ref_target(target)

    def _on_limits(self, lim, ok: bool):
        self.control.set_limits(lim, ok)
        safety_lim = self.safety.limits()
        self.control.set_output_limit(safety_lim.out_max
                                      if self.driver is not None and safety_lim.out_enabled else None)
        self.regs.set_limits(lim, ok)

    def _on_message(self, msg: str):
        self.statusBar().showMessage(msg, 8000)

    def _raise_test_limits(self, vel_max: float, out_max: float):
        if self.safety.raise_for_test(vel_max, out_max):
            self.statusBar().showMessage("已按测试参数提高左侧速度/输出限幅", 3000)

    def _use_suggestion(self, s):
        self.loop.set_suggestion(s)
        self.tabs.setCurrentWidget(self.loop)
