"""MIT 控制面板：使能/失能/急停/清错/设零、五参数滑条、快捷模式、发送频率、实时读数。"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QCheckBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QPushButton, QSlider, QSpinBox, QToolButton,
                               QVBoxLayout, QWidget)

from hm import protocol as P


class _Row:
    """一个参数：滑条 + 数值框 + 置零按钮，放在外部 QGridLayout 的一行里。"""
    STEPS = 2000

    def __init__(self, grid: QGridLayout, row: int, label: str, unit: str, decimals: int,
                 step: float, on_change):
        self.lo, self.hi = -1.0, 1.0
        self.on_change = on_change
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, self.STEPS)
        self.slider.setFocusPolicy(Qt.NoFocus)
        self.spin = QDoubleSpinBox()
        self.spin.setDecimals(decimals)
        self.spin.setSingleStep(step)
        self.spin.setKeyboardTracking(False)
        self.spin.setMinimumWidth(95)
        self.spin.setSuffix(f" {unit}" if unit else "")
        self.btn0 = QToolButton()
        self.btn0.setText("0")
        self.btn0.setFocusPolicy(Qt.NoFocus)
        self.range_lbl = QLabel()
        self.range_lbl.setStyleSheet("color:gray")
        grid.addWidget(QLabel(label), row, 0)
        grid.addWidget(self.slider, row, 1)
        grid.addWidget(self.spin, row, 2)
        grid.addWidget(self.btn0, row, 3)
        grid.addWidget(self.range_lbl, row, 4)
        self.slider.valueChanged.connect(self._on_slider)
        self.spin.valueChanged.connect(self._on_spin)
        self.btn0.clicked.connect(lambda: self.set_value(0.0, emit=True))

    def set_range(self, lo: float, hi: float):
        v = self.value()
        self.lo, self.hi = lo, hi
        self.spin.blockSignals(True)
        self.spin.setRange(lo, hi)
        self.spin.blockSignals(False)
        self.range_lbl.setText(f"[{lo:g}, {hi:g}]")
        self.set_value(v)

    def value(self) -> float:
        return self.spin.value()

    def set_value(self, v: float, emit: bool = False):
        v = P.clamp(v, self.lo, self.hi)
        self.spin.blockSignals(True)
        self.slider.blockSignals(True)
        self.spin.setValue(v)
        self.slider.setValue(self._to_slider(v))
        self.spin.blockSignals(False)
        self.slider.blockSignals(False)
        if emit:
            self.on_change()

    def _to_slider(self, v: float) -> int:
        span = self.hi - self.lo
        return int(round((v - self.lo) / span * self.STEPS)) if span > 0 else 0

    def _on_slider(self, i: int):
        v = self.lo + (self.hi - self.lo) * i / self.STEPS
        self.spin.blockSignals(True)
        self.spin.setValue(v)
        self.spin.blockSignals(False)
        self.on_change()

    def _on_spin(self, v: float):
        self.slider.blockSignals(True)
        self.slider.setValue(self._to_slider(v))
        self.slider.blockSignals(False)
        self.on_change()


class ControlPanel(QGroupBox):
    rate_changed = Signal(float, float)     # 使能发送频率, 失能轮询频率

    def __init__(self, parent=None):
        super().__init__("MIT 控制", parent)
        self.worker = None
        self._fb_pos = 0.0

        def btn(text, style="", h=28):
            b = QPushButton(text)
            b.setFocusPolicy(Qt.NoFocus)      # 防止空格键误触发
            b.setMinimumHeight(h)
            if style:
                b.setStyleSheet(style)
            return b

        self.btn_enable = btn("使能 (FC)", "QPushButton{background:#2e7d32;color:white;font-weight:bold}"
                                            "QPushButton:disabled{background:#9e9e9e}")
        self.btn_disable = btn("失能 (FD)")
        self.btn_estop = btn("急停  [Esc / 空格]", "QPushButton{background:#c62828;color:white;"
                                                   "font-weight:bold;font-size:15px}", h=40)
        self.btn_clear = btn("清错 (FB)")
        self.btn_zero = btn("设零点 (FE)")
        self.btn_zero.setToolTip("将当前输出轴位置设为零点（需失能）")

        top = QGridLayout()
        top.addWidget(self.btn_enable, 0, 0)
        top.addWidget(self.btn_disable, 0, 1)
        top.addWidget(self.btn_clear, 1, 0)
        top.addWidget(self.btn_zero, 1, 1)
        top.addWidget(self.btn_estop, 2, 0, 1, 2)

        # 参数
        pg = QGroupBox("目标参数")
        grid = QGridLayout(pg)
        grid.setColumnStretch(1, 1)
        self.rows = {
            "p": _Row(grid, 0, "位置 p", "rad", 4, 0.01, self._changed),
            "v": _Row(grid, 1, "速度 v", "rad/s", 3, 0.1, self._changed),
            "kp": _Row(grid, 2, "Kp", "", 2, 1.0, self._changed),
            "kd": _Row(grid, 3, "Kd", "", 3, 0.05, self._changed),
            "t": _Row(grid, 4, "前馈扭矩 t", "Nm", 3, 0.1, self._changed),
        }
        self.chk_live = QCheckBox("修改即下发")
        self.chk_live.setChecked(True)
        self.btn_apply = btn("应用")
        self.btn_apply.setEnabled(False)
        self.btn_p_here = btn("p = 当前位置")
        r = QHBoxLayout()
        r.addWidget(self.chk_live)
        r.addWidget(self.btn_apply)
        r.addStretch(1)
        r.addWidget(self.btn_p_here)
        grid.addLayout(r, 5, 0, 1, 5)

        # 快捷模式
        mg = QGroupBox("快捷模式")
        mh = QHBoxLayout(mg)
        self.btn_m_damp = btn("阻尼")
        self.btn_m_damp.setToolTip("kp=0 kd=0.5 v=0 t=0")
        self.btn_m_vel = btn("速度")
        self.btn_m_vel.setToolTip("kp=0 kd=1，由 v 控制转速")
        self.btn_m_pos = btn("位置")
        self.btn_m_pos.setToolTip("p=当前位置 kp=20 kd=1 v=0 t=0")
        self.btn_m_tq = btn("扭矩")
        self.btn_m_tq.setToolTip("kp=0 kd=0 v=0 t=0，由 t 控制扭矩")
        for b in (self.btn_m_damp, self.btn_m_vel, self.btn_m_pos, self.btn_m_tq):
            mh.addWidget(b)

        # 频率
        fg = QGroupBox("通信")
        ff = QFormLayout(fg)
        self.sp_rate = QSpinBox()
        self.sp_rate.setRange(50, 1000)
        self.sp_rate.setValue(200)
        self.sp_rate.setSuffix(" Hz")
        self.sp_poll = QSpinBox()
        self.sp_poll.setRange(0, 200)
        self.sp_poll.setValue(50)
        self.sp_poll.setSuffix(" Hz")
        self.sp_poll.setToolTip("失能时用 0xCC 查询状态的频率，0 = 不查询")
        self.lbl_rate = QLabel("—")
        ff.addRow("使能时 MIT 发送频率", self.sp_rate)
        ff.addRow("失能时状态轮询", self.sp_poll)
        ff.addRow("实际发送频率", self.lbl_rate)

        # 读数
        rg = QGroupBox("实时反馈")
        rf = QFormLayout(rg)
        mono = "font-family:Consolas,monospace;font-size:13px"
        self.lbl = {}
        for k, name in (("pos", "位置"), ("vel", "速度"), ("tq", "扭矩"), ("tmp", "温度 d6/d7"),
                        ("err", "错误码"), ("raw", "原始字节")):
            l = QLabel("—")
            l.setStyleSheet(mono)
            l.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.lbl[k] = l
            rf.addRow(name, l)

        lay = QVBoxLayout(self)
        lay.addLayout(top)
        lay.addWidget(pg)
        lay.addWidget(mg)
        lay.addWidget(fg)
        lay.addWidget(rg)
        lay.addStretch(1)

        self.set_limits(P.MODEL_LIMITS["G9"])
        self.btn_enable.clicked.connect(self._enable)
        self.btn_disable.clicked.connect(lambda: self.worker and self.worker.disable())
        self.btn_estop.clicked.connect(self.estop)
        self.btn_clear.clicked.connect(lambda: self.worker and self.worker.clear_error())
        self.btn_zero.clicked.connect(lambda: self.worker and self.worker.set_zero())
        self.btn_apply.clicked.connect(self._push)
        self.chk_live.toggled.connect(lambda on: self.btn_apply.setEnabled(not on))
        self.btn_p_here.clicked.connect(lambda: self._set(p=self._fb_pos))
        self.btn_m_damp.clicked.connect(lambda: self._set(p=self._fb_pos, v=0, kp=0, kd=0.5, t=0))
        self.btn_m_vel.clicked.connect(lambda: self._set(kp=0, kd=1.0, t=0))
        self.btn_m_pos.clicked.connect(lambda: self._set(p=self._fb_pos, v=0, kp=20, kd=1.0, t=0))
        self.btn_m_tq.clicked.connect(lambda: self._set(p=0, v=0, kp=0, kd=0, t=0))
        self.sp_rate.valueChanged.connect(self._rates)
        self.sp_poll.valueChanged.connect(self._rates)
        self.set_bound(None)

    # ---------- 外部接口 ----------
    def set_bound(self, worker):
        self.worker = worker
        on = worker is not None
        for w in (self.btn_enable, self.btn_disable, self.btn_estop, self.btn_clear, self.btn_zero):
            w.setEnabled(on)
        if on:
            self._rates()
        else:
            for l in self.lbl.values():
                l.setText("—")

    def set_limits(self, lim: P.Limits):
        self.rows["p"].set_range(lim.p_min, lim.p_max)
        self.rows["v"].set_range(lim.v_min, lim.v_max)
        self.rows["kp"].set_range(lim.kp_min, lim.kp_max)
        self.rows["kd"].set_range(lim.kd_min, lim.kd_max)
        self.rows["t"].set_range(lim.t_min, lim.t_max)

    def command(self) -> P.MitCommand:
        r = self.rows
        return P.MitCommand(r["p"].value(), r["v"].value(), r["kp"].value(), r["kd"].value(),
                            r["t"].value())

    def on_target_changed(self, cmd: P.MitCommand):
        """worker 改变了目标（使能初始化/测试信号结束）时同步到界面，不回写。"""
        for k in ("p", "v", "kp", "kd", "t"):
            self.rows[k].set_value(getattr(cmd, k))

    def on_status(self, st: P.Status):
        self._fb_pos = st.pos
        self.lbl["pos"].setText(f"{st.pos:+9.4f} rad")
        self.lbl["vel"].setText(f"{st.vel:+9.3f} rad/s")
        self.lbl["tq"].setText(f"{st.torque:+9.3f} Nm")
        self.lbl["tmp"].setText(f"{st.t_mos} / {st.t_rotor} ℃")
        self.lbl["err"].setText(f"{st.err}" + ("  ⚠ 请清错" if st.err else ""))
        self.lbl["err"].setStyleSheet("color:#c62828;font-weight:bold" if st.err else "")
        self.lbl["raw"].setText(P.hexdata(st.raw))

    def on_state(self, s: dict):
        en = s["enabled"]
        if self.worker is None:
            return
        self.btn_enable.setEnabled(not en and not s.get("scanning"))
        self.btn_zero.setEnabled(not en)
        self.btn_enable.setText("已使能" if en else "使能 (FC)")
        self.lbl_rate.setText(f"{s['tx_rate']:.0f} Hz" if en else "—（未使能）")

    def estop(self):
        if self.worker:
            self.worker.estop()

    # ---------- 内部 ----------
    def _enable(self):
        if self.worker:
            self.worker.enable()

    def _set(self, **kw):
        for k, v in kw.items():
            self.rows[k].set_value(v)
        if self.chk_live.isChecked():
            self._push()

    def _changed(self):
        if self.chk_live.isChecked():
            self._push()

    def _push(self):
        if self.worker:
            self.worker.set_target(self.command())

    def _rates(self):
        if self.worker:
            self.worker.rate_hz = float(self.sp_rate.value())
            self.worker.poll_hz = float(self.sp_poll.value())
        self.rate_changed.emit(float(self.sp_rate.value()), float(self.sp_poll.value()))
