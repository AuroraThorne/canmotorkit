"""MIT 控制面板：使能/失能/急停/清错/设零、周期下发开关、五参数（手动应用）、实时读数。

约定：
- 参数默认全 0；修改后必须点“应用”（或回车）才写入下发目标，没有自动更新。
- 使能/失能按钮不锁定，电机是否使能以状态帧状态码为准。
- “周期下发”独立开关：打开才发 MIT 帧。
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
                               QLabel, QPushButton, QSlider, QSpinBox, QToolButton, QVBoxLayout)

from hm import protocol as P

PENDING_STYLE = "QDoubleSpinBox{background:#fff3c4}"


class _Row:
    """一个参数：滑条 + 数值框 + 置零按钮。只修改界面值，不下发。"""
    STEPS = 2000

    def __init__(self, grid: QGridLayout, row: int, label: str, unit: str, decimals: int,
                 step: float, on_edit, on_enter):
        self.lo, self.hi = -1.0, 1.0
        self.applied = 0.0
        self.on_edit = on_edit
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, self.STEPS)
        self.slider.setFocusPolicy(Qt.NoFocus)
        self.spin = QDoubleSpinBox()
        self.spin.setDecimals(decimals)
        self.spin.setSingleStep(step)
        self.spin.setKeyboardTracking(True)
        self.spin.setMinimumWidth(110)
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
        self.spin.lineEdit().returnPressed.connect(on_enter)
        self.btn0.clicked.connect(lambda: self.set_value(0.0))

    def set_range(self, lo: float, hi: float):
        v = self.value()
        self.lo, self.hi = lo, hi
        self.spin.blockSignals(True)
        self.spin.setRange(lo, hi)
        self.spin.blockSignals(False)
        self.range_lbl.setText(f"[{lo:.4g}, {hi:.4g}]")
        self.set_value(v)

    def value(self) -> float:
        return self.spin.value()

    def set_value(self, v: float):
        v = P.clamp(v, self.lo, self.hi)
        self.spin.blockSignals(True)
        self.slider.blockSignals(True)
        self.spin.setValue(v)
        self.slider.setValue(self._to_slider(v))
        self.spin.blockSignals(False)
        self.slider.blockSignals(False)
        self._mark()

    def mark_applied(self):
        self.applied = self.value()
        self._mark()

    def pending(self) -> bool:
        return abs(self.value() - self.applied) > 1e-12

    def _mark(self):
        self.spin.setStyleSheet(PENDING_STYLE if self.pending() else "")
        self.on_edit()

    def _to_slider(self, v: float) -> int:
        span = self.hi - self.lo
        return int(round((v - self.lo) / span * self.STEPS)) if span > 0 else 0

    def _on_slider(self, i: int):
        v = self.lo + (self.hi - self.lo) * i / self.STEPS
        self.spin.blockSignals(True)
        self.spin.setValue(v)
        self.spin.blockSignals(False)
        self._mark()

    def _on_spin(self, v: float):
        self.slider.blockSignals(True)
        self.slider.setValue(self._to_slider(v))
        self.slider.blockSignals(False)
        self._mark()


class ControlPanel(QGroupBox):
    rate_changed = Signal(float, float)     # 周期下发频率, 状态轮询频率

    def __init__(self, parent=None):
        super().__init__("MIT 控制", parent)
        self.worker = None
        self._streaming = False

        def btn(text, style="", h=28):
            b = QPushButton(text)
            b.setFocusPolicy(Qt.NoFocus)      # 防止空格键误触发
            b.setMinimumHeight(h)
            if style:
                b.setStyleSheet(style)
            return b

        self.btn_enable = btn("使能 (FC)")
        self.btn_disable = btn("失能 (FD)")
        self.btn_stream = btn("周期下发：关")
        self.btn_stream.setCheckable(True)
        self.btn_stream.setStyleSheet("QPushButton:checked{background:#2e7d32;color:white;font-weight:bold}")
        self.btn_stream.setToolTip("打开后按频率发送 MIT 控制帧（发送的是已应用的参数）；\n"
                                   "关闭时用 0xCC 查询状态。失能/急停会自动关闭。")
        self.btn_estop = btn("急停  [Esc / 空格]", "QPushButton{background:#c62828;color:white;"
                                                   "font-weight:bold;font-size:15px}", h=40)
        self.btn_clear = btn("清错 (FB)")
        self.btn_zero = btn("设零点 (FE)")
        self.btn_zero.setToolTip("将当前输出轴位置设为零点（需失能且关闭周期下发）")
        self.lbl_motor = QLabel("电机状态：—")
        self.lbl_motor.setAlignment(Qt.AlignCenter)
        self.lbl_motor.setMinimumHeight(26)

        top = QGridLayout()
        top.addWidget(self.lbl_motor, 0, 0, 1, 2)
        top.addWidget(self.btn_enable, 1, 0)
        top.addWidget(self.btn_disable, 1, 1)
        top.addWidget(self.btn_stream, 2, 0, 1, 2)
        top.addWidget(self.btn_clear, 3, 0)
        top.addWidget(self.btn_zero, 3, 1)
        top.addWidget(self.btn_estop, 4, 0, 1, 2)

        # 参数
        pg = QGroupBox("目标参数（修改后点“应用”或按回车生效）")
        grid = QGridLayout(pg)
        grid.setColumnStretch(1, 1)
        args = (self._edited, self.apply)
        self.rows = {
            "p": _Row(grid, 0, "位置 p", "rad", 4, 0.01, *args),
            "v": _Row(grid, 1, "速度 v", "rad/s", 3, 0.1, *args),
            "kp": _Row(grid, 2, "Kp", "", 2, 1.0, *args),
            "kd": _Row(grid, 3, "Kd", "", 3, 0.05, *args),
            "t": _Row(grid, 4, "前馈扭矩 t", "Nm", 3, 0.1, *args),
        }
        self.btn_apply = btn("应用", "QPushButton:enabled{font-weight:bold}")
        self.btn_revert = btn("撤销修改")
        self.btn_all0 = btn("全部置 0")
        self.btn_all0.setToolTip("只修改界面值，仍需应用")
        self.lbl_applied = QLabel()
        self.lbl_applied.setStyleSheet("color:gray;font-family:Consolas,monospace")
        self.lbl_applied.setWordWrap(True)
        r = QHBoxLayout()
        r.addWidget(self.btn_apply)
        r.addWidget(self.btn_revert)
        r.addStretch(1)
        r.addWidget(self.btn_all0)
        grid.addLayout(r, 5, 0, 1, 5)
        grid.addWidget(self.lbl_applied, 6, 0, 1, 5)

        # 通信 / 量程
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
        self.sp_poll.setToolTip("周期下发关闭时用 0xCC 查询状态的频率，0 = 不查询")
        self.lbl_rate = QLabel("—")
        self.lbl_lim = QLabel("—")
        self.lbl_lim.setWordWrap(True)
        ff.addRow("周期下发频率", self.sp_rate)
        ff.addRow("状态轮询（不下发时）", self.sp_poll)
        ff.addRow("实际发送频率", self.lbl_rate)
        ff.addRow("当前量程", self.lbl_lim)

        # 读数
        rg = QGroupBox("实时反馈")
        rf = QFormLayout(rg)
        mono = "font-family:Consolas,monospace;font-size:13px"
        self.lbl = {}
        for k, name in (("pos", "位置"), ("vel", "速度"), ("tq", "扭矩"), ("tmp", "温度 d6/d7"),
                        ("err", "状态码"), ("raw", "原始字节")):
            lb = QLabel("—")
            lb.setStyleSheet(mono)
            lb.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.lbl[k] = lb
            rf.addRow(name, lb)

        lay = QVBoxLayout(self)
        lay.addLayout(top)
        lay.addWidget(pg)
        lay.addWidget(fg)
        lay.addWidget(rg)
        lay.addStretch(1)

        self.set_limits(P.DEFAULT_LIMITS, False)
        self.btn_enable.clicked.connect(lambda: self.worker and self.worker.enable())
        self.btn_disable.clicked.connect(lambda: self.worker and self.worker.disable())
        self.btn_stream.clicked.connect(self._stream_clicked)
        self.btn_estop.clicked.connect(self.estop)
        self.btn_clear.clicked.connect(lambda: self.worker and self.worker.clear_error())
        self.btn_zero.clicked.connect(lambda: self.worker and self.worker.set_zero())
        self.btn_apply.clicked.connect(self.apply)
        self.btn_revert.clicked.connect(self._revert)
        self.btn_all0.clicked.connect(lambda: [r.set_value(0.0) for r in self.rows.values()])
        self.sp_rate.valueChanged.connect(self._rates)
        self.sp_poll.valueChanged.connect(self._rates)
        self._show_applied()
        self.set_bound(None)

    # ---------- 外部接口 ----------
    def set_bound(self, worker):
        self.worker = worker
        on = worker is not None
        for w in (self.btn_enable, self.btn_disable, self.btn_stream, self.btn_estop, self.btn_clear,
                  self.btn_zero):
            w.setEnabled(on)
        self._set_stream_btn(False)
        if on:
            self._rates()
            # 连接后将已应用的参数同步给 worker
            worker.set_target(self.applied_command())
        else:
            for lb in self.lbl.values():
                lb.setText("—")
            self.lbl_motor.setText("电机状态：—")
            self.lbl_motor.setStyleSheet("")

    def set_limits(self, lim: P.Limits, from_motor: bool):
        self.rows["p"].set_range(lim.p_min, lim.p_max)
        self.rows["v"].set_range(lim.v_min, lim.v_max)
        self.rows["kp"].set_range(lim.kp_min, lim.kp_max)
        self.rows["kd"].set_range(lim.kd_min, lim.kd_max)
        self.rows["t"].set_range(lim.t_min, lim.t_max)
        src = "来自电机" if from_motor else "未读取（默认值，禁止使能）"
        self.lbl_lim.setText(f"p ±{lim.p_max:.4g}  v ±{lim.v_max:.4g}  t ±{lim.t_max:.4g}\n"
                             f"kp 0~{lim.kp_max:g}  kd 0~{lim.kd_max:g}（{src}）")
        self.lbl_lim.setStyleSheet("" if from_motor else "color:#c62828")

    def command(self) -> P.MitCommand:
        r = self.rows
        return P.MitCommand(r["p"].value(), r["v"].value(), r["kp"].value(), r["kd"].value(),
                            r["t"].value())

    def applied_command(self) -> P.MitCommand:
        r = self.rows
        return P.MitCommand(r["p"].applied, r["v"].applied, r["kp"].applied, r["kd"].applied,
                            r["t"].applied)

    def set_and_apply(self, cmd: P.MitCommand):
        """由测试页等显式操作调用：写入界面并应用。"""
        for k in ("p", "v", "kp", "kd", "t"):
            self.rows[k].set_value(getattr(cmd, k))
        self.apply()

    def on_status(self, st: P.Status):
        self.lbl["pos"].setText(f"{st.pos:+9.4f} rad")
        self.lbl["vel"].setText(f"{st.vel:+9.3f} rad/s")
        self.lbl["tq"].setText(f"{st.torque:+9.3f} Nm")
        self.lbl["tmp"].setText(f"{st.t_mos} / {st.t_rotor} ℃")
        fault = P.is_fault(st.err)
        self.lbl["err"].setText(f"{st.err}  {P.code_text(st.err)}" + ("  请清错" if fault else ""))
        self.lbl["err"].setStyleSheet("color:#c62828;font-weight:bold" if fault else "")
        self.lbl["raw"].setText(P.hexdata(st.raw))

    def on_state(self, s: dict):
        if self.worker is None:
            return
        code = s["code"]
        if code < 0:
            txt, style = "电机状态：无反馈", "background:#9e9e9e;color:white"
        elif code == P.CODE_ENABLED:
            txt, style = "电机状态：已使能", "background:#2e7d32;color:white"
        elif code == P.CODE_DISABLED:
            txt, style = "电机状态：失能", "background:#e0e0e0"
        else:
            txt, style = f"电机状态：{P.code_text(code)}", "background:#c62828;color:white"
        if s["enabled"] and not s["streaming"]:
            txt += "（未周期下发）"
        self.lbl_motor.setText(txt)
        self.lbl_motor.setStyleSheet(style + ";font-weight:bold;border-radius:3px")
        self._set_stream_btn(s["streaming"])
        self.btn_enable.setEnabled(not s.get("scanning") and s["limits_ok"])
        self.btn_stream.setEnabled(s["streaming"] or (s["limits_ok"] and not s.get("scanning")))
        self.btn_zero.setEnabled(not s["blocked"])
        self.lbl_rate.setText(f"{s['tx_rate']:.0f} Hz" if s["streaming"] else "—（未下发）")

    def estop(self):
        if self.worker:
            self.worker.estop()

    def apply(self):
        for r in self.rows.values():
            r.mark_applied()
        if self.worker:
            self.worker.set_target(self.applied_command())
        self._show_applied()

    # ---------- 内部 ----------
    def _set_stream_btn(self, on: bool):
        self._streaming = on
        self.btn_stream.blockSignals(True)
        self.btn_stream.setChecked(on)
        self.btn_stream.blockSignals(False)
        self.btn_stream.setText("周期下发：开（点击关闭）" if on else "周期下发：关（点击打开）")

    def _stream_clicked(self):
        want = not self._streaming
        self._set_stream_btn(self._streaming)      # 以 worker 反馈为准
        if self.worker:
            self.worker.set_streaming(want)

    def _revert(self):
        for r in self.rows.values():
            r.set_value(r.applied)

    def _edited(self):
        if not hasattr(self, "btn_apply"):
            return
        pend = any(r.pending() for r in self.rows.values()) if hasattr(self, "rows") and len(self.rows) == 5 else False
        self.btn_apply.setEnabled(pend)
        self.btn_revert.setEnabled(pend)

    def _show_applied(self):
        c = self.applied_command()
        self.lbl_applied.setText(f"已应用: p={c.p:.4f} v={c.v:.3f} kp={c.kp:g} kd={c.kd:g} t={c.t:.3f}")
        self._edited()

    def _rates(self):
        if self.worker:
            self.worker.rate_hz = float(self.sp_rate.value())
            self.worker.poll_hz = float(self.sp_poll.value())
        self.rate_changed.emit(float(self.sp_rate.value()), float(self.sp_poll.value()))
