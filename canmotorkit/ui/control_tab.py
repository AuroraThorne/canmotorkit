"""控制页：电流/扭矩（主页）+ MIT（子页，仅 MIT 电机）+ 实时反馈。

约定：参数默认全 0；修改后黄色标记，必须点“应用”或回车才下发；单位按厂家协议。
"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
                               QLabel, QPushButton, QSlider, QTabWidget, QToolButton, QVBoxLayout,
                               QWidget)

from canmotorkit.drivers import dm_protocol as P

PENDING_STYLE = "QDoubleSpinBox{background:#d6a936;color:#171717}"


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
        self.range_lbl.setStyleSheet("color:#aaaaaa")
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
        self.range_lbl.setText(f"[{lo:.7g}, {hi:.7g}]")
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


class _Params(QWidget):
    """一组参数行 + 应用/撤销/全部置0。"""

    def __init__(self, specs, on_apply):
        super().__init__()
        self.on_apply = on_apply
        self.rows = {}
        grid = QGridLayout(self)
        grid.setColumnStretch(1, 1)
        for i, (k, label, unit, dec, step) in enumerate(specs):
            self.rows[k] = _Row(grid, i, label, unit, dec, step, self._edited, self.apply)
        n = len(specs)
        self.btn_apply = QPushButton("应用（回车）")
        self.btn_apply.setStyleSheet("QPushButton:enabled{font-weight:bold;background:#d6a936;color:#171717}")
        self.btn_revert = QPushButton("撤销修改")
        self.btn_all0 = QPushButton("全部置 0")
        self.btn_all0.setToolTip("只修改界面值，仍需应用")
        for b in (self.btn_apply, self.btn_revert, self.btn_all0):
            b.setFocusPolicy(Qt.NoFocus)
        r = QHBoxLayout()
        r.addWidget(self.btn_apply)
        r.addWidget(self.btn_revert)
        r.addStretch(1)
        r.addWidget(self.btn_all0)
        grid.addLayout(r, n, 0, 1, 5)
        self.lbl_applied = QLabel()
        self.lbl_applied.setStyleSheet("color:#aaaaaa;font-family:Consolas,monospace")
        grid.addWidget(self.lbl_applied, n + 1, 0, 1, 5)
        self.btn_apply.clicked.connect(self.apply)
        self.btn_revert.clicked.connect(lambda: [x.set_value(x.applied) for x in self.rows.values()])
        self.btn_all0.clicked.connect(lambda: [x.set_value(0.0) for x in self.rows.values()])
        self._show()

    def applied(self) -> dict:
        return {k: r.applied for k, r in self.rows.items()}

    def apply(self):
        for r in self.rows.values():
            r.mark_applied()
        self._show()
        self.on_apply(self.applied())

    def reset_zero(self):
        for r in self.rows.values():
            r.set_value(0.0)
            r.mark_applied()
        self._show()

    def _edited(self):
        if not hasattr(self, "btn_apply"):
            return
        pend = any(r.pending() for r in self.rows.values())
        self.btn_apply.setEnabled(pend)
        self.btn_revert.setEnabled(pend)

    def _show(self):
        a = self.applied()
        self.lbl_applied.setText("已应用：" + "  ".join(f"{k}={v:.6g}" for k, v in a.items()))
        self._edited()


class ControlPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self.driver = None
        self._safety_out_max = None

        # 电流/扭矩主页
        self.cur = _Params([("out", "输出", "", 3, 0.05)], self._apply_cur)
        self.lbl_cur_hint = QLabel()
        self.lbl_cur_hint.setWordWrap(True)
        self.lbl_cur_hint.setStyleSheet("color:#d6b06a")
        pc = QWidget()
        lc = QVBoxLayout(pc)
        lc.addWidget(self.cur)
        lc.addWidget(self.lbl_cur_hint)
        lc.addStretch(1)

        # MIT 子页
        self.mit = _Params([("p", "位置 p", "rad", 4, 0.01), ("v", "速度 v", "rad/s", 3, 0.1),
                            ("kp", "Kp", "", 2, 1.0), ("kd", "Kd", "", 3, 0.05),
                            ("t", "前馈扭矩 t", "Nm", 3, 0.1)], self._apply_mit)
        self.lbl_lim = QLabel("—")
        self.lbl_lim.setWordWrap(True)
        pm = QWidget()
        lm = QVBoxLayout(pm)
        lm.addWidget(self.mit)
        f = QFormLayout()
        f.addRow("当前量程", self.lbl_lim)
        lm.addLayout(f)
        tip = QLabel("MIT 模式下，前馈扭矩 t 同样受左侧「电流/扭矩限幅」限制；位置/速度环由驱动器完成。")
        tip.setWordWrap(True)
        tip.setStyleSheet("color:#aaaaaa")
        lm.addWidget(tip)
        lm.addStretch(1)

        self.tabs = QTabWidget()
        self.tabs.addTab(pc, "电流 / 扭矩")
        self.tabs.addTab(pm, "MIT")
        self.tabs.currentChanged.connect(self._tab_changed)

        rg = QGroupBox("实时反馈")
        rf = QFormLayout(rg)
        mono = "font-family:Consolas,monospace;font-size:13px"
        self.lbl = {}
        for k, name in (("pos", "位置（原始 / 累计）"), ("vel", "速度"), ("out", "输出（指令 / 反馈）"),
                        ("tmp", "温度"), ("code", "状态码"), ("raw", "原始字节")):
            lb = QLabel("—")
            lb.setStyleSheet(mono)
            lb.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.lbl[k] = lb
            rf.addRow(name, lb)

        lay = QVBoxLayout(self)
        lay.addWidget(self.tabs)
        lay.addWidget(rg)
        lay.addStretch(1)
        self.set_bound(None)

    # ---------- 外部 ----------
    def set_bound(self, worker):
        self.worker = worker
        self.set_driver(None)

    def set_driver(self, d):
        self.driver = d
        self._safety_out_max = None
        self.cur.reset_zero()
        self.mit.reset_zero()
        has_mit = d is not None and d.caps.mit
        self.tabs.setTabEnabled(1, has_mit)
        self.tabs.setTabToolTip(1, "" if has_mit else "当前电机不支持 MIT 协议")
        if not has_mit:
            self.tabs.setCurrentIndex(0)
        if d is None:
            self.lbl_cur_hint.setText("")
            for lb in self.lbl.values():
                lb.setText("—")
            return
        r = self.cur.rows["out"]
        r.spin.setSuffix(f" {d.out_unit}")
        self._update_output_range()
        if d.caps.mit:
            self.lbl_cur_hint.setText("达妙/HM 的电流模式即 MIT kp=kd=p=v=0、t=输出（Nm）。"
                                      "空载时超过静摩擦会持续加速。")
        else:
            self.lbl_cur_hint.setText(f"大疆电调电流指令，单位 A（厂家比例 ±{d.m.raw_max} ↔ ±{d.out_max:g} A）。")
        self._tab_changed(self.tabs.currentIndex())

    def set_limits(self, lim: P.Limits, from_motor: bool):
        r = self.mit.rows
        r["p"].set_range(lim.p_min, lim.p_max)
        r["v"].set_range(lim.v_min, lim.v_max)
        r["kp"].set_range(lim.kp_min, lim.kp_max)
        r["kd"].set_range(lim.kd_min, lim.kd_max)
        r["t"].set_range(lim.t_min, lim.t_max)
        if self.driver is not None and self.driver.caps.mit:
            self._update_output_range()
        src = "来自电机" if from_motor else "未读取（默认值，禁止使能）"
        self.lbl_lim.setText(f"p ±{lim.p_max:.7g}  v ±{lim.v_max:.7g}  t ±{lim.t_max:.7g}\n"
                             f"kp 0~{lim.kp_max:g}  kd 0~{lim.kd_max:g}（{src}）")
        self.lbl_lim.setStyleSheet("" if from_motor else "color:#c62828")

    def set_output_limit(self, limit: float | None):
        """同步左侧安全限幅，控制滑条只能在当前允许范围内调整。"""
        value = None if limit is None or limit <= 0 else float(limit)
        if self._safety_out_max == value:
            return
        self._safety_out_max = value
        self._update_output_range()

    def _update_output_range(self):
        if self.driver is None:
            return
        protocol_limit = max(0.0, float(self.driver.out_max))
        limit = protocol_limit if self._safety_out_max is None else min(protocol_limit, self._safety_out_max)
        self.cur.rows["out"].set_range(-limit, limit)

    def on_sample(self, s):
        d = self.driver
        if d is None:
            return
        fb = s.fb
        self.lbl["pos"].setText(f"{fb.pos:+9.4f} / {s.pos_mt:+10.4f} rad")
        self.lbl["vel"].setText(f"{fb.vel:+9.3f} rad/s")
        cmd = "—" if s.out != s.out else f"{s.out:+.3f}"
        self.lbl["out"].setText(f"{cmd} / {fb.out:+.3f} {d.out_unit}")
        t2 = "" if fb.temp2 != fb.temp2 else f" / {fb.temp2:.0f}"
        self.lbl["tmp"].setText("—" if fb.temp != fb.temp else f"{fb.temp:.0f}{t2} ℃")
        if d.caps.status_code:
            fault = d.is_fault(fb.code)
            self.lbl["code"].setText(f"{fb.code}  {d.code_text(fb.code)}")
            self.lbl["code"].setStyleSheet("color:#c62828;font-weight:bold" if fault else "")
        else:
            self.lbl["code"].setText("（协议无状态码）")
        self.lbl["raw"].setText(P.hexdata(fb.raw))

    def to_current(self):
        self.tabs.setCurrentIndex(0)

    # ---------- 内部 ----------
    def _apply_cur(self, a: dict):
        if self.worker:
            self.worker.set_target_out(a["out"])

    def _apply_mit(self, a: dict):
        if self.worker:
            self.worker.set_target_mit(P.MitCommand(a["p"], a["v"], a["kp"], a["kd"], a["t"]))

    def _tab_changed(self, i: int):
        if self.worker is not None:
            self.worker.set_mode("mit" if i == 1 else "current")
