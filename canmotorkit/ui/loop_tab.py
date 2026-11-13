"""闭环测试页：上位机单环（速度 PI / 位置 PID），输出为电流/扭矩。仅用于测试与辨识。"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QComboBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel,
                               QMessageBox, QPlainTextEdit, QPushButton, QSplitter, QVBoxLayout,
                               QWidget)

from canmotorkit.analysis import sysid
from canmotorkit.analysis.runs import LOOP_KINDS, REF_KINDS, LoopConfig, LoopRun

from .test_tab import dspin
from .widgets import CheckBox


class LoopTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self.driver = None
        self.run: LoopRun | None = None
        self.sugg = None

        self.cb_loop = QComboBox()
        for k, v in LOOP_KINDS.items():
            self.cb_loop.addItem(v, k)
        self.sp_kp = dspin(0, 1e4, 0.05, 5, 0.01)
        self.sp_ki = dspin(0, 1e5, 0.0, 5, 0.01)
        self.sp_kd = dspin(0, 1e4, 0.0, 5, 0.001)
        self.cb_ref = QComboBox()
        for k, v in REF_KINDS.items():
            self.cb_ref.addItem(v, k)
        self.sp_amp = dspin(-1000, 1000, 2.0, 3, 0.5)
        self.sp_freq = dspin(0.05, 50, 1.0, 2, 0.5, " Hz")
        self.sp_ramp = dspin(0.05, 30, 1.0, 2, 0.1, " s")
        self.sp_dur = dspin(0.2, 60, 2.0, 2, 0.5, " s")
        self.sp_omax = dspin(0, 1000, 1.0, 3, 0.1)
        self.sp_omax.setToolTip("控制器输出限幅；另受左侧安全限幅约束，取二者较小值")
        self.chk_rel = CheckBox("位置参考相对起始位置")
        self.chk_rel.setChecked(True)
        self.lbl_sugg = QLabel("尚无响应测试建议")
        self.lbl_sugg.setWordWrap(True)
        self.lbl_sugg.setStyleSheet("color:#b8b8b8")
        self.btn_fill = QPushButton("填入建议值 × 50%")
        self.btn_fill.setEnabled(False)

        g = QGroupBox("控制器（单环，输出即电流/扭矩）")
        f = QFormLayout(g)
        f.addRow("回路", self.cb_loop)
        f.addRow("Kp", self.sp_kp)
        f.addRow("Ki", self.sp_ki)
        self.lbl_kd = QLabel("Kd")
        f.addRow(self.lbl_kd, self.sp_kd)
        f.addRow("输出限幅 ±", self.sp_omax)
        g2 = QGroupBox("参考信号")
        f2 = QFormLayout(g2)
        f2.addRow("类型", self.cb_ref)
        self.lbl_amp = QLabel("幅值")
        f2.addRow(self.lbl_amp, self.sp_amp)
        f2.addRow("正弦频率", self.sp_freq)
        f2.addRow("斜坡时间", self.sp_ramp)
        f2.addRow("时长", self.sp_dur)
        f2.addRow(self.chk_rel)
        g3 = QGroupBox("来自响应测试的建议")
        l3 = QVBoxLayout(g3)
        l3.addWidget(self.lbl_sugg)
        l3.addWidget(self.btn_fill)
        self.btn_start = QPushButton("开始闭环")
        self.btn_start.setStyleSheet("QPushButton{font-weight:bold;min-height:28px}")
        self.btn_stop = QPushButton("停止")
        b = QHBoxLayout()
        b.addWidget(self.btn_start)
        b.addWidget(self.btn_stop)
        note = QLabel("上位机闭环经 USB-CAN，延迟约数毫秒，只用于测试与辨识，不作为产品控制器。"
                      "速度/位置/电流超左侧安全限制时立即零电流并中止。建议发送频率 ≥ 500 Hz。")
        note.setWordWrap(True)
        note.setStyleSheet("color:#aaaaaa")
        left = QVBoxLayout()
        for w in (g, g2, g3):
            left.addWidget(w)
        left.addLayout(b)
        left.addWidget(note)
        left.addStretch(1)
        lw = QWidget()
        lw.setLayout(left)
        lw.setMaximumWidth(380)

        self.glw = pg.GraphicsLayoutWidget()
        self.glw.setBackground("w")
        self.p_y = self.glw.addPlot(row=0, col=0)
        self.p_u = self.glw.addPlot(row=1, col=0)
        self.p_u.setXLink(self.p_y)
        for p in (self.p_y, self.p_u):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.addLegend(offset=(5, 5))
        self.p_u.setLabel("bottom", "时间", units="s")
        self.c_ref = self.p_y.plot(name="参考", pen=pg.mkPen("#9e9e9e", width=1.5, style=Qt.DashLine))
        self.c_y = self.p_y.plot(name="反馈", pen=pg.mkPen("#1565c0", width=2))
        self.c_u = self.p_u.plot(name="输出", pen=pg.mkPen("#e65100", width=1.5))
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        sp = QSplitter(Qt.Vertical)
        sp.addWidget(self.glw)
        sp.addWidget(self.out)
        sp.setSizes([500, 160])
        lay = QHBoxLayout(self)
        lay.addWidget(lw)
        lay.addWidget(sp, 1)

        self.cb_loop.currentIndexChanged.connect(self._update)
        self.cb_ref.currentIndexChanged.connect(self._update)
        self.btn_start.clicked.connect(self.start)
        self.btn_stop.clicked.connect(lambda: self.worker and self.worker.stop_run())
        self.btn_fill.clicked.connect(self._fill)
        self._update()
        self.set_bound(None)

    def set_bound(self, worker):
        self.worker = worker
        self.set_driver(None)

    def set_driver(self, d):
        self.driver = d
        self.btn_start.setEnabled(self.worker is not None and d is not None)
        self.btn_stop.setEnabled(False)
        self._update()

    def on_state(self, s: dict):
        if self.driver is None:
            return
        self.btn_start.setEnabled(not s["running"] and s["active"] and not s["tripped"])
        self.btn_stop.setEnabled(s["running"])

    def _units(self):
        u = self.driver.out_unit if self.driver else ""
        if self.cb_loop.currentData() == "vel":
            return u, "rad/s", f"{u}/(rad/s)", f"{u}/rad", ""
        return u, "rad", f"{u}/rad", f"{u}/(rad·s)", f"{u}/(rad/s)"

    def _update(self):
        u, yu, kpu, kiu, kdu = self._units()
        is_pos = self.cb_loop.currentData() == "pos"
        self.sp_kp.setSuffix(f" {kpu}")
        self.sp_ki.setSuffix(f" {kiu}")
        self.sp_kd.setSuffix(f" {kdu}")
        self.sp_kd.setEnabled(is_pos)
        self.lbl_kd.setText("Kd（作用于测量速度）" if is_pos else "Kd（速度环不用）")
        self.sp_amp.setSuffix(f" {yu}")
        self.sp_omax.setSuffix(f" {u}")
        self.chk_rel.setEnabled(is_pos)
        k = self.cb_ref.currentData()
        self.sp_freq.setEnabled(k == "sine")
        self.sp_ramp.setEnabled(k == "ramp")
        self.p_y.setLabel("left", "速度" if not is_pos else "位置", units=yu)
        self.p_u.setLabel("left", "输出", units=u)
        self._show_sugg()

    def set_suggestion(self, s: sysid.Suggestion):
        self.sugg = s
        self.btn_fill.setEnabled(True)
        self._show_sugg()

    def _show_sugg(self):
        s = self.sugg
        if s is None:
            return
        self.lbl_sugg.setText(s.text().split("\n依据")[0])

    def _fill(self):
        s = self.sugg
        if s is None:
            return
        if self.cb_loop.currentData() == "vel":
            self.sp_kp.setValue(s.vel_kp * 0.5)
            self.sp_ki.setValue(s.vel_ki * 0.5)
            self.sp_kd.setValue(0.0)
        else:
            self.sp_kp.setValue(s.pos_kp * 0.5)
            self.sp_ki.setValue(s.pos_ki * 0.5)
            self.sp_kd.setValue(s.pos_kd * 0.5)

    def start(self):
        w = self.worker
        if not w or not self.driver:
            return
        if not w.active:
            QMessageBox.information(self, "闭环测试", "请先使能电机")
            return
        lim = w.safety.limits
        loop = self.cb_loop.currentData()
        amp = self.sp_amp.value()
        if loop == "vel" and lim.vel_max > 0 and abs(amp) >= lim.vel_max:
            QMessageBox.warning(self, "闭环测试", f"参考速度 {amp:g} 不小于安全速度上限 {lim.vel_max:g}")
            return
        if loop == "pos" and not lim.pos_enabled:
            if QMessageBox.question(self, "闭环测试", "位置闭环建议先在左侧启用位置窗口。仍要继续？") \
                    != QMessageBox.Yes:
                return
        cfg = LoopConfig(loop=loop, kp=self.sp_kp.value(), ki=self.sp_ki.value(),
                         kd=self.sp_kd.value() if loop == "pos" else 0.0,
                         ref_kind=self.cb_ref.currentData(), amp=amp, freq=self.sp_freq.value(),
                         ramp_time=self.sp_ramp.value(), duration=self.sp_dur.value(),
                         out_max=self.sp_omax.value(), relative=self.chk_rel.isChecked())
        self.run = LoopRun(cfg)
        for c in (self.c_ref, self.c_y, self.c_u):
            c.setData([], [])
        w.start_run(self.run)
        self.out.appendPlainText("▶ " + self.run.title)

    def on_run_finished(self, run):
        if run is not self.run:
            return
        r = run.rec
        self.out.appendPlainText(f"■ 结束（{run.reason or '完成'}），记录 {len(r)} 点，输出饱和 {run.sat_ratio * 100:.0f}%")
        if len(r) < 5:
            return
        t = np.asarray(r.t)
        y = np.asarray(r.vel if run.cfg.loop == "vel" else r.pos)
        self.c_ref.setData(t, np.asarray(r.ref), connect="finite")
        self.c_y.setData(t, y)
        self.c_u.setData(t, np.asarray(r.u), connect="finite")
        st = sysid.loop_stats(r, run.cfg.loop)
        if st:
            self.out.appendPlainText("  " + st.text())
