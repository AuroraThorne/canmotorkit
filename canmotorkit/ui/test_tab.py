"""响应测试页：只发电流/扭矩激励，得到电流→速度、电流→位置的幅频/相频曲线，并给出模型与 PID 建议。"""
from __future__ import annotations

import csv
import math

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
                               QHBoxLayout, QLabel, QMessageBox, QPlainTextEdit, QPushButton,
                               QSpinBox, QSplitter, QVBoxLayout, QWidget)

from canmotorkit.analysis import sysid
from canmotorkit.analysis.runs import EXCITE_KINDS, ExciteConfig, ExciteRun


def dspin(lo, hi, val, dec=3, step=0.1, suffix=""):
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(dec)
    s.setSingleStep(step)
    s.setValue(val)
    if suffix:
        s.setSuffix(suffix)
    return s


class TestTab(QWidget):
    suggestion_ready = Signal(object)        # sysid.Suggestion

    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self.driver = None
        self.run: ExciteRun | None = None
        self.model: sysid.Model | None = None
        self.sugg: sysid.Suggestion | None = None

        self.cb_kind = QComboBox()
        for k, v in EXCITE_KINDS.items():
            self.cb_kind.addItem(v, k)
        self.cb_kind.setCurrentIndex(1)
        self.sp_amp = dspin(0, 100, 0.5, 3, 0.1)
        self.sp_off = dspin(-100, 100, 0.0, 3, 0.1)
        self.sp_on = dspin(0.01, 10, 0.3, 3, 0.05, " s")
        self.sp_f0 = dspin(0.05, 200, 1.0, 2, 0.5, " Hz")
        self.sp_f1 = dspin(0.1, 300, 30.0, 2, 1.0, " Hz")
        self.sp_n = QSpinBox()
        self.sp_n.setRange(2, 40)
        self.sp_n.setValue(10)
        self.sp_cyc = QSpinBox()
        self.sp_cyc.setRange(2, 50)
        self.sp_cyc.setValue(6)
        self.sp_dur = dspin(0.5, 120, 5.0, 1, 1.0, " s")
        self.sp_bias = dspin(0, 100, 0.0, 2, 0.5, " rad/s")
        self.sp_bias.setToolTip("大于 0 时叠加一个弱速度环，让电机保持平均转速以越过静摩擦。\n"
                                "辨识使用实际下发的电流与实测速度，偏置环不影响结果。")
        self.sp_bkp = dspin(0, 100, 0.05, 4, 0.01)
        self.sp_bkp.setToolTip("偏置速度环增益（输出单位 / (rad/s)），应远小于正常速度环")
        self.lbl_est = QLabel()
        self.lbl_est.setWordWrap(True)
        self.lbl_est.setStyleSheet("color:#795548")

        g = QGroupBox("激励（只发送电流/扭矩）")
        f = QFormLayout(g)
        self.lbl_amp = QLabel("幅值")
        f.addRow("类型", self.cb_kind)
        f.addRow(self.lbl_amp, self.sp_amp)
        f.addRow("偏置输出", self.sp_off)
        f.addRow("阶跃持续时间", self.sp_on)
        f.addRow("起始频率", self.sp_f0)
        f.addRow("终止频率", self.sp_f1)
        f.addRow("频点数", self.sp_n)
        f.addRow("每频点周期", self.sp_cyc)
        f.addRow("时长（扫频/PRBS）", self.sp_dur)
        f.addRow("偏置速度", self.sp_bias)
        f.addRow("偏置环增益", self.sp_bkp)
        f.addRow(self.lbl_est)
        self.btn_start = QPushButton("开始测试")
        self.btn_start.setStyleSheet("QPushButton{font-weight:bold;min-height:28px}")
        self.btn_stop = QPushButton("停止")
        self.btn_csv = QPushButton("导出数据")
        self.btn_use = QPushButton("把建议参数带入闭环测试")
        self.btn_use.setEnabled(False)
        b = QHBoxLayout()
        b.addWidget(self.btn_start)
        b.addWidget(self.btn_stop)
        b.addWidget(self.btn_csv)
        note = QLabel("前提：电机已使能、没有安全锁存。全程受左侧「安全限制」保护，超限即零电流并中止。\n"
                      "电流→位置在低频位移很大，建议开启位置窗口。")
        note.setWordWrap(True)
        note.setStyleSheet("color:gray")
        left = QVBoxLayout()
        left.addWidget(g)
        left.addLayout(b)
        left.addWidget(self.btn_use)
        left.addWidget(note)
        left.addStretch(1)
        lw = QWidget()
        lw.setLayout(left)
        lw.setMaximumWidth(380)

        self.glw = pg.GraphicsLayoutWidget()
        self.glw.setBackground("w")
        self.p_mag = self.glw.addPlot(row=0, col=0)
        self.p_ph = self.glw.addPlot(row=1, col=0)
        for p in (self.p_mag, self.p_ph):
            p.setLogMode(x=True, y=False)
            p.showGrid(x=True, y=True, alpha=0.3)
            p.addLegend(offset=(5, 5))
        self.p_ph.setXLink(self.p_mag)
        self.p_mag.setLabel("left", "幅值", units="dB")
        self.p_ph.setLabel("left", "相位", units="°")
        self.p_ph.setLabel("bottom", "频率", units="Hz")
        sym = dict(symbol="o", symbolSize=6)
        self.c = {
            "mv": self.p_mag.plot(name="电流→速度", pen=pg.mkPen("#1565c0", width=2), symbolBrush="#1565c0", **sym),
            "mp": self.p_mag.plot(name="电流→位置", pen=pg.mkPen("#e65100", width=2), symbolBrush="#e65100", **sym),
            "mm": self.p_mag.plot(name="拟合模型(速度)", pen=pg.mkPen("#9e9e9e", width=1.5, style=Qt.DashLine)),
            "pv": self.p_ph.plot(name="电流→速度", pen=pg.mkPen("#1565c0", width=2), symbolBrush="#1565c0", **sym),
            "pp": self.p_ph.plot(name="电流→位置", pen=pg.mkPen("#e65100", width=2), symbolBrush="#e65100", **sym),
            "pm": self.p_ph.plot(name="拟合模型(速度)", pen=pg.mkPen("#9e9e9e", width=1.5, style=Qt.DashLine)),
        }
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setMaximumBlockCount(1000)
        sp = QSplitter(Qt.Vertical)
        sp.addWidget(self.glw)
        sp.addWidget(self.out)
        sp.setSizes([500, 200])
        lay = QHBoxLayout(self)
        lay.addWidget(lw)
        lay.addWidget(sp, 1)

        self.cb_kind.currentIndexChanged.connect(self._update)
        for w in (self.sp_amp, self.sp_f0, self.sp_f1, self.sp_n, self.sp_cyc, self.sp_dur, self.sp_on):
            w.valueChanged.connect(self._update)
        self.btn_start.clicked.connect(self.start)
        self.btn_stop.clicked.connect(lambda: self.worker and self.worker.stop_run())
        self.btn_csv.clicked.connect(self.export)
        self.btn_use.clicked.connect(lambda: self.sugg and self.suggestion_ready.emit(self.sugg))
        self._update()
        self.set_bound(None)

    def set_bound(self, worker):
        self.worker = worker
        self.set_driver(None)

    def set_driver(self, d):
        self.driver = d
        on = self.worker is not None and d is not None
        self.btn_start.setEnabled(on)
        self.btn_stop.setEnabled(False)
        u = d.out_unit if d else ""
        for s in (self.sp_amp, self.sp_off):
            s.setSuffix(f" {u}" if u else "")
        self.sp_bkp.setSuffix(f" {u}/(rad/s)" if u else "")
        if d is not None:
            self.sp_amp.setMaximum(d.out_max)
            self.sp_off.setRange(-d.out_max, d.out_max)

    def on_state(self, s: dict):
        if self.driver is None:
            return
        running = s["running"]
        self.btn_start.setEnabled(not running and s["active"] and not s["tripped"])
        self.btn_stop.setEnabled(running)

    def _update(self):
        k = self.cb_kind.currentData()
        self.sp_on.setEnabled(k == "step")
        for w in (self.sp_f0, self.sp_f1):
            w.setEnabled(k != "step")
        self.sp_n.setEnabled(k == "stepsine")
        self.sp_cyc.setEnabled(k == "stepsine")
        self.sp_dur.setEnabled(k in ("chirp", "prbs"))
        try:
            run = ExciteRun(self._cfg())
            self.lbl_est.setText(f"预计时长 {run.total:.1f} s")
        except Exception:
            self.lbl_est.setText("")

    def _cfg(self) -> ExciteConfig:
        return ExciteConfig(kind=self.cb_kind.currentData(), amp=self.sp_amp.value(),
                            offset=self.sp_off.value(), step_on=self.sp_on.value(),
                            f0=self.sp_f0.value(), f1=max(self.sp_f1.value(), self.sp_f0.value() * 1.01),
                            n_freq=self.sp_n.value(), cycles=self.sp_cyc.value(),
                            duration=self.sp_dur.value(), bias_vel=self.sp_bias.value(),
                            bias_kp=self.sp_bkp.value())

    def start(self):
        w = self.worker
        if not w or not self.driver:
            return
        if not w.active:
            QMessageBox.information(self, "响应测试", "请先使能电机")
            return
        cfg = self._cfg()
        lim = w.safety.limits.out_max
        if lim > 0 and abs(cfg.offset) + cfg.amp > lim:
            if QMessageBox.question(self, "响应测试", f"偏置+幅值超过安全限幅 ±{lim:g}，"
                                    "激励会被削顶，影响辨识结果。继续？") != QMessageBox.Yes:
                return
        self.run = ExciteRun(cfg)
        for c in self.c.values():
            c.setData([], [])
        w.start_run(self.run)
        self.out.appendPlainText(f"▶ {self.run.title}，预计 {self.run.total:.1f} s")

    def on_run_finished(self, run):
        if run is not self.run:
            return
        n = len(run.rec)
        self.out.appendPlainText(f"■ 结束（{run.reason or '完成'}），记录 {n} 点")
        if n < 10:
            return
        unit = self.driver.out_unit if self.driver else ""
        lat = self.worker.latency_stats().get("p50", 0.002) if self.worker else 0.002
        rate = self.worker.rate_hz if self.worker else 500
        if run.cfg.kind == "step":
            m = sysid.step_model(run.rec, run.cfg.amp, run.cfg.step_delay, run.cfg.step_on, unit)
            if m is None:
                self.out.appendPlainText("  阶跃辨识失败：电机可能未转动（幅值低于静摩擦？）")
                return
            self._show_model(m, None, lat, rate)
            return
        bode = sysid.bode_from_run(run)
        if bode is None:
            self.out.appendPlainText("  频响计算失败：有效数据不足（电机是否卡在静摩擦？可加偏置速度）")
            return
        f = bode.f
        db = lambda x: 20 * np.log10(np.maximum(x, 1e-12))
        self.c["mv"].setData(f, db(bode.mag_v))
        self.c["mp"].setData(f, db(bode.mag_p))
        self.c["pv"].setData(f, bode.ph_v)
        self.c["pp"].setData(f, bode.ph_p)
        bad = int(np.sum(bode.coh < 0.6))
        if bad:
            self.out.appendPlainText(f"  注意：{bad} 个频点拟合质量低（噪声/摩擦/饱和）")
        m = sysid.fit_model(bode, unit)
        if m is not None:
            w = 2 * np.pi * f
            g = np.exp(-1j * w * m.Td) / (m.J * 1j * w + m.b)
            self.c["mm"].setData(f, db(np.abs(g)))
            self.c["pm"].setData(f, np.degrees(np.unwrap(np.angle(g))))
            self._show_model(m, bode, lat, rate)

    def _show_model(self, m, bode, lat, rate):
        self.model = m
        self.sugg = sysid.suggest(m, latency=lat, rate_hz=rate)
        self.out.appendPlainText("  模型：" + m.text())
        self.out.appendPlainText("  PID 建议：\n    " + self.sugg.text().replace("\n", "\n    "))
        self.btn_use.setEnabled(True)

    def export(self):
        if self.run is None or not len(self.run.rec):
            QMessageBox.information(self, "导出", "没有数据")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出响应测试", "response.csv", "CSV (*.csv)")
        if not path:
            return
        r = self.run.rec
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["t", "u", "excite", "pos", "vel", "out_fb", "seg"])
            for row in zip(r.t, r.u, r.ref, r.pos, r.vel, r.fb_out, r.seg):
                w.writerow(["" if isinstance(x, float) and math.isnan(x) else f"{x:.7g}" for x in row])
        self.out.appendPlainText(f"已导出 {path}")
