"""测试信号页：阶跃/方波/正弦/扫频，自动录制并给出阶跃指标，导出 CSV。"""
from __future__ import annotations

import csv

from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
                               QHBoxLayout, QLabel, QMessageBox, QPlainTextEdit, QPushButton,
                               QVBoxLayout, QWidget)

from hm.siggen import KINDS, SignalGen, step_metrics

CHANNELS = {"p": "位置 p (rad)", "v": "速度 v (rad/s)", "t": "扭矩 t (Nm)"}
REC_COLS = ["t", "p_cmd", "v_cmd", "t_cmd", "kp", "kd", "p_fb", "v_fb", "t_fb", "t_mos", "t_rotor", "err"]


def _spin(lo, hi, val, dec=3, step=0.1, suffix=""):
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(dec)
    s.setSingleStep(step)
    s.setValue(val)
    if suffix:
        s.setSuffix(suffix)
    return s


class TestTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self.control = None             # ControlPanel，用于读取当前 kp/kd 与位置
        self.recording = False
        self.rec: list = []
        self.channel = "p"

        self.cb_kind = QComboBox()
        for k, v in KINDS.items():
            self.cb_kind.addItem(v, k)
        self.cb_kind.setCurrentIndex(1)
        self.cb_ch = QComboBox()
        for k, v in CHANNELS.items():
            self.cb_ch.addItem(v, k)
        self.sp_amp = _spin(-100, 100, 0.5)
        self.sp_off = _spin(-100, 100, 0.0)
        self.chk_rel = QComboBox()
        self.chk_rel.addItems(["偏置 = 当前位置/目标 + 偏置", "偏置 = 绝对值"])
        self.sp_freq = _spin(0.01, 100, 1.0, 2, 0.5, " Hz")
        self.sp_freq2 = _spin(0.01, 200, 10.0, 2, 1, " Hz")
        self.sp_dur = _spin(0.1, 600, 3.0, 2, 0.5, " s")
        self.sp_delay = _spin(0, 60, 0.5, 2, 0.1, " s")
        self.sp_kp = _spin(0, 500, 20.0, 2, 1)
        self.sp_kd = _spin(0, 5, 1.0, 3, 0.05)

        g = QGroupBox("信号")
        f = QFormLayout(g)
        f.addRow("类型", self.cb_kind)
        f.addRow("作用通道", self.cb_ch)
        f.addRow("幅值", self.sp_amp)
        f.addRow("偏置", self.sp_off)
        f.addRow("偏置模式", self.chk_rel)
        f.addRow("频率 / 起始频率", self.sp_freq)
        f.addRow("扫频终止频率", self.sp_freq2)
        f.addRow("持续时间", self.sp_dur)
        f.addRow("阶跃延时", self.sp_delay)
        g2 = QGroupBox("增益（开始时写入控制面板）")
        f2 = QFormLayout(g2)
        f2.addRow("Kp", self.sp_kp)
        f2.addRow("Kd", self.sp_kd)
        self.lbl_hint = QLabel()
        self.lbl_hint.setWordWrap(True)
        self.lbl_hint.setStyleSheet("color:#795548")

        self.btn_start = QPushButton("开始")
        self.btn_start.setStyleSheet("QPushButton{font-weight:bold;min-height:28px}")
        self.btn_stop = QPushButton("停止")
        self.btn_csv = QPushButton("导出录制 CSV")
        bar = QHBoxLayout()
        bar.addWidget(self.btn_start)
        bar.addWidget(self.btn_stop)
        bar.addWidget(self.btn_csv)
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setMaximumBlockCount(500)

        left = QVBoxLayout()
        left.addWidget(g)
        left.addWidget(g2)
        left.addWidget(self.lbl_hint)
        left.addLayout(bar)
        left.addStretch(1)
        lay = QHBoxLayout(self)
        lay.addLayout(left, 0)
        lay.addWidget(self.out, 1)

        self.cb_kind.currentIndexChanged.connect(self._update_ui)
        self.cb_ch.currentIndexChanged.connect(self._update_ui)
        self.btn_start.clicked.connect(self.start)
        self.btn_stop.clicked.connect(self.stop)
        self.btn_csv.clicked.connect(self.export)
        self._update_ui()
        self.set_bound(None)

    def set_bound(self, worker):
        self.worker = worker
        self.btn_start.setEnabled(worker is not None)
        self.btn_stop.setEnabled(False)

    def _update_ui(self):
        k = self.cb_kind.currentData()
        ch = self.cb_ch.currentData()
        self.sp_freq.setEnabled(k in ("sine", "square", "chirp"))
        self.sp_freq2.setEnabled(k == "chirp")
        self.sp_delay.setEnabled(k == "step")
        hints = {
            "p": "位置测试：需 kp>0、kd>0（kd=0 会振荡）。建议从小幅值、小 kp 开始。",
            "v": "速度测试：建议 kp=0、kd>0，由 kd 决定速度环刚度。",
            "t": "扭矩测试：kp=kd=0 时为纯扭矩输出，空载会持续加速，请注意安全！",
        }
        self.lbl_hint.setText(hints[ch])
        defaults = {"p": (20.0, 1.0, 0.5), "v": (0.0, 1.0, 2.0), "t": (0.0, 0.0, 0.5)}
        kp, kd, amp = defaults[ch]
        if ch != self.channel:
            self.channel = ch
            self.sp_kp.setValue(kp)
            self.sp_kd.setValue(kd)
            self.sp_amp.setValue(amp)

    def start(self):
        w = self.worker
        if not w:
            return
        if not w.enabled:
            QMessageBox.information(self, "测试", "请先在控制面板使能电机")
            return
        ch = self.cb_ch.currentData()
        if ch == "p" and self.sp_kd.value() == 0:
            QMessageBox.warning(self, "测试", "位置控制时 kd 不能为 0（手册要求）")
            return
        cur = w.get_target()
        base = 0.0
        if self.chk_rel.currentIndex() == 0:
            if ch == "p":
                base = w.last_status.pos if w.last_status else cur.p
            else:
                base = getattr(cur, ch)
        gen = SignalGen(self.cb_kind.currentData(), self.sp_amp.value(), base + self.sp_off.value(),
                        self.sp_freq.value(), self.sp_freq2.value(), self.sp_dur.value(),
                        self.sp_delay.value())
        # 设置增益与初始值，并同步控制面板
        cur.kp, cur.kd = self.sp_kp.value(), self.sp_kd.value()
        if ch == "p":
            cur.p = gen.value(0.0)
        if self.control:
            self.control.on_target_changed(cur)
        w.set_target(cur)
        self.rec = []
        self.recording = True
        w.start_signal(gen, ch)
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.out.appendPlainText(f"▶ {KINDS[gen.kind]} 通道={ch} 幅值={gen.amp:g} 偏置={gen.offset:.4f} "
                                 f"kp={cur.kp:g} kd={cur.kd:g} 时长={gen.duration:g}s")

    def stop(self):
        if self.worker:
            self.worker.stop_signal()

    def on_samples(self, samples):
        if self.recording:
            self.rec.extend(s for s in samples if s.enabled)

    def on_finished(self):
        if not self.recording:
            return
        self.recording = False
        self.btn_start.setEnabled(self.worker is not None)
        self.btn_stop.setEnabled(False)
        n = len(self.rec)
        self.out.appendPlainText(f"■ 结束，录制 {n} 点")
        if n < 2:
            return
        ch = self.channel
        fbk = {"p": lambda s: s.st.pos, "v": lambda s: s.st.vel, "t": lambda s: s.st.torque}[ch]
        t = [s.t for s in self.rec]
        cmd = [getattr(s.cmd, ch) for s in self.rec]
        fb = [fbk(s) for s in self.rec]
        err = [c - y for c, y in zip(cmd, fb)]
        rms = (sum(e * e for e in err) / n) ** 0.5
        self.out.appendPlainText(f"  跟踪误差 RMS = {rms:.5f}，最大 |e| = {max(map(abs, err)):.5f}")
        if self.cb_kind.currentData() == "step":
            m = step_metrics(t, cmd, fb)
            self.out.appendPlainText("  " + (m.text() if m else "未检测到阶跃"))

    def export(self):
        if not self.rec:
            QMessageBox.information(self, "导出", "没有录制数据")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出录制", "test.csv", "CSV (*.csv)")
        if not path:
            return
        t0 = self.rec[0].t
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(REC_COLS)
            for s in self.rec:
                c, st = s.cmd, s.st
                w.writerow([f"{s.t - t0:.6f}", f"{c.p:.6g}", f"{c.v:.6g}", f"{c.t:.6g}", f"{c.kp:.6g}",
                            f"{c.kd:.6g}", f"{st.pos:.6g}", f"{st.vel:.6g}", f"{st.torque:.6g}",
                            st.t_mos, st.t_rotor, st.err])
        self.out.appendPlainText(f"已导出 {path}")
