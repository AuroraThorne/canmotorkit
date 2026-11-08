"""实时曲线：位置/速度/扭矩/温度，环形缓冲 + 暂停 + CSV 导出。"""
from __future__ import annotations

import csv
import math

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QCheckBox, QDoubleSpinBox, QFileDialog, QHBoxLayout, QLabel,
                               QMessageBox, QPushButton, QVBoxLayout, QWidget)

COLS = ["t", "p_cmd", "p_fb", "v_cmd", "v_fb", "t_cmd", "t_fb", "t_mos", "t_rotor", "err", "enabled"]


class Ring:
    def __init__(self, n: int, cols: int):
        self.buf = np.full((n, cols), np.nan)
        self.n, self.i, self.count = n, 0, 0

    def extend(self, rows: np.ndarray):
        for r in rows if len(rows) <= self.n else rows[-self.n:]:
            self.buf[self.i] = r
            self.i = (self.i + 1) % self.n
            self.count = min(self.count + 1, self.n)

    def data(self) -> np.ndarray:
        if self.count < self.n:
            return self.buf[:self.count]
        return np.vstack((self.buf[self.i:], self.buf[:self.i]))

    def clear(self):
        self.buf[:] = np.nan
        self.i = self.count = 0


class PlotWidget(QWidget):
    def __init__(self, parent=None, capacity: int = 120_000):
        super().__init__(parent)
        self.ring = Ring(capacity, len(COLS))
        self.t0: float | None = None
        self._dirty = False

        self.sp_win = QDoubleSpinBox()
        self.sp_win.setRange(1, 300)
        self.sp_win.setValue(10)
        self.sp_win.setSuffix(" s")
        self.chk_pause = QCheckBox("暂停显示")
        self.chk_cmd = QCheckBox("显示指令")
        self.chk_cmd.setChecked(True)
        self.btn_clear = QPushButton("清空")
        self.btn_csv = QPushButton("导出 CSV")
        self.lbl_info = QLabel()
        self.lbl_info.setStyleSheet("color:gray")
        bar = QHBoxLayout()
        bar.addWidget(QLabel("时间窗口"))
        bar.addWidget(self.sp_win)
        bar.addWidget(self.chk_pause)
        bar.addWidget(self.chk_cmd)
        bar.addWidget(self.btn_clear)
        bar.addWidget(self.btn_csv)
        bar.addStretch(1)
        bar.addWidget(self.lbl_info)

        self.glw = pg.GraphicsLayoutWidget()
        self.glw.setBackground("w")
        specs = [("位置", "rad", "p_cmd", "p_fb"), ("速度", "rad/s", "v_cmd", "v_fb"),
                 ("扭矩", "Nm", "t_cmd", "t_fb"), ("温度", "℃", "t_mos", "t_rotor")]
        self.curves: dict[str, pg.PlotDataItem] = {}
        self.plots = []
        first = None
        for i, (title, unit, a, b) in enumerate(specs):
            p = self.glw.addPlot(row=i, col=0)
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("left", title, units=unit)
            p.addLegend(offset=(5, 5))
            p.setDownsampling(auto=True, mode="peak")
            p.setClipToView(True)
            if first is None:
                first = p
            else:
                p.setXLink(first)
            if i == 3:
                p.setLabel("bottom", "时间", units="s")
                self.curves[a] = p.plot(name="d6 (t_mos)", pen=pg.mkPen("#e65100", width=1.5))
                self.curves[b] = p.plot(name="d7 (t_rotor)", pen=pg.mkPen("#6a1b9a", width=1.5))
            else:
                self.curves[a] = p.plot(name="指令", pen=pg.mkPen("#9e9e9e", width=1.5,
                                                                   style=Qt.DashLine))
                self.curves[b] = p.plot(name="反馈", pen=pg.mkPen("#1565c0", width=1.5))
            self.plots.append(p)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addLayout(bar)
        lay.addWidget(self.glw, 1)

        self.btn_clear.clicked.connect(self.clear)
        self.btn_csv.clicked.connect(self.export_csv)
        self.chk_cmd.toggled.connect(lambda _: self._mark())
        self.sp_win.valueChanged.connect(lambda _: self._mark())
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(33)

    def _mark(self):
        self._dirty = True

    def add_samples(self, samples):
        if not samples:
            return
        if self.t0 is None:
            self.t0 = samples[0].t
        nan = math.nan
        rows = np.array([
            (s.t - self.t0,
             s.cmd.p if s.enabled else nan, s.st.pos,
             s.cmd.v if s.enabled else nan, s.st.vel,
             s.cmd.t if s.enabled else nan, s.st.torque,
             s.st.t_mos, s.st.t_rotor, s.st.err, 1.0 if s.enabled else 0.0)
            for s in samples], dtype=float)
        self.ring.extend(rows)
        self._dirty = True

    def clear(self):
        self.ring.clear()
        self.t0 = None
        for c in self.curves.values():
            c.setData([], [])

    def _refresh(self):
        if not self._dirty or self.chk_pause.isChecked() or self.ring.count == 0:
            return
        self._dirty = False
        d = self.ring.data()
        t = d[:, 0]
        tmax = t[-1]
        win = self.sp_win.value()
        k = int(np.searchsorted(t, tmax - win))
        d = d[k:]
        t = d[:, 0]
        show_cmd = self.chk_cmd.isChecked()
        for j, name in enumerate(COLS[1:9], start=1):
            c = self.curves[name]
            if name.endswith("_cmd") and not show_cmd:
                c.setData([], [])
            else:
                c.setData(t, d[:, j], connect="finite")
        self.plots[0].setXRange(max(0.0, tmax - win), max(win, tmax), padding=0)
        rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0
        self.lbl_info.setText(f"反馈 {rate:.0f} Hz | 缓存 {self.ring.count} 点")

    def export_csv(self):
        if self.ring.count == 0:
            QMessageBox.information(self, "导出", "没有数据")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出曲线数据", "curve.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(COLS)
            for r in self.ring.data():
                w.writerow(["" if math.isnan(x) else f"{x:.6g}" for x in r])
