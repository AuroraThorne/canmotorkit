"""实时曲线：位置（累计）/ 速度 / 输出（指令 vs 反馈）/ 温度；环形缓冲 + 暂停 + CSV。

pyqtgraph 的 SI 前缀自动缩放（mrad、mNm 等）保持开启。
"""
from __future__ import annotations

import csv
import math

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QFileDialog, QHBoxLayout, QLabel,
                               QMessageBox, QPushButton, QVBoxLayout, QWidget)

from .widgets import CheckBox

COLS = ["t", "pos_raw", "pos_mt", "vel", "out_cmd", "out_fb", "ref", "temp", "temp2", "code", "active"]
IDX = {c: i for i, c in enumerate(COLS)}


class Ring:
    def __init__(self, n: int, cols: int):
        self.buf = np.full((n, cols), np.nan)
        self.n, self.i, self.count = n, 0, 0

    def extend(self, rows: np.ndarray):
        rows = rows[-self.n:]
        k = len(rows)
        end = self.i + k
        if end <= self.n:
            self.buf[self.i:end] = rows
        else:
            a = self.n - self.i
            self.buf[self.i:] = rows[:a]
            self.buf[:k - a] = rows[a:]
        self.i = end % self.n
        self.count = min(self.count + k, self.n)

    def data(self) -> np.ndarray:
        if self.count < self.n:
            return self.buf[:self.count]
        return np.vstack((self.buf[self.i:], self.buf[:self.i]))

    def clear(self):
        self.buf[:] = np.nan
        self.i = self.count = 0


class PlotWidget(QWidget):
    def __init__(self, parent=None, capacity: int = 200_000, compact: bool = False):
        super().__init__(parent)
        self.ring = Ring(capacity, len(COLS))
        self.t0: float | None = None
        self._dirty = False
        self.out_unit = "Nm"

        self.sp_win = QDoubleSpinBox()
        self.sp_win.setRange(0.5, 600)
        self.sp_win.setValue(10)
        self.sp_win.setSuffix(" s")
        self.cb_pos = QComboBox()
        self.cb_pos.addItems(["位置：多圈累计", "位置：电机原始值"])
        self.chk_pause = CheckBox("暂停")
        self.chk_ref = CheckBox("显示测试参考")
        self.chk_ref.setChecked(True)
        self.btn_clear = QPushButton("清空")
        self.btn_csv = QPushButton("导出 CSV")
        self.lbl_info = QLabel()
        self.lbl_info.setStyleSheet("color:#aaaaaa")
        bar = QHBoxLayout()
        for w in (QLabel("窗口"), self.sp_win, self.cb_pos, self.chk_pause, self.chk_ref,
                  self.btn_clear, self.btn_csv):
            bar.addWidget(w)
        bar.addStretch(1)
        bar.addWidget(self.lbl_info)

        self.glw = pg.GraphicsLayoutWidget()
        self.glw.setBackground("w")
        self.curves: dict[str, pg.PlotDataItem] = {}
        self.plots = []
        specs = [("位置", "rad", [("pos", "反馈", "#1565c0", False), ("ref_pos", "参考", "#9e9e9e", True)]),
                 ("速度", "rad/s", [("vel", "反馈", "#1565c0", False), ("ref_vel", "参考", "#9e9e9e", True)]),
                 ("输出", "Nm", [("out_cmd", "指令", "#e65100", True), ("out_fb", "反馈", "#1565c0", False)])]
        if not compact:
            specs.append(("温度", "℃", [("temp", "温度 1", "#e65100", False), ("temp2", "温度 2", "#6a1b9a", False)]))
        first = None
        for i, (title, unit, cs) in enumerate(specs):
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
            for key, name, color, dash in cs:
                pen = pg.mkPen(color, width=1.5, style=Qt.DashLine if dash else Qt.SolidLine)
                self.curves[key] = p.plot(name=name, pen=pen)
            self.plots.append(p)
        self.plots[-1].setLabel("bottom", "时间", units="s")
        self.ref_target = ""          # "pos" / "vel" / "out"

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addLayout(bar)
        lay.addWidget(self.glw, 1)
        self.btn_clear.clicked.connect(self.clear)
        self.btn_csv.clicked.connect(self.export_csv)
        for w in (self.chk_ref, self.cb_pos, self.sp_win):
            sig = w.toggled if isinstance(w, CheckBox) else (
                w.currentIndexChanged if isinstance(w, QComboBox) else w.valueChanged)
            sig.connect(lambda *_: self._mark())
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(33)

    def set_unit(self, unit: str, name: str = "输出"):
        self.out_unit = unit
        self.plots[2].setLabel("left", name, units=unit)

    def set_ref_target(self, target: str):
        self.ref_target = target
        self._mark()

    def _mark(self):
        self._dirty = True

    def add_samples(self, samples):
        if not samples:
            return
        if self.t0 is None:
            self.t0 = samples[0].t
        nan = math.nan
        rows = np.array([(s.t - self.t0, s.fb.pos, s.pos_mt, s.fb.vel, s.out, s.fb.out, s.ref,
                          s.fb.temp, s.fb.temp2, s.fb.code, 1.0 if s.active else 0.0)
                         for s in samples], dtype=float)
        self.ring.extend(rows)
        self._dirty = True
        _ = nan

    def clear(self):
        self.ring.clear()
        self.t0 = None
        for c in self.curves.values():
            c.setData([], [])

    def _refresh(self):
        if not self._dirty or self.chk_pause.isChecked() or self.ring.count == 0 or not self.isVisible():
            return
        self._dirty = False
        d = self.ring.data()
        t = d[:, 0]
        tmax = t[-1]
        win = self.sp_win.value()
        d = d[int(np.searchsorted(t, tmax - win)):]
        t = d[:, 0]
        pos = d[:, IDX["pos_mt"] if self.cb_pos.currentIndex() == 0 else IDX["pos_raw"]]
        self.curves["pos"].setData(t, pos, connect="finite")
        self.curves["vel"].setData(t, d[:, IDX["vel"]], connect="finite")
        self.curves["out_cmd"].setData(t, d[:, IDX["out_cmd"]], connect="finite")
        self.curves["out_fb"].setData(t, d[:, IDX["out_fb"]], connect="finite")
        if "temp" in self.curves:
            self.curves["temp"].setData(t, d[:, IDX["temp"]], connect="finite")
            self.curves["temp2"].setData(t, d[:, IDX["temp2"]], connect="finite")
        ref = d[:, IDX["ref"]] if self.chk_ref.isChecked() else np.full(len(t), np.nan)
        self.curves["ref_pos"].setData(t, ref if self.ref_target == "pos" else t * np.nan, connect="finite")
        self.curves["ref_vel"].setData(t, ref if self.ref_target == "vel" else t * np.nan, connect="finite")
        self.plots[0].setXRange(max(0.0, tmax - win), max(win, tmax), padding=0)
        rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0
        self.lbl_info.setText(f"反馈 {rate:.0f} Hz｜缓存 {self.ring.count} 点")

    def export_csv(self):
        if self.ring.count == 0:
            QMessageBox.information(self, "导出", "没有数据")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出曲线数据", "curve.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow([c if c not in ("out_cmd", "out_fb") else f"{c}[{self.out_unit}]" for c in COLS])
            for r in self.ring.data():
                w.writerow(["" if math.isnan(x) else f"{x:.7g}" for x in r])
