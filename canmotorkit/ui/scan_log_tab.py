"""报文页：原始帧日志（解析注释/过滤/保存）、手动发送、往返延迟直方图。"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QCheckBox, QFileDialog, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
                               QMessageBox, QPlainTextEdit, QPushButton, QSplitter, QVBoxLayout,
                               QWidget)

from canmotorkit.drivers import dm_protocol as P


class ScanLogTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self.driver = None
        self.t0: float | None = None

        lg = QGroupBox("报文日志")
        self.chk_log = QCheckBox("记录")
        self.chk_log.setChecked(True)
        self.chk_pause = QCheckBox("暂停显示")
        self.chk_hide = QCheckBox("隐藏周期帧（控制/反馈/查询）")
        self.chk_mine = QCheckBox("只显示当前电机")
        self.ed_filter = QLineEdit()
        self.ed_filter.setPlaceholderText("过滤 ID（十六进制，逗号分隔，如 7FF,0,201）")
        self.btn_clear = QPushButton("清空")
        self.btn_save = QPushButton("保存…")
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(5000)
        self.log.setFont(QFont("Consolas", 9))
        self.log.setLineWrapMode(QPlainTextEdit.NoWrap)
        lr = QHBoxLayout()
        for w in (self.chk_log, self.chk_pause, self.chk_hide, self.chk_mine):
            lr.addWidget(w)
        lr.addWidget(self.ed_filter, 1)
        lr.addWidget(self.btn_clear)
        lr.addWidget(self.btn_save)
        self.ed_id = QLineEdit("7FF")
        self.ed_id.setMaximumWidth(60)
        self.ed_data = QLineEdit("01 00 33 3C 00 00 00 00")
        self.btn_send = QPushButton("发送")
        sr = QHBoxLayout()
        for w in (QLabel("手动发送  ID(hex)"), self.ed_id, QLabel("数据(hex)")):
            sr.addWidget(w)
        sr.addWidget(self.ed_data, 1)
        sr.addWidget(self.btn_send)
        ll = QVBoxLayout(lg)
        ll.addLayout(lr)
        ll.addWidget(self.log, 1)
        ll.addLayout(sr)

        hg = QGroupBox("往返延迟（发送 → 收到应答，仅问答式协议）")
        self.hist = pg.PlotWidget(background="w")
        self.hist.setLabel("bottom", "延迟", units="s")
        self.hist.setLabel("left", "次数")
        self.hist.showGrid(x=True, y=True, alpha=0.3)
        self.bars = pg.BarGraphItem(x=[], height=[], width=1e-4, brush="#1565c0")
        self.hist.addItem(self.bars)
        self.lbl_lat = QLabel("—")
        self.btn_lat_clear = QPushButton("清空统计")
        hl = QVBoxLayout(hg)
        r = QHBoxLayout()
        r.addWidget(self.lbl_lat, 1)
        r.addWidget(self.btn_lat_clear)
        hl.addLayout(r)
        hl.addWidget(self.hist)

        sp = QSplitter(Qt.Vertical)
        sp.addWidget(lg)
        sp.addWidget(hg)
        sp.setSizes([600, 250])
        lay = QVBoxLayout(self)
        lay.addWidget(sp)

        self.btn_clear.clicked.connect(self.log.clear)
        self.btn_save.clicked.connect(self._save)
        self.btn_send.clicked.connect(self._send)
        self.chk_log.toggled.connect(lambda on: self.worker and setattr(self.worker, "log_enabled", on))
        self.btn_lat_clear.clicked.connect(lambda: self.worker and self.worker.latencies.clear())
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._hist)
        self.timer.start(500)
        self.set_bound(None)

    def set_bound(self, worker):
        self.worker = worker
        self.btn_send.setEnabled(worker is not None)
        if worker:
            worker.log_enabled = self.chk_log.isChecked()
        self.driver = None

    def set_driver(self, d):
        self.driver = d

    def _hist(self):
        w = self.worker
        if w is None or not self.isVisible():
            return
        xs = np.array(list(w.latencies))
        if len(xs) < 2:
            self.lbl_lat.setText("暂无数据（大疆为主动广播，无往返延迟）" if self.driver and
                                 not self.driver.caps.enable_cmd else "暂无数据")
            return
        h, e = np.histogram(xs, bins=40)
        self.bars.setOpts(x=(e[:-1] + e[1:]) / 2, height=h, width=(e[1] - e[0]) * 0.9)
        p = np.percentile(xs, [50, 95, 99])
        self.lbl_lat.setText(f"n={len(xs)}  p50 {p[0] * 1e3:.2f} ms  p95 {p[1] * 1e3:.2f} ms  "
                             f"p99 {p[2] * 1e3:.2f} ms  max {xs.max() * 1e3:.2f} ms")

    def _filter_ids(self):
        t = self.ed_filter.text().strip()
        if not t:
            return None
        out = set()
        for x in t.replace(" ", "").split(","):
            try:
                out.add(int(x, 16))
            except ValueError:
                pass
        return out

    def on_frames(self, frames):
        if self.chk_pause.isChecked() or not frames:
            return
        if self.t0 is None:
            self.t0 = frames[0].t
        flt = self._filter_ids()
        hide = self.chk_hide.isChecked()
        mine = self.chk_mine.isChecked()
        d = self.driver
        lines = []
        for f in frames:
            if flt is not None and f.can_id not in flt:
                continue
            note = d.describe(f.can_id, f.data, f.tx) if d else ""
            if mine and not note:
                continue
            if hide and (note in ("MIT控制", "查询状态", "反馈") or note.startswith("状态 ")
                         or note.startswith("电流指令")):
                continue
            lines.append(f"{f.t - self.t0:10.4f}  {'TX' if f.tx else 'RX'}  {f.can_id:03X}  "
                         f"[{len(f.data)}] {P.hexdata(f.data):<23}  {note}")
        if lines:
            self.log.appendPlainText("\n".join(lines[-1000:]))

    def append_text(self, s: str):
        self.log.appendPlainText(s)

    def _save(self):
        path, _ = QFileDialog.getSaveFileName(self, "保存日志", "can_log.txt", "Text (*.txt)")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.log.toPlainText())

    def _send(self):
        try:
            cid = int(self.ed_id.text().strip(), 16)
            data = bytes.fromhex(self.ed_data.text().replace(",", " "))
            if not 0 <= cid <= 0x7FF or len(data) > 8:
                raise ValueError("ID 需在 0~7FF，数据最多 8 字节")
        except ValueError as e:
            QMessageBox.warning(self, "发送", str(e))
            return
        if QMessageBox.question(self, "手动发送", f"发送 {cid:03X} [{P.hexdata(data)}]？\n"
                                "手动帧不经过安全限制。") == QMessageBox.Yes:
            self.worker.send_raw(cid, data)
