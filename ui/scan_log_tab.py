"""总线扫描 + 原始报文日志 + 手动发送。"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QCheckBox, QFileDialog, QGroupBox, QHBoxLayout, QHeaderView,
                               QLabel, QLineEdit, QMessageBox, QPlainTextEdit, QProgressBar,
                               QPushButton, QSpinBox, QSplitter, QTableWidget, QTableWidgetItem,
                               QVBoxLayout, QWidget)

from hm import protocol as P


class ScanLogTab(QWidget):
    use_motor = Signal(int, int)        # motor_id, master_id

    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self.t0: float | None = None
        self._lines: list[str] = []

        # ---- 扫描 ----
        sg = QGroupBox("总线扫描（0x7FF / 0xCC 查询）")
        self.sp_from = QSpinBox()
        self.sp_from.setRange(1, 127)
        self.sp_to = QSpinBox()
        self.sp_to.setRange(1, 127)
        self.sp_to.setValue(127)
        self.sp_win = QSpinBox()
        self.sp_win.setRange(5, 200)
        self.sp_win.setValue(20)
        self.sp_win.setSuffix(" ms")
        self.btn_scan = QPushButton("开始扫描")
        self.prog = QProgressBar()
        self.tbl = QTableWidget(0, 6)
        self.tbl.setHorizontalHeaderLabels(["电机 ID", "应答帧 ID (Master)", "位置", "错误码", "温度", ""])
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        row = QHBoxLayout()
        for w in (QLabel("ID 从"), self.sp_from, QLabel("到"), self.sp_to, QLabel("每个等待"),
                  self.sp_win, self.btn_scan):
            row.addWidget(w)
        row.addWidget(self.prog, 1)
        sl = QVBoxLayout(sg)
        sl.addLayout(row)
        sl.addWidget(self.tbl)
        note = QLabel("扫描时需失能；状态帧中 ID 仅低 4 位，结果按“请求 ID → 等待窗口内应答”关联。")
        note.setStyleSheet("color:gray")
        sl.addWidget(note)

        # ---- 日志 ----
        lg = QGroupBox("报文日志")
        self.chk_log = QCheckBox("记录")
        self.chk_log.setChecked(True)
        self.chk_pause = QCheckBox("暂停显示")
        self.chk_hide_mit = QCheckBox("隐藏 MIT 周期帧/状态帧")
        self.ed_filter = QLineEdit()
        self.ed_filter.setPlaceholderText("过滤 ID（十六进制，逗号分隔，如 7FF,0）")
        self.btn_clear = QPushButton("清空")
        self.btn_save = QPushButton("保存…")
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(5000)
        self.log.setFont(QFont("Consolas", 9))
        self.log.setLineWrapMode(QPlainTextEdit.NoWrap)
        lr = QHBoxLayout()
        for w in (self.chk_log, self.chk_pause, self.chk_hide_mit):
            lr.addWidget(w)
        lr.addWidget(self.ed_filter, 1)
        lr.addWidget(self.btn_clear)
        lr.addWidget(self.btn_save)

        # ---- 手动发送 ----
        self.ed_id = QLineEdit("7FF")
        self.ed_id.setMaximumWidth(60)
        self.ed_data = QLineEdit("01 00 33 3C 00 00 00 00")
        self.btn_send = QPushButton("发送")
        sr = QHBoxLayout()
        sr.addWidget(QLabel("手动发送  ID(hex)"))
        sr.addWidget(self.ed_id)
        sr.addWidget(QLabel("数据(hex)"))
        sr.addWidget(self.ed_data, 1)
        sr.addWidget(self.btn_send)
        ll = QVBoxLayout(lg)
        ll.addLayout(lr)
        ll.addWidget(self.log, 1)
        ll.addLayout(sr)

        sp = QSplitter(Qt.Vertical)
        sp.addWidget(sg)
        sp.addWidget(lg)
        sp.setStretchFactor(1, 1)
        lay = QVBoxLayout(self)
        lay.addWidget(sp)

        self.btn_scan.clicked.connect(self._scan)
        self.btn_clear.clicked.connect(self.log.clear)
        self.btn_save.clicked.connect(self._save)
        self.btn_send.clicked.connect(self._send)
        self.chk_log.toggled.connect(self._log_toggled)
        self.set_bound(None)

    def set_bound(self, worker):
        self.worker = worker
        for b in (self.btn_scan, self.btn_send):
            b.setEnabled(worker is not None)
        if worker:
            worker.log_enabled = self.chk_log.isChecked()

    def _log_toggled(self, on: bool):
        if self.worker:
            self.worker.log_enabled = on

    def on_state(self, s: dict):
        if self.worker:
            self.btn_scan.setEnabled(not s["blocked"] and not s["scanning"])

    # ---- 扫描 ----
    def _scan(self):
        a, b = self.sp_from.value(), self.sp_to.value()
        if a > b:
            a, b = b, a
        self.tbl.setRowCount(0)
        self.prog.setRange(0, b - a + 1)
        self.prog.setValue(0)
        self.btn_scan.setEnabled(False)
        self.worker.start_scan(range(a, b + 1), self.sp_win.value() / 1000.0)

    def on_scan_progress(self, i: int, n: int):
        self.prog.setRange(0, n)
        self.prog.setValue(i)

    def on_scan_done(self, found):
        self.tbl.setRowCount(0)
        for mid, cid, st in found:
            r = self.tbl.rowCount()
            self.tbl.insertRow(r)
            for c, txt in enumerate((str(mid), f"0x{cid:03X}", f"{st.pos:+.4f} rad", str(st.err),
                                     f"{st.t_mos}/{st.t_rotor}")):
                it = QTableWidgetItem(txt)
                it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                self.tbl.setItem(r, c, it)
            b = QPushButton("使用此电机")
            b.clicked.connect(lambda _=False, m=mid, c=cid: self.use_motor.emit(m, c))
            self.tbl.setCellWidget(r, 5, b)
        self.append_text(f"# 扫描完成，发现 {len(found)} 台: " +
                         ", ".join(f"ID{m}→Master 0x{c:X}" for m, c, _ in found))

    # ---- 日志 ----
    def _filter_ids(self) -> set[int] | None:
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
        hide = self.chk_hide_mit.isChecked()
        mid = self.worker.motor_id if self.worker else -1
        mst = self.worker.master_id if self.worker else -1
        lines = []
        for f in frames:
            if flt is not None and f.can_id not in flt:
                continue
            note = self._describe(f, mid, mst)
            if hide and note in ("MIT控制", "状态", "查询状态"):
                continue
            lines.append(f"{f.t - self.t0:10.4f}  {'TX' if f.tx else 'RX'}  {f.can_id:03X}  "
                         f"[{len(f.data)}] {P.hexdata(f.data):<23}  {note}")
        if lines:
            self.log.appendPlainText("\n".join(lines[-1000:]))

    def _describe(self, f, mid: int, mst: int) -> str:
        d = f.data
        if len(d) != 8:
            return ""
        if f.tx:
            if f.can_id == P.CONFIG_ID:
                return {P.CMD_READ: f"读 0x{d[3]:02X}", P.CMD_WRITE: f"写 0x{d[3]:02X}",
                        P.CMD_SAVE: "保存" if d[3] == 1 else f"0xAA/{d[3]:02X}",
                        P.CMD_STATE: "查询状态"}.get(d[2], "")
            if P.is_special(d):
                return P.SPECIAL_NAMES[d[7]]
            return "MIT控制"
        if d[2] in P.CONFIG_CMDS and (d[0] | (d[1] << 8)) == mid:
            r = P.REG_BY_RID.get(d[3])
            if d[2] == P.CMD_SAVE:
                return "保存应答"
            if r:
                return f"应答 {r.name} = {P.decode_value(r.typ, d[4:8]):.6g}"
            return "应答"
        if f.can_id == mst:
            return "状态"
        return ""

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
        if self.worker.enabled and P.is_special(data.ljust(8, b"\0")):
            if QMessageBox.question(self, "发送", "电机使能中，确认发送模式切换帧？") != QMessageBox.Yes:
                return
        self.worker.send_raw(cid, data)
