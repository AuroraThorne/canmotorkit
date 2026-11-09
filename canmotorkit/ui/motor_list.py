"""② 总线电机：扫描（自动识别协议/型号）、手动添加、选中唯一电机进行调试。"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QAbstractItemView, QComboBox, QFormLayout, QGroupBox, QHBoxLayout,
                               QHeaderView, QLabel, QMessageBox, QProgressBar, QPushButton, QSpinBox,
                               QTableWidget, QTableWidgetItem, QVBoxLayout)

from canmotorkit.drivers import dji
from canmotorkit.drivers.catalog import PROTOCOLS, make_driver

ALL_MODELS = [(p, m) for p, (_, ms) in PROTOCOLS.items() for m in ms]


class MotorListPanel(QGroupBox):
    motor_selected = Signal(object)     # Driver

    def __init__(self, parent=None):
        super().__init__("② 总线电机", parent)
        self.worker = None
        self.entries: list[dict] = []
        self.btn_scan = QPushButton("扫描总线")
        self.btn_scan.setToolTip("先监听大疆广播反馈，再用 0x7FF/0xCC 逐个查询达妙/HM，\n"
                                 "并读 CTRL_MODE / v_bus 区分达妙与 HM。需在失能状态下进行。")
        self.sp_to = QSpinBox()
        self.sp_to.setRange(1, 127)
        self.sp_to.setValue(32)
        self.sp_to.setPrefix("达妙 ID 1~")
        self.prog = QProgressBar()
        self.prog.setTextVisible(True)
        self.prog.setFormat("")
        self.tbl = QTableWidget(0, 4)
        self.tbl.setHorizontalHeaderLabels(["协议 / 型号", "ID", "反馈", "依据"])
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tbl.setEditTriggers(QAbstractItemView.NoEditTriggers)
        hh = self.tbl.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(3, QHeaderView.Stretch)
        self.tbl.setMinimumHeight(130)
        self.btn_use = QPushButton("调试选中电机")
        self.btn_use.setStyleSheet("QPushButton:enabled{font-weight:bold}")
        self.btn_del = QPushButton("移除")

        # 手动添加
        self.cb_model = QComboBox()
        for p, m in ALL_MODELS:
            self.cb_model.addItem(f"{PROTOCOLS[p][0]} · {m}", (p, m))
        self.sp_id = QSpinBox()
        self.sp_id.setRange(1, 127)
        self.sp_id.setValue(1)
        self.sp_mst = QSpinBox()
        self.sp_mst.setRange(0, 0x7FE)
        self.sp_mst.setDisplayIntegerBase(16)
        self.sp_mst.setPrefix("0x")
        self.lbl_mst = QLabel("Master")
        self.btn_add = QPushButton("添加")
        add = QGroupBox("手动添加（也可用于修正扫描识别的协议）")
        fa = QFormLayout(add)
        fa.addRow("协议/型号", self.cb_model)
        r = QHBoxLayout()
        r.addWidget(QLabel("ID"))
        r.addWidget(self.sp_id)
        r.addWidget(self.lbl_mst)
        r.addWidget(self.sp_mst)
        r.addWidget(self.btn_add)
        fa.addRow(r)

        top = QHBoxLayout()
        top.addWidget(self.btn_scan)
        top.addWidget(self.sp_to)
        lay = QVBoxLayout(self)
        lay.addLayout(top)
        lay.addWidget(self.prog)
        lay.addWidget(self.tbl)
        b = QHBoxLayout()
        b.addWidget(self.btn_use, 1)
        b.addWidget(self.btn_del)
        lay.addLayout(b)
        lay.addWidget(add)

        self.btn_scan.clicked.connect(self._scan)
        self.btn_add.clicked.connect(self._add_manual)
        self.btn_use.clicked.connect(self._use)
        self.btn_del.clicked.connect(self._delete)
        self.tbl.itemDoubleClicked.connect(lambda _: self._use())
        self.cb_model.currentIndexChanged.connect(self._model_changed)
        self._model_changed()
        self.set_bound(None)

    # ---------- 外部 ----------
    def set_bound(self, worker):
        self.worker = worker
        for w in (self.btn_scan, self.btn_use, self.btn_add, self.btn_del):
            w.setEnabled(worker is not None)
        if worker is None:
            self.entries.clear()
            self.tbl.setRowCount(0)
            self.prog.setValue(0)
            self.prog.setFormat("")

    def on_state(self, s: dict):
        busy = s["active"] or s["scanning"]
        self.btn_scan.setEnabled(not busy)
        self.btn_use.setEnabled(not s["active"])
        self.btn_scan.setToolTip("请先失能" if s["active"] else self.btn_scan.toolTip())

    def on_scan_progress(self, phase: str, i: int, n: int):
        self.prog.setRange(0, max(1, n))
        self.prog.setValue(i)
        self.prog.setFormat(f"{phase} {i}/{n}" if phase != "完成" else "扫描完成")

    def on_scan_done(self, found: list):
        self.btn_scan.setEnabled(True)
        keep = [e for e in self.entries if e.get("manual")]
        self.entries = keep + list(found)
        self._refresh()
        self.prog.setFormat(f"扫描完成，发现 {len(found)} 台")
        if len(self.entries) == 1:
            self.tbl.selectRow(0)

    # ---------- 内部 ----------
    def _scan(self):
        if self.worker:
            self.btn_scan.setEnabled(False)
            self.worker.start_scan(range(1, self.sp_to.value() + 1))

    def _model_changed(self):
        p, m = self.cb_model.currentData()
        is_dm = p == "dm"
        self.sp_mst.setVisible(is_dm)
        self.lbl_mst.setVisible(is_dm)
        if p == "dji":
            ids = dji.MODELS[m].ids
            self.sp_id.setRange(ids.start, ids.stop - 1)
        else:
            self.sp_id.setRange(1, 127)

    def _add_manual(self):
        p, m = self.cb_model.currentData()
        mid = self.sp_id.value()
        mst = self.sp_mst.value() if p == "dm" else None
        try:
            drv = make_driver(p, m, mid, mst or 0)
        except ValueError as e:
            QMessageBox.warning(self, "添加电机", str(e))
            return
        fb = getattr(drv, "fb_id", mst)
        self.entries.append({"protocol": p, "model": m, "motor_id": mid, "master_id": mst,
                             "fb_id": fb, "candidates": [(m, mid)], "evidence": "手动添加",
                             "manual": True})
        self._refresh()
        self.tbl.selectRow(len(self.entries) - 1)

    def _refresh(self):
        self.tbl.setRowCount(0)
        for e in self.entries:
            r = self.tbl.rowCount()
            self.tbl.insertRow(r)
            cb = QComboBox()
            opts = []
            for m, mid in e["candidates"]:
                p = "dji" if m in dji.MODELS else e["protocol"]
                opts.append((p, m, mid))
            for p, m in ALL_MODELS:          # 允许用户改选任意协议
                if p == e["protocol"] and not any(o[1] == m for o in opts):
                    opts.append((p, m, e["motor_id"]))
            for p, m, mid in opts:
                cb.addItem(f"{PROTOCOLS[p][0]} · {m}", (p, m, mid))
            cb.currentIndexChanged.connect(lambda _=0, row=r, c=cb: self._model_pick(row, c))
            self.tbl.setCellWidget(r, 0, cb)
            fb = e.get("fb_id")
            for c, txt in ((1, str(e["motor_id"])), (2, f"0x{fb:03X}" if fb is not None else "—"),
                           (3, e.get("evidence", ""))):
                it = QTableWidgetItem(txt)
                it.setToolTip(txt)
                self.tbl.setItem(r, c, it)

    def _model_pick(self, row: int, cb: QComboBox):
        p, m, mid = cb.currentData()
        e = self.entries[row]
        e.update(protocol=p, model=m, motor_id=mid)
        self.tbl.item(row, 1).setText(str(mid))

    def _delete(self):
        r = self.tbl.currentRow()
        if 0 <= r < len(self.entries):
            del self.entries[r]
            self._refresh()

    def _use(self):
        r = self.tbl.currentRow()
        if not (0 <= r < len(self.entries)) or self.worker is None:
            QMessageBox.information(self, "选择电机", "请先在列表中选中一台电机")
            return
        e = self.entries[r]
        try:
            drv = make_driver(e["protocol"], e["model"], e["motor_id"], e.get("master_id") or 0)
        except ValueError as ex:
            QMessageBox.warning(self, "选择电机", str(ex))
            return
        self.motor_selected.emit(drv)
