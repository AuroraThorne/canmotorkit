"""② 总线电机：扫描、确认型号并选中唯一电机进行调试。"""
from __future__ import annotations

from PySide6.QtCore import QEvent, QTimer, Qt, Signal
from PySide6.QtWidgets import (QAbstractItemView, QComboBox, QGroupBox, QHBoxLayout, QHeaderView,
                               QLabel, QLineEdit, QMessageBox, QProgressBar, QPushButton, QSpinBox,
                               QTableWidget, QVBoxLayout)

from canmotorkit.drivers import dji
from canmotorkit.drivers.catalog import PROTOCOLS, make_driver

from .widgets import ArrowOnlyComboBox

ALL_MODELS = [(p, m) for p, (_, ms) in PROTOCOLS.items() for m in ms]


class _FillHeader(QHeaderView):
    """三列始终填满表格，拖动一个分界线时只调整相邻两列。"""

    MIN_WIDTHS = (180, 80, 120)

    def __init__(self, parent=None):
        super().__init__(Qt.Horizontal, parent)
        self._ready = False
        self._adjusting = False
        self.setSectionsMovable(False)
        self.setStretchLastSection(False)
        self.setMinimumSectionSize(min(self.MIN_WIDTHS))
        self.sectionResized.connect(self._on_section_resized)

    def set_default_sizes(self):
        width = self.width()
        if width <= 0:
            return False
        total_min = sum(self.MIN_WIDTHS)
        width = max(width, total_min)
        first = max(self.MIN_WIDTHS[0], int(width * 0.50))
        second = max(self.MIN_WIDTHS[1], int(width * 0.20))
        third = max(self.MIN_WIDTHS[2], width - first - second)
        if first + second + third > width:
            first = width - second - third
        self._adjusting = True
        for index, size in enumerate((first, second, third)):
            self.resizeSection(index, size)
        self._adjusting = False
        self._ready = True
        self._fill_to_width()
        return True

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._ready and not self._adjusting:
            self._fill_to_width()

    def _fill_to_width(self):
        target = self.width()
        current = sum(self.sectionSize(i) for i in range(self.count()))
        delta = target - current
        if delta == 0:
            return
        self._adjusting = True
        if delta > 0:
            self.resizeSection(self.count() - 1, self.sectionSize(self.count() - 1) + delta)
        else:
            for index in reversed(range(self.count())):
                size = self.sectionSize(index)
                minimum = self.MIN_WIDTHS[index]
                reduced = max(minimum, size + delta)
                self.resizeSection(index, reduced)
                delta -= reduced - size
                if delta == 0:
                    break
        self._adjusting = False

    def _on_section_resized(self, index, old_size, new_size):
        if not self._ready or self._adjusting or old_size == new_size:
            return
        minimum = self.MIN_WIDTHS[index]
        if new_size < minimum:
            self._adjusting = True
            self.resizeSection(index, minimum)
            new_size = minimum
            self._adjusting = False
        neighbor = index + 1 if index + 1 < self.count() else index - 1
        if neighbor < 0:
            return
        delta = new_size - old_size
        neighbor_size = self.sectionSize(neighbor)
        neighbor_min = self.MIN_WIDTHS[neighbor]
        target = neighbor_size - delta
        if target < neighbor_min:
            available = neighbor_size - neighbor_min
            corrected = old_size + available
            self._adjusting = True
            self.resizeSection(index, corrected)
            self.resizeSection(neighbor, neighbor_min)
            self._adjusting = False
            return
        self._adjusting = True
        self.resizeSection(neighbor, target)
        self._adjusting = False


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
        self.sp_to.setValue(127)
        self.sp_to.setPrefix("达妙/HM ID 1~")
        self.sp_to.setToolTip("达妙/HM 使用 1~127 的 ID 查询范围；大疆不需要设置 ID，会自动监听反馈广播")
        self.lbl_dji_scan = QLabel("大疆：自动监听")
        self.lbl_dji_scan.setStyleSheet("color:#aaaaaa")
        self.lbl_dji_scan.setToolTip("大疆电调主动广播反馈，扫描时自动监听，不需要单独设置扫描 ID")
        self.prog = QProgressBar()
        self.prog.setTextVisible(True)
        self.prog.setFormat("")
        self.tbl = QTableWidget(0, 3)
        self.tbl.setHorizontalHeader(_FillHeader(self.tbl))
        self.tbl.setHorizontalHeaderLabels(["协议 / 型号", "电机 ID", "反馈帧 ID"])
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tbl.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl.itemSelectionChanged.connect(self._update_selection_style)
        hh = self.tbl.horizontalHeader()
        for column in range(3):
            hh.setSectionResizeMode(column, QHeaderView.Interactive)
        hh.setStyleSheet(
            "QHeaderView::section{background:#303030;color:#f1f1f1;"
            "border:1px solid #666666;border-right:2px solid #707070;"
            "padding:4px 6px;}"
            "QHeaderView::section:hover{background:#3e3e3e;}"
        )
        self.tbl.setStyleSheet(
            "QTableWidget{gridline-color:#666666;}"
        )
        self._columns_initialized = False
        QTimer.singleShot(0, self._set_default_column_widths)
        self.tbl.setMinimumHeight(130)
        self.btn_use = QPushButton("调试选中电机")
        self.btn_use.setStyleSheet("QPushButton:enabled{font-weight:bold}")

        top = QHBoxLayout()
        top.addWidget(self.btn_scan)
        top.addWidget(self.sp_to)
        top.addWidget(self.lbl_dji_scan)
        lay = QVBoxLayout(self)
        lay.addLayout(top)
        lay.addWidget(self.prog)
        lay.addWidget(self.tbl)
        b = QHBoxLayout()
        b.addWidget(self.btn_use, 1)
        lay.addLayout(b)

        self.btn_scan.clicked.connect(self.scan_bus)
        self.btn_use.clicked.connect(self._use)
        self.tbl.itemDoubleClicked.connect(lambda _: self._use())
        self.set_bound(None)

    # ---------- 外部 ----------
    def set_bound(self, worker):
        self.worker = worker
        for w in (self.btn_scan, self.btn_use):
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
        self.entries = list(found)
        self._refresh()
        self.prog.setFormat(f"扫描完成，发现 {len(found)} 台")
        if len(self.entries) == 1:
            self.tbl.selectRow(0)

    # ---------- 内部 ----------
    def scan_bus(self):
        if self.worker:
            self.btn_scan.setEnabled(False)
            self.worker.start_scan(range(1, self.sp_to.value() + 1))

    def _refresh(self):
        self.tbl.setRowCount(0)
        for e in self.entries:
            r = self.tbl.rowCount()
            self.tbl.insertRow(r)
            cb = ArrowOnlyComboBox()
            opts = []
            for m, mid in e["candidates"]:
                p = "dji" if m in dji.MODELS else e["protocol"]
                opts.append((p, m, mid))
            for p, m in ALL_MODELS:          # 允许用户改选任意协议
                if p == e["protocol"] and not any(o[1] == m for o in opts):
                    opts.append((p, m, e["motor_id"]))
            selected = (e["protocol"], e["model"], e["motor_id"])
            if selected in opts:
                opts.remove(selected)
                opts.insert(0, selected)
            for p, m, mid in opts:
                cb.addItem(f"{PROTOCOLS[p][0]} · {m}", (p, m, mid))
            cb.currentIndexChanged.connect(lambda _=0, row=r, c=cb: self._model_changed(row, c))
            cb.clicked.connect(lambda row=r: self.tbl.selectRow(row))
            self.tbl.setCellWidget(r, 0, cb)
            fb = e.get("fb_id")
            for c, txt in ((1, str(e["motor_id"])), (2, f"0x{fb:03X}" if fb is not None else "")):
                field = QLineEdit(txt)
                field.setReadOnly(True)
                field.setFocusPolicy(Qt.NoFocus)
                field.setAlignment(Qt.AlignCenter)
                field.setToolTip(txt or "暂无反馈帧 ID")
                field.setProperty("motorRow", r)
                field.installEventFilter(self)
                self.tbl.setCellWidget(r, c, field)
        self._update_selection_style()

    def eventFilter(self, watched, event):
        if event.type() == QEvent.MouseButtonPress:
            row = watched.property("motorRow")
            if row is not None:
                self.tbl.selectRow(int(row))
        return super().eventFilter(watched, event)

    def _set_default_column_widths(self):
        if self._columns_initialized:
            return
        header = self.tbl.horizontalHeader()
        if header.width() <= 0:
            QTimer.singleShot(50, self._set_default_column_widths)
            return
        self._columns_initialized = header.set_default_sizes()

    def _model_changed(self, row: int, cb: QComboBox):
        self.tbl.selectRow(row)
        self._model_pick(row, cb)

    def _update_selection_style(self):
        selected_row = self.tbl.currentRow()
        for row in range(self.tbl.rowCount()):
            cb = self.tbl.cellWidget(row, 0)
            if cb is None:
                continue
            if row == selected_row:
                cb.setStyleSheet(
                    "QComboBox{background:#356a9a;color:#ffffff;"
                    "selection-background-color:#356a9a;selection-color:#ffffff;}"
                )
            else:
                cb.setStyleSheet("")
            for column in (1, 2):
                field = self.tbl.cellWidget(row, column)
                if field is None:
                    continue
                field.setStyleSheet(
                    "QLineEdit{background:#356a9a;color:#ffffff;border:1px solid #6ea8d6;"
                    "border-radius:2px;padding:2px 4px;}"
                    if row == selected_row else
                    "QLineEdit{background:#202020;color:#f1f1f1;border:1px solid #555555;"
                    "border-radius:2px;padding:2px 4px;}"
                )

    def _model_pick(self, row: int, cb: QComboBox):
        p, m, mid = cb.currentData()
        e = self.entries[row]
        e.update(protocol=p, model=m, motor_id=mid)
        if p == "dji":
            e["fb_id"] = dji.MODELS[m].fb_base + mid
        elif e.get("master_id") is not None:
            e["fb_id"] = e["master_id"]
        candidates = [(m0, mid0) for m0, mid0 in e.get("candidates", [])
                      if not (m0 == m and mid0 == mid)]
        e["candidates"] = [(m, mid), *candidates]
        self.tbl.cellWidget(row, 1).setText(str(mid))
        fb = e.get("fb_id")
        fb_text = f"0x{fb:03X}" if fb is not None else ""
        self.tbl.cellWidget(row, 2).setText(fb_text)
        self.tbl.cellWidget(row, 2).setToolTip(fb_text or "暂无反馈帧 ID")
        if cb.currentIndex() > 0:
            items = [(cb.itemText(i), cb.itemData(i)) for i in range(cb.count())]
            selected_item = items.pop(cb.currentIndex())
            cb.blockSignals(True)
            cb.clear()
            cb.addItem(*selected_item)
            for item in items:
                cb.addItem(*item)
            cb.setCurrentIndex(0)
            cb.blockSignals(False)

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
