"""寄存器页：全部读取/单个写入/保存 Flash/JSON 导入导出/改 ID。"""
from __future__ import annotations

import json

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (QAbstractItemView, QFileDialog, QHBoxLayout, QHeaderView,
                               QInputDialog, QLabel, QMessageBox, QPushButton, QTableWidget,
                               QTableWidgetItem, QVBoxLayout, QWidget)

from hm import protocol as P

C_RID, C_NAME, C_TYPE, C_VAL, C_NEW, C_DESC = range(6)
TYPE_TXT = {"f": "float", "u": "uint32", "i": "int32"}
MODIFIED = QColor("#fff3c4")


def fmt(typ: str, v) -> str:
    if v is None:
        return ""
    return f"{v:.6g}" if typ == "f" else str(int(v))


class RegisterTab(QWidget):
    limits_read = Signal(object)        # dict name->value，用于更新量程
    ids_changed = Signal(int, int)      # 新 motor_id, master_id

    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self.enabled = False
        self.values: dict[int, float | int] = {}
        self._row_of = {r.rid: i for i, r in enumerate(P.REGISTERS)}
        self._pending_id_change: dict[int, int] = {}

        self.table = QTableWidget(len(P.REGISTERS), 6)
        self.table.setHorizontalHeaderLabels(["RID", "名称", "类型", "当前值", "新值(双击编辑)", "说明"])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(C_DESC, QHeaderView.Stretch)
        for i, r in enumerate(P.REGISTERS):
            cells = [f"0x{r.rid:02X}", r.name, TYPE_TXT[r.typ] + (" (只读)" if r.ro else ""), "", "",
                     r.desc + (f" [{r.unit}]" if r.unit else "")]
            for c, text in enumerate(cells):
                it = QTableWidgetItem(text)
                if c != C_NEW or r.ro:
                    it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                if r.ro and c == C_NEW:
                    it.setBackground(QBrush(QColor("#eeeeee")))
                self.table.setItem(i, c, it)

        self.btn_read_all = QPushButton("全部读取")
        self.btn_read_sel = QPushButton("读取选中")
        self.btn_write = QPushButton("写入已修改")
        self.btn_save = QPushButton("保存到 Flash")
        self.btn_save.setStyleSheet("QPushButton{font-weight:bold}")
        self.btn_save.setToolTip("保存参数只在失能模式下生效")
        self.btn_change_id = QPushButton("修改 ID…")
        self.btn_export = QPushButton("导出 JSON")
        self.btn_import = QPushButton("导入 JSON")
        self.lbl = QLabel()
        self.lbl.setStyleSheet("color:gray")
        bar = QHBoxLayout()
        for b in (self.btn_read_all, self.btn_read_sel, self.btn_write, self.btn_save,
                  self.btn_change_id, self.btn_export, self.btn_import):
            bar.addWidget(b)
        bar.addStretch(1)

        note = QLabel("提示：写入只修改 RAM，需“保存到 Flash”（失能状态）才会掉电保持。"
                      "波特率仅 CAN FD 版可修改，此处只读。")
        note.setStyleSheet("color:#795548")
        lay = QVBoxLayout(self)
        lay.addLayout(bar)
        lay.addWidget(note)
        lay.addWidget(self.table, 1)
        lay.addWidget(self.lbl)

        self.btn_read_all.clicked.connect(self.read_all)
        self.btn_read_sel.clicked.connect(self._read_sel)
        self.btn_write.clicked.connect(self._write_modified)
        self.btn_save.clicked.connect(self._save)
        self.btn_change_id.clicked.connect(self._change_id)
        self.btn_export.clicked.connect(self._export)
        self.btn_import.clicked.connect(self._import)
        self.table.itemChanged.connect(self._on_item_changed)
        self.set_bound(None)

    # ---------- 外部 ----------
    def set_bound(self, worker):
        self.worker = worker
        for b in (self.btn_read_all, self.btn_read_sel, self.btn_write, self.btn_save,
                  self.btn_change_id):
            b.setEnabled(worker is not None)

    def on_state(self, s: dict):
        en = s["enabled"]
        if en != self.enabled:
            self.enabled = en
            tip = "使能中不可用，请先失能" if en else ""
            for b in (self.btn_save, self.btn_change_id):
                b.setEnabled(self.worker is not None and not en)
                b.setToolTip(tip)
        if s.get("queue"):
            self.lbl.setText(f"队列中请求: {s['queue']}")

    def read_all(self):
        if not self.worker:
            return
        for r in P.REGISTERS:
            self.worker.read_reg(r.rid)
        self.lbl.setText(f"正在读取 {len(P.REGISTERS)} 个寄存器…")

    def read_limits(self):
        if self.worker:
            for rid in (0x15, 0x16, 0x17):
                self.worker.read_reg(rid)

    def on_reg_done(self, kind: str, rid: int, value, err: str):
        if kind == "save":
            if err:
                QMessageBox.warning(self, "保存失败", err)
            else:
                self.lbl.setText("✔ 参数已保存到 Flash")
                if self._pending_id_change:
                    mid = self._pending_id_change.get(0x08, self.worker.motor_id)
                    mst = self._pending_id_change.get(0x07, self.worker.master_id)
                    self._pending_id_change = {}
                    QMessageBox.information(
                        self, "ID 已修改",
                        f"新电机 ID = {mid}，新 Master ID = {mst}。\n"
                        "请断电重启电机后用新 ID 重新连接（连接面板已自动更新）。")
                    self.ids_changed.emit(mid, mst)
            return
        reg = P.REG_BY_RID.get(rid)
        row = self._row_of.get(rid)
        if row is None:
            return
        if err:
            self.lbl.setText(f"✖ {kind} 0x{rid:02X} {reg.name if reg else ''}: {err}")
            if kind == "write":
                self._pending_id_change.pop(rid, None)
            return
        self.values[rid] = value
        self.table.blockSignals(True)
        self.table.item(row, C_VAL).setText(fmt(reg.typ, value))
        if kind == "write":
            it = self.table.item(row, C_NEW)
            it.setText("")
            it.setBackground(QBrush())
        self.table.blockSignals(False)
        self.lbl.setText(f"✔ {kind} 0x{rid:02X} {reg.name} = {fmt(reg.typ, value)}")
        if rid in (0x15, 0x16, 0x17) and kind == "read":
            self.limits_read.emit({reg.name: value})

    # ---------- 内部 ----------
    def _on_item_changed(self, it: QTableWidgetItem):
        if it.column() != C_NEW:
            return
        it.setBackground(QBrush(MODIFIED) if it.text().strip() else QBrush())

    def _read_sel(self):
        rows = sorted({i.row() for i in self.table.selectedIndexes()})
        for r in rows:
            self.worker.read_reg(P.REGISTERS[r].rid)

    def _parse(self, reg: P.Register, text: str):
        text = text.strip()
        if reg.typ == "f":
            return float(text)
        v = int(text, 0)
        if reg.typ == "u" and not 0 <= v <= 0xFFFFFFFF:
            raise ValueError("超出 uint32 范围")
        if reg.typ == "i" and not -2**31 <= v < 2**31:
            raise ValueError("超出 int32 范围")
        return v

    def _write_modified(self):
        todo = []
        for i, reg in enumerate(P.REGISTERS):
            t = self.table.item(i, C_NEW).text()
            if not t.strip() or reg.ro:
                continue
            try:
                todo.append((reg, self._parse(reg, t)))
            except ValueError as e:
                QMessageBox.warning(self, "数值错误", f"{reg.name}: {e}")
                return
        if not todo:
            self.lbl.setText("没有已修改的值（在“新值”列双击输入）")
            return
        ids = [r for r, _ in todo if r.rid in (0x07, 0x08)]
        if ids:
            QMessageBox.information(self, "提示", "修改 ID 请使用“修改 ID…”按钮，已跳过 can_master/can_id。")
            todo = [(r, v) for r, v in todo if r.rid not in (0x07, 0x08)]
        txt = "\n".join(f"{r.name}: {fmt(r.typ, self.values.get(r.rid))} → {fmt(r.typ, v)}" for r, v in todo)
        if todo and QMessageBox.question(self, "确认写入", txt) == QMessageBox.Yes:
            for r, v in todo:
                self.worker.write_reg(r.rid, v)

    def _save(self):
        if QMessageBox.question(self, "保存到 Flash", "确认将当前参数保存到 Flash？") == QMessageBox.Yes:
            self.worker.save()

    def _change_id(self):
        w = self.worker
        mid, ok = QInputDialog.getInt(self, "修改 ID", f"新电机 ID（当前 {w.motor_id}）",
                                      w.motor_id, 1, 127)
        if not ok:
            return
        mst, ok = QInputDialog.getInt(self, "修改 ID", f"新 Master ID（当前 {w.master_id}）",
                                      w.master_id, 0, 0x7FE)
        if not ok:
            return
        if mid == mst:
            QMessageBox.warning(self, "修改 ID", "电机 ID 与 Master ID 不能相同")
            return
        if (mid, mst) == (w.motor_id, w.master_id):
            return
        if QMessageBox.question(
                self, "确认修改 ID",
                f"电机 ID {w.motor_id} → {mid}\nMaster ID {w.master_id} → {mst}\n\n"
                "将依次写入并保存到 Flash。完成后需断电重启并用新 ID 重连。") != QMessageBox.Yes:
            return
        self._pending_id_change = {}
        # 先写 master：应答仍从旧 master 发出还是新 master 取决于固件，故不依赖应答；
        # 写完后立即保存。超时会在日志提示，可在重启后用扫描确认。
        if mst != w.master_id:
            self._pending_id_change[0x07] = mst
            w.write_reg(0x07, mst)
        if mid != w.motor_id:
            self._pending_id_change[0x08] = mid
            w.write_reg(0x08, mid)
        w.save()

    def _export(self):
        if not self.values:
            QMessageBox.information(self, "导出", "请先读取参数")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出参数", "hm_params.json", "JSON (*.json)")
        if path:
            data = {P.REG_BY_RID[r].name: v for r, v in sorted(self.values.items())}
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)

    def _import(self):
        path, _ = QFileDialog.getOpenFileName(self, "导入参数", "", "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            QMessageBox.warning(self, "导入失败", str(e))
            return
        n = 0
        for name, v in data.items():
            reg = P.REG_BY_NAME.get(name)
            if reg is None or reg.ro or reg.rid in (0x07, 0x08):
                continue
            self.table.item(self._row_of[reg.rid], C_NEW).setText(fmt(reg.typ, v))
            n += 1
        self.lbl.setText(f"已填入 {n} 个新值（未写入），确认后点“写入已修改”")
