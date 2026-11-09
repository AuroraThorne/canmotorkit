"""电机参数页：连接参数 + 合并寄存器表（达妙 ∪ HM，标注来源与实测应答）。

仅支持参数读写的协议（达妙/HM）启用寄存器表；大疆只显示连接参数。
"""
from __future__ import annotations

import json

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QFileDialog, QFormLayout, QGroupBox,
                               QHBoxLayout, QHeaderView, QInputDialog, QLabel, QMessageBox,
                               QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from canmotorkit.drivers import dm_protocol as P

C_RID, C_NAME, C_SRC, C_TYPE, C_VAL, C_NEW, C_RESP, C_DESC = range(8)
HEAD = ["RID", "名称", "来源", "类型", "当前值", "新值（双击编辑）", "实测", "说明"]
TYPE_TXT = {"f": "float", "u": "uint32", "i": "int32"}
MODIFIED = QColor("#fff3c4")
SRC_COLOR = {"both": None, "dm": QColor("#e3f2fd"), "hm": QColor("#f3e5f5")}


def fmt(typ: str, v) -> str:
    if v is None:
        return ""
    return f"{v:.7g}" if typ == "f" else str(int(v))


class RegisterTab(QWidget):
    ids_changed = Signal(int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self.driver = None
        self.blocked = False
        self.values: dict[int, float | int] = {}
        self._row_of = {r.rid: i for i, r in enumerate(P.REGISTERS)}
        self._pending_id_change: dict[int, int] = {}

        # 连接参数
        cg = QGroupBox("连接参数")
        cf = QFormLayout(cg)
        self.lbl_proto = QLabel("—")
        self.lbl_ids = QLabel("—")
        self.lbl_lim = QLabel("—")
        self.lbl_lim.setWordWrap(True)
        self.lbl_extra = QLabel("—")
        self.lbl_extra.setWordWrap(True)
        cf.addRow("协议 / 型号", self.lbl_proto)
        cf.addRow("ID", self.lbl_ids)
        cf.addRow("量程", self.lbl_lim)
        cf.addRow("说明", self.lbl_extra)

        self.table = QTableWidget(len(P.REGISTERS), len(HEAD))
        self.table.setHorizontalHeaderLabels(HEAD)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(C_DESC, QHeaderView.Stretch)
        for i, r in enumerate(P.REGISTERS):
            name = r.name + (f" ({r.alt})" if r.alt and r.alt.lower() != r.name else "")
            cells = [f"0x{r.rid:02X}", name, P.SRC_TEXT[r.src], TYPE_TXT[r.typ] + ("（只读）" if r.ro else ""),
                     "", "", "", r.desc + (f" [{r.unit}]" if r.unit else "")]
            for c, text in enumerate(cells):
                it = QTableWidgetItem(text)
                if c != C_NEW or r.ro:
                    it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                if r.ro and c == C_NEW:
                    it.setBackground(QBrush(QColor("#eeeeee")))
                if c == C_SRC and SRC_COLOR[r.src] is not None:
                    it.setBackground(QBrush(SRC_COLOR[r.src]))
                self.table.setItem(i, c, it)

        self.chk_family = QCheckBox("只显示当前型号家族的寄存器")
        self.btn_read_all = QPushButton("全部读取")
        self.btn_read_all.setToolTip("读取表中所有可见寄存器；无应答的在「实测」列标为无应答")
        self.btn_read_sel = QPushButton("读取选中")
        self.btn_write = QPushButton("写入已修改")
        self.btn_save = QPushButton("保存到 Flash")
        self.btn_save.setStyleSheet("QPushButton{font-weight:bold}")
        self.btn_save.setToolTip("AA 01；仅失能时生效；Flash 擦写约 1 万次，请勿频繁保存")
        self.btn_change_id = QPushButton("修改 ID…")
        self.btn_limits = QPushButton("重读量程")
        self.btn_export = QPushButton("导出 JSON")
        self.btn_import = QPushButton("导入 JSON")
        bar = QHBoxLayout()
        for b in (self.btn_read_all, self.btn_read_sel, self.btn_write, self.btn_save,
                  self.btn_change_id, self.btn_limits, self.btn_export, self.btn_import):
            bar.addWidget(b)
        bar.addStretch(1)
        bar.addWidget(self.chk_family)
        note = QLabel("写入立即生效但不存储（达妙文档），需「保存到 Flash」才掉电保持；HM 是否一致待实测。"
                      "控制模式、波特率、标定值等在本软件中只读。蓝色=仅达妙，紫色=仅 HM。")
        note.setWordWrap(True)
        note.setStyleSheet("color:#795548")
        self.lbl = QLabel()
        self.lbl.setStyleSheet("color:gray")
        self.reg_box = QWidget()
        rl = QVBoxLayout(self.reg_box)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.addLayout(bar)
        rl.addWidget(note)
        rl.addWidget(self.table, 1)
        rl.addWidget(self.lbl)
        self.lbl_noreg = QLabel("当前协议不支持参数读写（大疆电调的参数需使用 RoboMaster Assistant）。")
        self.lbl_noreg.setStyleSheet("color:gray;padding:20px")

        lay = QVBoxLayout(self)
        lay.addWidget(cg)
        lay.addWidget(self.reg_box, 1)
        lay.addWidget(self.lbl_noreg)

        self.btn_read_all.clicked.connect(self.read_all)
        self.btn_read_sel.clicked.connect(self._read_sel)
        self.btn_write.clicked.connect(self._write_modified)
        self.btn_save.clicked.connect(self._save)
        self.btn_change_id.clicked.connect(self._change_id)
        self.btn_limits.clicked.connect(lambda: self.worker and self.worker.reload_limits())
        self.btn_export.clicked.connect(self._export)
        self.btn_import.clicked.connect(self._import)
        self.chk_family.toggled.connect(self._filter_rows)
        self.table.itemChanged.connect(self._on_item_changed)
        self.set_bound(None)

    # ---------- 外部 ----------
    def set_bound(self, worker):
        self.worker = worker
        self.set_driver(None)

    def set_driver(self, d):
        self.driver = d
        self.values.clear()
        self.table.blockSignals(True)
        for i in range(self.table.rowCount()):
            for c in (C_VAL, C_NEW, C_RESP):
                it = self.table.item(i, c)
                it.setText("")
                if c != C_NEW or not P.REGISTERS[i].ro:
                    it.setBackground(QBrush())
        self.table.blockSignals(False)
        has_reg = d is not None and d.caps.registers
        self.reg_box.setVisible(has_reg or d is None)
        self.lbl_noreg.setVisible(d is not None and not has_reg)
        for b in (self.btn_read_all, self.btn_read_sel, self.btn_write, self.btn_save,
                  self.btn_change_id, self.btn_limits):
            b.setEnabled(has_reg)
        if d is None:
            for l in (self.lbl_proto, self.lbl_ids, self.lbl_lim, self.lbl_extra):
                l.setText("—")
            return
        self.lbl_proto.setText(f"{d.protocol_name} · {d.model}")
        if has_reg:
            self.lbl_ids.setText(f"电机 ID {d.motor_id}（控制帧 ID），Master 0x{d.master_id:X}（反馈帧 ID）")
            self.lbl_extra.setText(d.note or "—")
        else:
            cid, slot = d.ctrl
            self.lbl_ids.setText(f"电调 ID {d.motor_id}：反馈 0x{d.fb_id:03X}，控制帧 0x{cid:03X} 第 {slot + 1} 槽")
            m = d.m
            self.lbl_lim.setText(f"电流 ±{m.amp_max:g} A（原始 ±{m.raw_max}），减速比 {m.gr:.6g}（位置/速度换算到输出轴）")
            self.lbl_extra.setText((d.note + "；" if d.note else "") +
                                   "同一控制帧其余槽位填 0，会把同组其它电机电流置 0")
        self._filter_rows()

    def set_limits(self, lim: P.Limits, from_motor: bool):
        if self.driver is None or not self.driver.caps.registers:
            return
        src = "来自电机" if from_motor else "未读取（默认值，禁止使能）"
        self.lbl_lim.setText(f"p ±{lim.p_max:.7g} rad，v ±{lim.v_max:.7g} rad/s，t ±{lim.t_max:.7g} Nm，"
                             f"kp 0~{lim.kp_max:g}，kd 0~{lim.kd_max:g}（{src}）")
        self.lbl_lim.setStyleSheet("" if from_motor else "color:#c62828")

    def on_state(self, s: dict):
        b = s["blocked"]
        if b != self.blocked:
            self.blocked = b
            ok = self.driver is not None and self.driver.caps.registers and not b
            for w in (self.btn_save, self.btn_change_id, self.btn_write, self.btn_limits):
                w.setEnabled(ok)
                w.setToolTip("使能中不可用，请先失能" if b else "")
        if s.get("queue"):
            self.lbl.setText(f"队列中请求：{s['queue']}")

    def read_all(self):
        if not self.worker:
            return
        n = 0
        for i, r in enumerate(P.REGISTERS):
            if not self.table.isRowHidden(i):
                self.worker.read_reg(r.rid)
                n += 1
        self.lbl.setText(f"正在读取 {n} 个寄存器…")

    def on_reg_done(self, kind: str, rid: int, value, err: str):
        if kind == "save":
            if err:
                QMessageBox.warning(self, "保存失败", err)
                return
            self.lbl.setText("✔ 参数已保存到 Flash")
            if self._pending_id_change and self.driver is not None:
                mid = self._pending_id_change.get(0x08, self.driver.motor_id)
                mst = self._pending_id_change.get(0x07, self.driver.master_id)
                self._pending_id_change = {}
                QMessageBox.information(self, "ID 已修改", f"新电机 ID = {mid}，新 Master = 0x{mst:X}。\n"
                                        "请断电重启后重新扫描。")
                self.ids_changed.emit(mid, mst)
            return
        row = self._row_of.get(rid)
        if row is None:
            return
        reg = P.REG_BY_RID[rid]
        resp = self.table.item(row, C_RESP)
        self.table.blockSignals(True)
        if err:
            if "超时" in err:
                resp.setText("无应答")
                resp.setForeground(QBrush(QColor("#c62828")))
            self.table.blockSignals(False)
            self.lbl.setText(f"✖ {kind} 0x{rid:02X} {reg.name}：{err}")
            if kind == "write":
                self._pending_id_change.pop(rid, None)
            return
        resp.setText("有应答")
        resp.setForeground(QBrush(QColor("#2e7d32")))
        self.values[rid] = value
        self.table.item(row, C_VAL).setText(fmt(reg.typ, value))
        if kind == "write":
            it = self.table.item(row, C_NEW)
            it.setText("")
            it.setBackground(QBrush())
        self.table.blockSignals(False)
        self.lbl.setText(f"✔ {kind} 0x{rid:02X} {reg.name} = {fmt(reg.typ, value)}")

    # ---------- 内部 ----------
    def _filter_rows(self):
        fam = getattr(self.driver, "family", "both")
        only = self.chk_family.isChecked() and fam in ("dm", "hm")
        for i, r in enumerate(P.REGISTERS):
            self.table.setRowHidden(i, only and r.src not in ("both", fam))

    def _on_item_changed(self, it: QTableWidgetItem):
        if it.column() == C_NEW:
            it.setBackground(QBrush(MODIFIED) if it.text().strip() else QBrush())

    def _read_sel(self):
        for r in sorted({i.row() for i in self.table.selectedIndexes()}):
            self.worker.read_reg(P.REGISTERS[r].rid)

    @staticmethod
    def _parse(reg: P.Register, text: str):
        text = text.strip()
        if reg.typ == "f":
            return float(text)
        v = int(text, 0)
        if reg.typ == "u" and not 0 <= v <= 0xFFFFFFFF:
            raise ValueError("超出 uint32 范围")
        if reg.typ == "i" and not -2 ** 31 <= v < 2 ** 31:
            raise ValueError("超出 int32 范围")
        return v

    def _write_modified(self):
        todo = []
        for i, reg in enumerate(P.REGISTERS):
            t = self.table.item(i, C_NEW).text()
            if not t.strip() or reg.ro:
                continue
            if reg.rid in (0x07, 0x08):
                QMessageBox.information(self, "提示", "修改 ID 请使用“修改 ID…”按钮，已跳过。")
                continue
            try:
                todo.append((reg, self._parse(reg, t)))
            except ValueError as e:
                QMessageBox.warning(self, "数值错误", f"{reg.name}: {e}")
                return
        if not todo:
            self.lbl.setText("没有已修改的值（在“新值”列双击输入）")
            return
        txt = "\n".join(f"{r.name}: {fmt(r.typ, self.values.get(r.rid))} → {fmt(r.typ, v)}" for r, v in todo)
        if QMessageBox.question(self, "确认写入", txt) == QMessageBox.Yes:
            for r, v in todo:
                self.worker.write_reg(r.rid, v)

    def _save(self):
        if QMessageBox.question(self, "保存到 Flash", "确认将当前参数保存到 Flash？") == QMessageBox.Yes:
            self.worker.save()

    def _change_id(self):
        d = self.driver
        mid, ok = QInputDialog.getInt(self, "修改 ID", f"新电机 ID（当前 {d.motor_id}）", d.motor_id, 1, 127)
        if not ok:
            return
        mst, ok = QInputDialog.getInt(self, "修改 ID", f"新 Master ID（当前 0x{d.master_id:X}）",
                                      d.master_id, 0, 0x7FE)
        if not ok or (mid, mst) == (d.motor_id, d.master_id):
            return
        if mid == mst:
            QMessageBox.warning(self, "修改 ID", "电机 ID 与 Master ID 不能相同")
            return
        if QMessageBox.question(self, "确认修改 ID",
                                f"电机 ID {d.motor_id} → {mid}\nMaster 0x{d.master_id:X} → 0x{mst:X}\n\n"
                                "将依次写入并保存到 Flash，完成后需断电重启并重新扫描。") != QMessageBox.Yes:
            return
        self._pending_id_change = {}
        # ID 寄存器在表中不是只读，经专用流程写入
        if mst != d.master_id:
            self._pending_id_change[0x07] = mst
            self.worker.write_reg(0x07, mst)
        if mid != d.motor_id:
            self._pending_id_change[0x08] = mid
            self.worker.write_reg(0x08, mid)
        self.worker.save()

    def _export(self):
        if not self.values:
            QMessageBox.information(self, "导出", "请先读取参数")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出参数", "motor_params.json", "JSON (*.json)")
        if path:
            data = {"model": self.driver.model if self.driver else "",
                    "registers": {P.REG_BY_RID[r].name: v for r, v in sorted(self.values.items())}}
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
        regs = data.get("registers", data)
        n = 0
        for name, v in regs.items():
            reg = P.REG_BY_NAME.get(name)
            if reg is None or reg.ro or reg.rid in (0x07, 0x08):
                continue
            self.table.item(self._row_of[reg.rid], C_NEW).setText(fmt(reg.typ, v))
            n += 1
        self.lbl.setText(f"已填入 {n} 个新值（未写入），确认后点“写入已修改”")
