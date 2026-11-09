"""顶部电机卡片：当前电机、状态、使能/失能/清错/设零、发送开关 + 频率。"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QCheckBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton,
                               QSpinBox)


class MotorCard(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        self.worker = None
        self.lbl_title = QLabel("未选择电机")
        self.lbl_title.setStyleSheet("font-weight:bold;font-size:14px")
        self.lbl_proto = QLabel()
        self.lbl_proto.setStyleSheet("color:#555")
        self.lbl_state = QLabel("—")
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setMinimumWidth(150)
        self.lbl_info = QLabel()
        self.lbl_info.setStyleSheet("font-family:Consolas,monospace")

        def btn(t, tip=""):
            b = QPushButton(t)
            b.setFocusPolicy(Qt.NoFocus)
            b.setToolTip(tip)
            return b
        self.btn_en = btn("使能", "达妙/HM 发 FC；大疆为软件开关。会自动打开发送。")
        self.btn_dis = btn("失能", "达妙/HM 发 FD（会刹车）；大疆发零电流并停止输出")
        self.btn_clr = btn("清错")
        self.btn_zero = btn("设零点", "当前位置设为零点（需失能）")
        self.chk_send = QCheckBox("发送")
        self.chk_send.setToolTip("按频率发送：输出中发控制帧，否则发状态查询（大疆无需查询）")
        self.sp_rate = QSpinBox()
        self.sp_rate.setRange(10, 1000)
        self.sp_rate.setValue(200)
        self.sp_rate.setSuffix(" Hz")
        self.lbl_rate = QLabel("—")
        self.lbl_rate.setMinimumWidth(170)

        g = QGridLayout(self)
        g.setContentsMargins(8, 4, 8, 4)
        g.addWidget(self.lbl_title, 0, 0)
        g.addWidget(self.lbl_proto, 1, 0)
        g.addWidget(self.lbl_state, 0, 1)
        g.addWidget(self.lbl_info, 1, 1, 1, 2)
        b = QHBoxLayout()
        for w in (self.btn_en, self.btn_dis, self.btn_clr, self.btn_zero):
            b.addWidget(w)
        g.addLayout(b, 0, 2)
        s = QHBoxLayout()
        s.addWidget(self.chk_send)
        s.addWidget(self.sp_rate)
        s.addWidget(self.lbl_rate)
        g.addLayout(s, 0, 3)
        g.setColumnStretch(0, 1)

        self.btn_en.clicked.connect(lambda: self.worker and self.worker.enable())
        self.btn_dis.clicked.connect(lambda: self.worker and self.worker.disable())
        self.btn_clr.clicked.connect(lambda: self.worker and self.worker.clear_error())
        self.btn_zero.clicked.connect(lambda: self.worker and self.worker.set_zero())
        self.chk_send.clicked.connect(lambda on: self.worker and self.worker.set_sending(on))
        self.sp_rate.valueChanged.connect(self._rate)
        self.set_bound(None)

    def set_bound(self, worker):
        self.worker = worker
        self._rate()
        self.set_driver(None)

    def set_driver(self, d):
        on = self.worker is not None and d is not None
        for w in (self.btn_en, self.btn_dis, self.chk_send, self.sp_rate):
            w.setEnabled(on)
        self.btn_clr.setEnabled(on and d.caps.clear)
        self.btn_zero.setEnabled(on and d.caps.zero)
        if d is None:
            self.lbl_title.setText("未选择电机" if self.worker else "未连接")
            self.lbl_proto.setText("在左侧「总线电机」中扫描或添加，然后选中一台进行调试")
            self.lbl_state.setText("—")
            self.lbl_state.setStyleSheet("")
            self.lbl_info.setText("")
            self.lbl_rate.setText("—")
            return
        self.lbl_title.setText(d.title())
        self.lbl_proto.setText(f"协议：{d.protocol_name}　输出单位：{d.out_name}（{d.out_unit}）"
                               + (f"　{d.note}" if d.note else ""))
        self.btn_en.setText("使能" if d.caps.enable_cmd else "开始输出")
        self.btn_dis.setText("失能" if d.caps.enable_cmd else "停止输出")

    def _rate(self):
        if self.worker:
            self.worker.rate_hz = float(self.sp_rate.value())

    def on_state(self, s: dict):
        d = s["driver"]
        if d is None:
            return
        self.chk_send.setChecked(s["sending"])
        code = s["code"]
        if s["scanning"]:
            txt, st = "扫描中", "background:#90caf9"
        elif not s["link_ok"]:
            txt, st = "无反馈", "background:#9e9e9e;color:white"
        elif d.caps.status_code and d.is_fault(code):
            txt, st = d.code_text(code), "background:#c62828;color:white"
        elif s["active"]:
            txt, st = "已使能" if d.caps.enable_cmd else "输出中", "background:#2e7d32;color:white"
        else:
            txt, st = "失能" if d.caps.enable_cmd else "未输出", "background:#e0e0e0"
        if s["tripped"] and s["active"]:
            txt += " · 零电流锁存"
        if s["running"]:
            txt += " · 测试中"
        self.lbl_state.setText(txt)
        self.lbl_state.setStyleSheet(st + ";font-weight:bold;padding:3px;border-radius:3px")
        if not s["sending"]:
            what = "已关闭"
        elif s["active"]:
            what = "MIT 帧" if s["mode"] == "mit" else f"{d.out_name}指令"
        else:
            what = "状态查询" if d.idle_frames() else "无（等待使能）"
        self.lbl_rate.setText(f"{what} {s['tx_rate']:.0f} Hz")
        lat = s.get("lat") or {}
        lat_t = (f"往返 p50 {lat['p50'] * 1000:.2f} / p95 {lat['p95'] * 1000:.2f} ms"
                 if lat else "")
        self.lbl_info.setText(f"总线 {s['bus']}　{lat_t}")
