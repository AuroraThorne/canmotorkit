"""演示台端到端冒烟测试（无界面显示，无硬件）。

用法：QT_QPA_PLATFORM=offscreen python tools/demo_smoke.py [截图目录]
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.stdout.reconfigure(encoding="utf-8")

from PySide6.QtWidgets import QApplication  # noqa: E402

app = QApplication([])

from canmotorkit.ui.main_window import MainWindow  # noqa: E402

shot_dir = sys.argv[1] if len(sys.argv) > 1 else None


def pump(s, until=None):
    e = time.perf_counter() + s
    while time.perf_counter() < e:
        app.processEvents()
        if until and until():
            return True
        time.sleep(0.005)
    return bool(until and until())


def shot(name):
    if shot_dir:
        w.grab().save(os.path.join(shot_dir, f"demo_{name}.png"))


def check(cond, text):
    print(("  OK   " if cond else "  FAIL ") + text)
    if not cond:
        global failed
        failed = True


failed = False
w = MainWindow()
w.show()
w.conn.select_sim()
w.connect_bus(w.conn.params())
print("1. 扫描总线")
w.motors.btn_scan.click()
pump(6, lambda: w.motors.prog.format().startswith("扫描完成"))
rows = [(e["protocol"], e["model"], e["motor_id"]) for e in w.motors.entries]
print("   ", rows)
check(len(rows) == 5, "发现 5 台（HM、J8009P、3 台大疆）")
shot("scan")

print("2. 选择 HM9816G9，读量程")
i = next(k for k, e in enumerate(w.motors.entries) if e["model"] == "HM9816G9")
w.motors.tbl.selectRow(i)
w.motors.btn_use.click()
pump(1.5, lambda: w.worker.limits_ok)
check(w.worker.limits_ok, "量程已读取：" + w.control.lbl_lim.text().splitlines()[0])
check(w.control.tabs.isTabEnabled(1), "MIT 子页可用")
w.card.chk_send.click()
pump(0.5)
check(w.worker.sending, "发送已打开（未使能时发查询）：" + w.card.lbl_rate.text())

print("3. 寄存器全部读取")
w.tabs.setCurrentWidget(w.regs)
w.regs.read_all()
pump(8, lambda: w.worker._pending is None and not w.worker._reqs)
n_ok = sum(1 for r in range(w.regs.table.rowCount()) if w.regs.table.item(r, 6).text() == "有应答")
n_no = sum(1 for r in range(w.regs.table.rowCount()) if w.regs.table.item(r, 6).text() == "无应答")
check(n_ok > 20 and n_no > 5, f"HM 有应答 {n_ok} 个，无应答（仅达妙）{n_no} 个")
shot("regs")

print("4. 电流模式：应用 0.5 Nm（低于静摩擦）→ 使能 → 急停 → 解除 → 失能")
w.tabs.setCurrentIndex(0)
m = w.bus.motors[0]
w.control.cur.rows["out"].set_value(0.5)
w.control.cur.apply()
w.card.btn_en.click()
pump(0.6)
check(m.enabled and abs(m.cmd.t - 0.5) < 0.03 and m.cmd.kp == 0, f"使能后 t={m.cmd.t:.3f}：" + w.card.lbl_state.text())
w.safety.btn_estop.click()
pump(0.3)
check(m.enabled and abs(m.cmd.t) < 0.03, "急停后仍使能、零扭矩：" + w.safety.lbl_state.text())
w.safety.btn_reset.click()
pump(0.3)
check(abs(m.cmd.t - 0.5) < 0.03, "解除后恢复 0.5")
w.card.btn_dis.click()
pump(0.4)
check(not m.enabled, "已失能：" + w.card.lbl_state.text())

print("5. MIT 子页：位置 0.5 rad")
w.control.tabs.setCurrentIndex(1)
r = w.control.mit.rows
for k, v in (("p", 0.5), ("kp", 20), ("kd", 1)):
    r[k].set_value(v)
w.control.mit.apply()
w.card.btn_en.click()
pump(2.0, lambda: w.worker.last_fb and abs(w.worker.last_fb.pos - 0.5) < 0.03)
check(abs(w.worker.last_fb.pos - 0.5) < 0.05, f"位置 {w.worker.last_fb.pos:.3f}")
shot("mit")
w.card.btn_dis.click()
pump(0.4)
w.control.tabs.setCurrentIndex(0)

print("6. 切换到大疆 M3508，响应测试（步进正弦 + 偏置速度）")
i = next(k for k, e in enumerate(w.motors.entries) if e["model"] == "M3508 (C620)")
w.motors.tbl.selectRow(i)
w.motors.btn_use.click()
pump(0.5)
check(not w.control.tabs.isTabEnabled(1), "大疆电机 MIT 子页禁用")
w.card.sp_rate.setValue(500)
w.safety.sp_vel.setValue(40)
w.safety.sp_out.setValue(5)
w.card.btn_en.click()
pump(0.3)
t = w.test
w.tabs.setCurrentWidget(t)
t.cb_kind.setCurrentIndex(1)
t.sp_amp.setValue(1.5)
t.sp_f0.setValue(2)
t.sp_f1.setValue(20)
t.sp_n.setValue(6)
t.sp_cyc.setValue(4)
t.sp_bias.setValue(8)
t.sp_bkp.setValue(0.3)
t.start()
pump(12, lambda: t.run.done)
print("   " + t.out.toPlainText().replace("\n", "\n   "))
check(t.sugg is not None, "得到模型与 PID 建议")
shot("bode")

print("7. 闭环测试：速度 PI，建议值×50%，阶跃 10 rad/s")
t.btn_use.click()
lp = w.loop
lp.btn_fill.click()
lp.sp_amp.setValue(10)
lp.sp_omax.setValue(5)
lp.start()
pump(5, lambda: lp.run.done)
print("   " + lp.out.toPlainText().replace("\n", "\n   "))
check(lp.run.done and not lp.run.reason, "闭环完成")
shot("loop")

print("8. 超速保护：速度上限 5 rad/s，电流 3 A")
w.safety.sp_vel.setValue(5)
w.tabs.setCurrentIndex(0)
w.control.cur.rows["out"].set_value(3.0)
w.control.cur.apply()
pump(3, lambda: w.worker.safety.tripped)
check(w.worker.safety.tripped, "触发：" + w.safety.lbl_state.text())
check(w.bus.motors[2].cur == 0.0, "电流已归零")
shot("trip")
w.card.btn_dis.click()
pump(0.3)
w.close()
print("结果：", "全部通过" if not failed else "有失败项")
sys.exit(1 if failed else 0)
