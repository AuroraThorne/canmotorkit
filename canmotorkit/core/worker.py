"""后台通信线程（与协议无关，经 Driver 翻译）。

约定（AGENTS.md）：
- 「发送」开关 + 频率：电机输出中发控制帧（电流/扭矩或 MIT），否则发驱动的 idle 帧（达妙/HM 为 0xCC）。
  点使能会自动打开发送。
- 「输出中」：有使能命令的协议以电机反馈为准；大疆等无使能命令的协议为软件开关。
- 急停/超限：零电流并锁存，不失能。
- 达妙/HM：量程必须从电机读取后才允许使能；使能中不发 0x7FF 配置帧。
- 测试/闭环由 Run 对象在本线程内按发送频率计算输出，并在本线程内记录数据。
"""
from __future__ import annotations

import collections
import ctypes
import gc
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass, replace

from PySide6.QtCore import QThread, Signal

from ..control.safety import SafetyLimits, Supervisor
from ..drivers import dji, dm
from ..drivers import dm_protocol as P
from ..drivers.base import Driver, Feedback
from .bus import BusError, CanBus, Frame

LIMIT_RIDS = (0x15, 0x16, 0x17)
NAN = float("nan")


@dataclass
class Sample:
    t: float
    out: float              # 当时下发的电流/扭矩指令（未输出时 NaN）
    fb: Feedback
    pos_mt: float           # 多圈累计位置
    active: bool            # 电机输出中
    ref: float = NAN        # 测试激励/闭环参考值
    mit: P.MitCommand | None = None


@dataclass
class _Req:
    kind: str               # read / write / save
    rid: int
    typ: str
    frame: bytes
    tries: int = 0
    deadline: float = 0.0


class Worker(QThread):
    samples = Signal(object)            # list[Sample]
    frames = Signal(object)             # list[Frame]
    reg_done = Signal(str, int, object, str)
    scan_progress = Signal(str, int, int)   # 阶段, i, n
    scan_done = Signal(object)          # list[dict]
    state = Signal(object)
    message = Signal(str)
    limits_changed = Signal(object, bool)
    run_finished = Signal(object)       # Run

    EMIT_HZ = 30

    def __init__(self, bus: CanBus, driver: Driver | None = None):
        super().__init__()
        self.bus = bus
        self.driver: Driver | None = None
        self.rate_hz = 200.0
        self.sending = False
        self.mode = "current"           # current / mit
        self.req_timeout = 0.1
        self.req_retries = 2
        self.log_enabled = True
        self.safety = Supervisor()

        self._lock = threading.Lock()
        self._target_out = 0.0
        self._target_mit = P.MitCommand()
        self._last_out = NAN
        self._last_mit: P.MitCommand | None = None
        self._sw_en = False
        self._ops: queue.Queue = queue.Queue()
        self._reqs: collections.deque[_Req] = collections.deque()
        self._pending: _Req | None = None
        self._lim_vals: dict[int, float] = {}
        self._scan: dict | None = None
        self._run_obj = None
        self._gc_was = True
        self._running = False
        self._frames: list[Frame] = []
        self._last_err_t = 0.0
        self._trip_reported = False
        self._tx_t: float | None = None
        self.latencies: collections.deque[float] = collections.deque(maxlen=5000)
        self.motor_code = -1
        self.last_fb: Feedback | None = None
        self.last_fb_t = 0.0
        if driver is not None:
            self._apply_driver(driver)

    # ================= 公开接口（线程安全） =================
    def _post(self, fn, *a):
        self._ops.put((fn, a))

    @property
    def active(self) -> bool:
        """电机是否处于输出状态（达妙/HM 以反馈为准，大疆为软件使能）。"""
        d = self.driver
        if d is None:
            return False
        if d.caps.enable_cmd:
            return self.motor_code == P.CODE_ENABLED
        return self._sw_en

    @property
    def enabled(self) -> bool:
        return self.active

    @property
    def limits_ok(self) -> bool:
        return self.driver is not None and self.driver.ready

    @property
    def lim(self) -> P.Limits:
        d = self.driver
        return d.lim if isinstance(d, dm.DmDriver) else P.DEFAULT_LIMITS

    @property
    def motor_id(self) -> int:
        return self.driver.motor_id if self.driver else -1

    @property
    def master_id(self) -> int:
        return getattr(self.driver, "master_id", -1)

    def set_target_out(self, v: float):
        with self._lock:
            self._target_out = float(v)

    def set_target_mit(self, cmd: P.MitCommand):
        with self._lock:
            self._target_mit = replace(cmd)

    set_target = set_target_mit

    def get_target(self) -> P.MitCommand:
        with self._lock:
            return replace(self._target_mit)

    def get_target_out(self) -> float:
        with self._lock:
            return self._target_out

    def set_mode(self, mode: str):
        self._post(self._do_set_mode, mode)

    def set_driver(self, driver: Driver | None):
        self._post(self._do_set_driver, driver)

    def set_sending(self, on: bool):
        self._post(self._do_set_sending, bool(on))

    def reload_limits(self):
        self._post(self._do_reload_limits)

    def enable(self):
        self._post(self._do_enable)

    def disable(self):
        self._post(self._do_disable)

    def estop(self, reason: str = "用户急停"):
        self._post(self._do_estop, reason)

    def reset_safety(self):
        self._post(self._do_reset_safety)

    def set_safety_limits(self, lim: SafetyLimits):
        self._post(setattr, self.safety, "limits", lim)

    def zero_safety_position(self):
        self._post(self.safety.zero_position)

    def clear_error(self):
        self._post(self._do_frames, "clear_frames", "已发送清错")

    def set_zero(self):
        self._post(self._do_zero)

    def read_reg(self, rid: int):
        self._post(self._do_read, rid)

    def write_reg(self, rid: int, value):
        self._post(self._do_write, rid, value)

    def save(self):
        self._post(self._do_save)

    def start_scan(self, dm_ids=range(1, 128), window: float = 0.02, listen: float = 0.3,
                   identify: bool = True):
        self._post(self._do_scan, list(dm_ids), window, listen, identify)

    def stop_scan(self):
        self._post(self._do_stop_scan)

    def send_raw(self, can_id: int, data: bytes):
        self._post(self._send, can_id, bytes(data))

    def start_run(self, run):
        self._post(self._do_start_run, run)

    def stop_run(self):
        self._post(self._abort_run, "用户停止")

    def stop(self):
        self._post(self._do_shutdown)

    # ================= 内部 =================
    def _msg(self, s: str):
        self.message.emit(s)

    def _err(self, s: str):
        now = time.perf_counter()
        if now - self._last_err_t > 1.0:
            self._last_err_t = now
            self.message.emit("错误: " + s)

    def _send(self, can_id: int, data: bytes) -> bool:
        try:
            self.bus.send(can_id, data)
        except BusError as e:
            self._err(str(e))
            return False
        if self.log_enabled:
            self._frames.append(Frame(time.perf_counter(), can_id, bytes(data), True))
        return True

    def _send_all(self, frames) -> bool:
        ok = True
        for cid, d in frames:
            ok = self._send(cid, d) and ok
        return ok

    def _is_dm(self) -> bool:
        return isinstance(self.driver, dm.DmDriver)

    def _config_blocked(self) -> bool:
        return self._is_dm() and self.motor_code == P.CODE_ENABLED

    BLOCK_TIP = "电机使能中，请先失能"

    # ---------- 电机选择 ----------
    def _apply_driver(self, driver: Driver | None):
        self.driver = driver
        self.motor_code = -1
        self.last_fb = None
        self._sw_en = False
        self._last_out = NAN
        self._reqs.clear()
        self._pending = None
        self.latencies.clear()
        self.safety.mt.reset()
        if driver is None:
            return
        self.safety.set_wrap_span(driver.wrap_span)
        if not driver.caps.mit:
            self.mode = "current"
        if self._is_dm():
            driver.invalidate_limits()
            self._queue_limit_reads()

    def _do_set_driver(self, driver: Driver | None):
        if self.active:
            self._msg("请先失能当前电机再切换")
            return
        self._abort_run("切换电机")
        self._apply_driver(driver)
        self.limits_changed.emit(self.lim, self.limits_ok)
        self._msg("已选择电机：" + (driver.title() if driver else "无"))

    def _do_set_mode(self, mode: str):
        if mode == "mit" and not (self.driver and self.driver.caps.mit):
            self._msg("当前电机不支持 MIT")
            return
        if mode != self.mode:
            self._abort_run("切换控制模式")
            self.mode = mode

    def _do_set_sending(self, on: bool):
        self.sending = on
        if not on and self.active:
            self._msg("发送已关闭，电机仍处于输出状态（可能保持最后指令或触发超时保护）")

    # ---------- 量程 ----------
    def _queue_limit_reads(self):
        self._lim_vals = {}
        self._reqs = collections.deque(r for r in self._reqs if r.rid not in LIMIT_RIDS)
        for rid in LIMIT_RIDS:
            self._reqs.append(_Req("read", rid, "f", P.read_reg_frame(self.motor_id, rid)))

    def _do_reload_limits(self):
        if not self._is_dm():
            return
        if self._config_blocked():
            self._msg("不能重读量程：" + self.BLOCK_TIP)
            return
        self.driver.invalidate_limits()
        self._queue_limit_reads()
        self.limits_changed.emit(self.lim, False)

    def _on_limit_value(self, rid: int, val):
        v = float(val)
        if not math.isfinite(v) or v <= 0:
            self._msg(f"量程寄存器 0x{rid:02X} 值无效: {v}")
            return
        self._lim_vals[rid] = v
        if all(r in self._lim_vals for r in LIMIT_RIDS):
            was = self.driver.ready
            self.driver.set_limits(self._lim_vals[0x15], self._lim_vals[0x16], self._lim_vals[0x17])
            self.safety.set_wrap_span(self.driver.wrap_span)
            self.limits_changed.emit(self.lim, True)
            if not was:
                L = self.lim
                self._msg(f"已读取量程: p±{L.p_max:.7g} v±{L.v_max:.7g} t±{L.t_max:.7g}")

    # ---------- 使能 ----------
    def _do_enable(self):
        d = self.driver
        if d is None:
            self._msg("未选择电机")
            return
        if self._scan is not None:
            self._msg("扫描中，不能使能")
            return
        if not d.ready:
            self._msg("尚未从电机读到量程，禁止使能（可在参数页重读）")
            return
        if not self.sending:
            self.sending = True
            self._msg("已自动打开发送")
        if d.caps.enable_cmd:
            if self._send_all(d.enable_frames()):
                self._msg("已发送使能")
        else:
            self._sw_en = True
            self._msg("已使能（软件开关，开始发送电流指令）")

    def _do_disable(self):
        d = self.driver
        if d is None:
            return
        self._abort_run("失能")
        if d.caps.enable_cmd:
            if self._send_all(d.disable_frames()) and self.motor_code == P.CODE_ENABLED:
                self.motor_code = P.CODE_DISABLED      # 立即改发 idle 帧，以后续反馈为准
            self._msg("已发送失能")
        else:
            self._sw_en = False
            self._send_all(d.stop_frames())
            self._msg("已失能（发送零电流）")

    def _do_estop(self, reason: str):
        self._abort_run(reason)
        self.safety.trip(reason)
        if self.driver is not None:
            self._send_all(self.driver.stop_frames())
        self._msg(f"急停（{reason}）：已发送零电流并锁存，电机未失能")

    def _do_reset_safety(self):
        if self.safety.reset():
            self._trip_reported = False
            self._msg("已解除急停/超限锁存")
        else:
            self._msg("不能解除：" + self.safety.reason)

    def _do_frames(self, name: str, text: str):
        if self.driver is not None:
            fr = getattr(self.driver, name)()
            if fr and self._send_all(fr):
                self._msg(text)

    def _do_zero(self):
        if self._config_blocked():
            self._msg("不能设零点：" + self.BLOCK_TIP)
            return
        self._do_frames("zero_frames", "已发送设零点")
        self.safety.mt.reset()

    def _do_shutdown(self):
        self._abort_run("退出")
        if self.driver is not None and self.active:
            self._send_all(self.driver.stop_frames())
            if self.driver.caps.enable_cmd:
                self._send_all(self.driver.disable_frames())
        self._running = False

    def _on_trip(self):
        if not self._trip_reported:
            self._trip_reported = True
            self._abort_run("安全限制")
            self._msg("安全限制触发零电流：" + self.safety.reason)

    # ---------- 测试 / 闭环 ----------
    def _do_start_run(self, run):
        why = ""
        if not self.active:
            why = "请先使能电机"
        elif self.safety.tripped:
            why = "安全锁存中，请先解除"
        if why:
            self._msg(why)
            run.finish(why)
            self.run_finished.emit(run)
            return
        self._abort_run("启动新测试")
        fb = self.last_fb
        run.start(time.perf_counter(), self.safety.pos, fb.vel if fb else 0.0)
        self._run_obj = run
        self._gc_was = gc.isenabled()
        gc.disable()
        self._msg(f"开始：{run.title}")

    def _finish_run(self, run, reason: str = ""):
        if self._run_obj is not run:
            return
        self._run_obj = None
        if self._gc_was:
            gc.enable()
        run.finish(reason)
        if self.driver is not None and self.active:
            self._send_all(self.driver.stop_frames())
            self.set_target_out(0.0)
        self._msg(f"结束：{run.title}" + (f"（{reason}）" if reason else "") + "，输出已归零")
        self.run_finished.emit(run)

    def _abort_run(self, reason: str):
        if self._run_obj is not None:
            self._finish_run(self._run_obj, reason)

    # ---------- 配置 ----------
    def _do_read(self, rid: int):
        if not self._is_dm():
            return
        reg = P.REG_BY_RID.get(rid)
        if any(r.kind == "read" and r.rid == rid for r in self._reqs):
            return
        self._reqs.append(_Req("read", rid, reg.typ if reg else "u",
                               P.read_reg_frame(self.motor_id, rid)))

    def _do_write(self, rid: int, value):
        if not self._is_dm():
            return
        reg = P.REG_BY_RID.get(rid)
        typ = reg.typ if reg else "u"
        if reg and reg.ro:
            self.reg_done.emit("write", rid, None, "本软件中为只读寄存器")
            return
        if self._config_blocked():
            self.reg_done.emit("write", rid, None, self.BLOCK_TIP)
            return
        try:
            frame = P.write_reg_frame(self.motor_id, rid, typ, value)
        except Exception as e:
            self.reg_done.emit("write", rid, None, f"数值无效: {e}")
            return
        self._reqs.append(_Req("write", rid, typ, frame))

    def _do_save(self):
        if not self._is_dm():
            return
        if self._config_blocked():
            self.reg_done.emit("save", 0, None, "保存参数只在失能模式下生效：" + self.BLOCK_TIP)
            return
        self._reqs.append(_Req("save", 0, "u", P.save_frame(self.motor_id)))

    def _finish_req(self, d: bytes):
        p = self._pending
        self._pending = None
        r = P.parse_config_reply(d)
        val = True if p.kind == "save" else P.decode_value(p.typ, r.raw4)
        if p.kind == "read" and p.rid in LIMIT_RIDS:
            self._on_limit_value(p.rid, val)
        self.reg_done.emit(p.kind, p.rid, val, "")

    def _service_config(self, now: float):
        p = self._pending
        if p is not None:
            if now >= p.deadline:
                if p.tries <= self.req_retries:
                    p.tries += 1
                    p.deadline = now + self.req_timeout
                    self._send(P.CONFIG_ID, p.frame)
                else:
                    self._pending = None
                    if p.kind == "read" and p.rid in LIMIT_RIDS:
                        self._msg(f"量程寄存器 0x{p.rid:02X} 读取超时，禁止使能（可在参数页重读）")
                    self.reg_done.emit(p.kind, p.rid, None, "超时无应答")
            return
        if self._reqs and self._scan is None and not self._config_blocked():
            p = self._reqs.popleft()
            p.tries = 1
            p.deadline = now + self.req_timeout
            self._pending = p
            self._send(P.CONFIG_ID, p.frame)

    # ---------- 扫描 ----------
    def _do_scan(self, ids: list[int], window: float, listen: float, identify: bool):
        if self.active:
            self._msg("不能扫描：请先失能")
            self.scan_done.emit([])
            return
        self._abort_run("扫描")
        self._scan = {"phase": "listen", "until": time.perf_counter() + listen, "listen": listen,
                      "dji": {}, "ids": ids, "i": -1, "cur": None, "deadline": 0.0,
                      "window": window, "dm": {}, "identify": identify, "probe": [], "pi": -1,
                      "preq": None, "pres": {}}
        self.scan_progress.emit("监听大疆反馈", 0, 1)

    def _do_stop_scan(self):
        if self._scan is not None:
            self._finish_scan()

    def _scan_rx(self, f: Frame):
        s = self._scan
        d = f.data
        if s["phase"] == "listen":
            if 0x201 <= f.can_id <= 0x20B and len(d) >= 7:
                e = s["dji"].setdefault(f.can_id, {"n": 0, "data": d})
                e["n"] += 1
                e["data"] = d
        elif s["phase"] == "dm":
            cur = s["cur"]
            if (cur is not None and cur not in s["dm"] and len(d) == 8 and f.can_id != P.CONFIG_ID
                    and not (0x201 <= f.can_id <= 0x20B) and d[2] not in P.CONFIG_CMDS):
                s["dm"][cur] = (f.can_id, bytes(d))
        elif s["phase"] == "probe" and s["preq"] is not None:
            if P.matches_request(d, s["preq"]):
                s["pres"][s["probe"][s["pi"]]] = True
                s["deadline"] = 0.0

    def _service_scan(self, now: float):
        s = self._scan
        if s is None or self._pending is not None:
            return
        if s["phase"] == "listen":
            if now < s["until"]:
                return
            s["phase"] = "dm"
        if s["phase"] == "dm":
            if now < s["deadline"]:
                return
            s["i"] += 1
            if s["i"] < len(s["ids"]):
                cur = s["ids"][s["i"]]
                s["cur"] = cur
                s["deadline"] = now + s["window"]
                self._send(P.CONFIG_ID, P.query_state_frame(cur))
                if s["i"] % 4 == 0:
                    self.scan_progress.emit("查询达妙/HM", s["i"], len(s["ids"]))
                return
            s["cur"] = None
            s["phase"] = "probe"
            if s["identify"]:
                s["probe"] = [(mid, rid) for mid in sorted(s["dm"]) for rid in (0x0A, 0x3C)]
            s["deadline"] = 0.0
        if s["phase"] == "probe":
            if now < s["deadline"]:
                return
            s["pi"] += 1
            if s["pi"] < len(s["probe"]):
                mid, rid = s["probe"][s["pi"]]
                s["preq"] = P.read_reg_frame(mid, rid)
                s["deadline"] = now + 0.06
                self._send(P.CONFIG_ID, s["preq"])
                self.scan_progress.emit("识别型号", s["pi"], len(s["probe"]))
                return
            self._finish_scan()

    def _finish_scan(self):
        s = self._scan
        self._scan = None
        found = []
        min_n = max(5, int(s["listen"] * 200))      # 大疆约 1 kHz 广播
        for fb_id, e in sorted(s["dji"].items()):
            cands = dji.guess_models(fb_id, e["data"])
            if e["n"] < min_n or not cands:
                continue
            rate = e["n"] / s["listen"]
            found.append({"protocol": "dji", "model": cands[0][0], "motor_id": cands[0][1],
                          "master_id": None, "fb_id": fb_id, "candidates": cands,
                          "evidence": f"反馈 0x{fb_id:03X} 广播约 {rate:.0f} Hz" +
                          ("；与其它型号 ID 重叠，请确认" if len(cands) > 1 else "")})
        for mid, (cid, d) in sorted(s["dm"].items()):
            has_mode = s["pres"].get((mid, 0x0A), False)
            has_vbus = s["pres"].get((mid, 0x3C), False)
            if not s["identify"]:
                model, ev = "达妙/HM 通用", "0xCC 有应答"
            elif has_mode:
                model, ev = "DM-J8009P", "有 CTRL_MODE(0x0A) → 达妙（型号请确认）"
            elif has_vbus:
                model, ev = "HM9816G9", "有 v_bus(0x3C)、无 CTRL_MODE → HM"
            else:
                model, ev = "达妙/HM 通用", "特征不足"
            found.append({"protocol": "dm", "model": model, "motor_id": mid, "master_id": cid,
                          "fb_id": cid, "candidates": [(model, mid)],
                          "evidence": f"0xCC 应答 ID 0x{cid:X}；{ev}", "code": d[0] >> 4})
        self.scan_progress.emit("完成", 1, 1)
        self.scan_done.emit(found)

    # ---------- 接收 ----------
    def _on_rx(self, f: Frame) -> Sample | None:
        d = f.data
        p = self._pending
        if p and self._is_dm() and f.can_id == self.master_id and P.matches_request(d, p.frame):
            self._finish_req(d)
            return None
        if self._scan is not None:
            self._scan_rx(f)
            if self._scan["phase"] != "listen":
                return None
        drv = self.driver
        if drv is None or not drv.accepts(f.can_id, d):
            return None
        fb = drv.parse(f.can_id, d)
        if fb is None:
            return None
        if self._tx_t is not None and drv.caps.enable_cmd:
            self.latencies.append(f.t - self._tx_t)
            self._tx_t = None
        if drv.caps.status_code:
            if fb.code != self.motor_code and self.motor_code != -1 and drv.is_fault(fb.code):
                self._msg(f"电机报告 {drv.code_text(fb.code)}")
            if fb.code != P.CODE_ENABLED and self._run_obj is not None:
                self._abort_run("电机已退出使能")
            self.motor_code = fb.code
        self.last_fb = fb
        self.last_fb_t = f.t
        self.safety.on_feedback(f.t, fb.pos, fb.vel, fb.max_temp())
        if self.safety.tripped and self.active:
            self._on_trip()
        elif not self.safety.tripped:
            self._trip_reported = False
        run = self._run_obj
        if run is not None:
            run.record(f.t, self._last_out, fb, self.safety.pos)
        act = self.active
        return Sample(f.t, self._last_out if act else NAN, fb, self.safety.pos, act,
                      run.ref if run is not None else NAN,
                      self._last_mit if act and self.mode == "mit" else None)

    # ---------- 发送 ----------
    def _tick(self, now: float) -> bool:
        d = self.driver
        if d is None:
            return False
        if not self.active:
            self._last_out = NAN
            if self._scan is None and self._pending is None:
                fr = d.idle_frames()
                if fr and self._send_all(fr):
                    self._tx_t = now
                    return True
            return False
        mit = None
        if self.safety.tripped or not d.ready:
            fr, out = d.stop_frames(), 0.0
        else:
            run = self._run_obj
            if run is not None:
                u = run.output(now, self.last_fb, self.safety.pos)
                if u is None:
                    self._finish_run(run)
                    u = 0.0
                out = self.safety.filter_output(now, u)
                fr = d.output_frames(out)
            elif self.mode == "mit" and d.caps.mit:
                cmd = self.get_target()
                cmd.t = self.safety.filter_output(now, cmd.t)
                L = self.lim
                mit = P.MitCommand(P.clamp(cmd.p, L.p_min, L.p_max), P.clamp(cmd.v, L.v_min, L.v_max),
                                   P.clamp(cmd.kp, L.kp_min, L.kp_max),
                                   P.clamp(cmd.kd, L.kd_min, L.kd_max), P.clamp(cmd.t, L.t_min, L.t_max))
                out = mit.t
                fr = d.mit_frames(mit)
            else:
                out = self.safety.filter_output(now, self.get_target_out())
                fr = d.output_frames(out)
            if self.safety.tripped:
                self._on_trip()
                fr, out, mit = d.stop_frames(), 0.0, None
        if self._send_all(fr):
            self._last_out = out
            self._last_mit = mit
            self._tx_t = now
            return True
        return False

    def latency_stats(self) -> dict:
        xs = sorted(self.latencies)
        if not xs:
            return {}
        n = len(xs)
        return {"n": n, "p50": xs[n // 2], "p95": xs[min(n - 1, int(n * 0.95))], "max": xs[-1]}

    # ---------- 主循环 ----------
    def run(self):
        winmm = None
        if sys.platform == "win32":
            try:
                winmm = ctypes.windll.winmm
                winmm.timeBeginPeriod(1)
            except Exception:
                winmm = None
        self._running = True
        now = time.perf_counter()
        next_tx = next_emit = now
        next_bus_check = now + 1.0
        rate_t0, rate_n, tx_rate = now, 0, 0.0
        bus_text = "OK"
        samples: list[Sample] = []
        try:
            while self._running:
                while True:
                    try:
                        fn, a = self._ops.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        fn(*a)
                    except Exception as e:
                        self._err(f"{type(e).__name__}: {e}")
                try:
                    rx = self.bus.recv_all()
                except Exception as e:
                    self._err(f"接收失败: {e}")
                    rx = []
                for f in rx:
                    if self.log_enabled:
                        self._frames.append(f)
                    smp = self._on_rx(f)
                    if smp is not None:
                        samples.append(smp)
                now = time.perf_counter()
                if self.sending:
                    if now >= next_tx:
                        period = 1.0 / max(1.0, self.rate_hz)
                        next_tx += period
                        if next_tx < now:
                            next_tx = now + period
                        if self._tick(now):
                            rate_n += 1
                else:
                    next_tx = now
                    self._last_out = NAN
                self._service_config(now)
                self._service_scan(now)
                if now >= next_bus_check:
                    next_bus_check = now + 1.0
                    try:
                        bus_text = self.bus.status_text()
                    except Exception as e:
                        bus_text = str(e)
                    tx_rate = rate_n / (now - rate_t0)
                    rate_t0, rate_n = now, 0
                if now >= next_emit:
                    next_emit = now + 1.0 / self.EMIT_HZ
                    if samples:
                        self.samples.emit(samples)
                        samples = []
                    if self._frames:
                        fr, self._frames = self._frames, []
                        fr.sort(key=lambda x: x.t)
                        self.frames.emit(fr)
                    self.state.emit(self._state(now, bus_text, tx_rate))
                time.sleep(0.0003)
        finally:
            if not gc.isenabled():
                gc.enable()
            if winmm is not None:
                winmm.timeEndPeriod(1)

    def _state(self, now: float, bus_text: str, tx_rate: float) -> dict:
        d = self.driver
        link_ok = self.last_fb is not None and (now - self.last_fb_t) < 0.2
        return {
            "driver": d,
            "code": self.motor_code if link_ok else -1,
            "enabled": self.active,
            "active": self.active,
            "sending": self.sending,
            "mode": self.mode,
            "link_ok": link_ok,
            "bus": bus_text,
            "tx_rate": tx_rate if self.sending else 0.0,
            "running": self._run_obj is not None,
            "scanning": self._scan is not None,
            "queue": len(self._reqs) + (1 if self._pending else 0),
            "limits_ok": self.limits_ok,
            "blocked": self._config_blocked(),
            "tripped": self.safety.tripped,
            "trip_reason": self.safety.reason,
            "pos_mt": self.safety.pos,
            "lat": self.latency_stats(),
        }
