"""后台通信线程：接收解析、MIT控制帧或0xCC查询帧定频发送、配置请求串行调度、总线扫描、测试信号。

状态约定（实机验证）：
- 电机是否使能以状态帧 d0 高 4 位为准：0=失能，1=使能，>=2 故障。
- 按统一的通信频率发送：电机已使能时发 MIT 控制帧（已应用的目标），否则发 0xCC 查询帧。
- 电机使能期间不发 0x7FF 配置帧（读请求排队等待，写/保存/扫描拒绝）。
- p/v/t 量程必须从电机读取（0x15/0x16/0x17）后才允许使能；若电机已使能但量程未知，
  只发与量程无关的零扭矩帧（kp=kd=t=0）。
"""
from __future__ import annotations

import collections
import ctypes
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass, replace

from PySide6.QtCore import QThread, Signal

from ..drivers import dm_protocol as P
from .bus import BusError, CanBus, Frame
from ..analysis.siggen import SignalGen
from ..control.safety import SafetyLimits, Supervisor

LIMIT_RIDS = (0x15, 0x16, 0x17)


@dataclass
class Sample:
    t: float
    cmd: P.MitCommand       # 最近一次下发的 MIT 指令（mit=False 时无意义）
    st: P.Status
    mit: bool               # 该状态帧显示电机已使能（处于 MIT 发送中）


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
    frames = Signal(object)             # list[Frame]（含发送帧 tx=True）
    reg_done = Signal(str, int, object, str)   # kind, rid, value, 错误信息(空=成功)
    scan_progress = Signal(int, int)
    scan_done = Signal(object)          # list[(motor_id, reply_can_id, Status)]
    state = Signal(object)              # dict
    message = Signal(str)
    limits_changed = Signal(object, bool)   # Limits, 是否已从电机读取
    signal_finished = Signal()

    EMIT_HZ = 30

    def __init__(self, bus: CanBus, motor_id: int = 1, master_id: int = 0):
        super().__init__()
        self.bus = bus
        self.motor_id = motor_id
        self.master_id = master_id
        self.lim = P.DEFAULT_LIMITS
        self.limits_ok = False
        self._lim_vals: dict[int, float] = {}
        self.strict_id = False
        self.rate_hz = 200.0        # 通信频率：MIT 帧或 0xCC 查询帧
        self.req_timeout = 0.1
        self.req_retries = 2
        self.log_enabled = True

        self._lock = threading.Lock()
        self._target = P.MitCommand()
        self._last_cmd = P.MitCommand()
        self._warned_nolim = False
        self.safety = Supervisor()
        self._ops: queue.Queue = queue.Queue()
        self._reqs: collections.deque[_Req] = collections.deque()
        self._pending: _Req | None = None
        self._scan: dict | None = None
        self._sig: tuple[SignalGen, str, float] | None = None
        self._running = False
        self._frames: list[Frame] = []
        self._last_err_t = 0.0
        self.motor_code = -1                # 最近状态帧状态码，-1 = 未知
        self.last_status: P.Status | None = None
        self.last_status_t = 0.0
        self._queue_limit_reads()

    # ================= 线程安全的公开接口 =================
    def _post(self, fn, *a):
        self._ops.put((fn, a))

    @property
    def enabled(self) -> bool:
        """电机反馈的使能状态。"""
        return self.motor_code == P.CODE_ENABLED

    def set_target(self, cmd: P.MitCommand):
        with self._lock:
            self._target = replace(cmd)

    def get_target(self) -> P.MitCommand:
        with self._lock:
            return replace(self._target)

    def set_ids(self, motor_id: int, master_id: int):
        self._post(self._do_set_ids, motor_id, master_id)

    def reload_limits(self):
        self._post(self._do_reload_limits)

    def enable(self):
        self._post(self._do_enable)

    def disable(self):
        self._post(self._do_disable)

    def estop(self, reason: str = "用户急停"):
        """急停：零扭矩（kp=kd=t=0）并锁存，不失能。"""
        self._post(self._do_estop, reason)

    def reset_safety(self):
        self._post(self._do_reset_safety)

    def set_safety_limits(self, lim: SafetyLimits):
        self._post(setattr, self.safety, "limits", lim)

    def zero_safety_position(self):
        self._post(self.safety.zero_position)

    def clear_error(self):
        self._post(self._do_clear)

    def set_zero(self):
        self._post(self._do_zero)

    def read_reg(self, rid: int):
        self._post(self._do_read, rid)

    def write_reg(self, rid: int, value):
        self._post(self._do_write, rid, value)

    def save(self):
        self._post(self._do_save)

    def start_scan(self, ids=range(1, 128), window: float = 0.02):
        self._post(self._do_scan, list(ids), window)

    def stop_scan(self):
        self._post(self._do_stop_scan)

    def send_raw(self, can_id: int, data: bytes):
        self._post(self._send, can_id, bytes(data))

    def start_signal(self, gen: SignalGen, channel: str):
        self._post(self._do_start_signal, gen, channel)

    def stop_signal(self):
        self._post(self._do_stop_signal)

    def stop(self):
        """安全停止：若已使能先发失能，再退出线程。"""
        self._post(self._do_shutdown)

    # ================= 线程内部实现 =================
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

    def _config_blocked(self) -> bool:
        return self.motor_code == P.CODE_ENABLED

    BLOCK_TIP = "电机使能中，请先失能"

    # ---------- 量程 ----------
    def _queue_limit_reads(self):
        self.limits_ok = False
        self._lim_vals = {}
        self._reqs = collections.deque(r for r in self._reqs if r.rid not in LIMIT_RIDS)
        for rid in LIMIT_RIDS:
            self._reqs.append(_Req("read", rid, "f", P.read_reg_frame(self.motor_id, rid)))

    def _do_reload_limits(self):
        if self._config_blocked():
            self._msg("不能重读量程：" + self.BLOCK_TIP)
            return
        self._queue_limit_reads()
        self.limits_changed.emit(self.lim, False)

    def _on_limit_value(self, rid: int, val):
        v = float(val)
        if not math.isfinite(v) or v <= 0:
            self._msg(f"量程寄存器 0x{rid:02X} 值无效: {v}")
            return
        self._lim_vals[rid] = v
        if all(r in self._lim_vals for r in LIMIT_RIDS):
            self.lim = P.DEFAULT_LIMITS.with_maps(self._lim_vals[0x15], self._lim_vals[0x16],
                                                  self._lim_vals[0x17])
            was = self.limits_ok
            self.limits_ok = True
            self.safety.set_wrap_span(2 * self.lim.p_max)
            self.limits_changed.emit(self.lim, True)
            if not was:
                L = self.lim
                self._msg(f"已读取量程: p±{L.p_max:.4g} v±{L.v_max:.4g} t±{L.t_max:.4g}")

    # ---------- 模式 ----------
    def _do_set_ids(self, motor_id: int, master_id: int):
        if self._config_blocked():
            self._msg("不能切换 ID：" + self.BLOCK_TIP)
            return
        self.motor_id, self.master_id = motor_id, master_id
        self.last_status = None
        self.motor_code = -1
        self._queue_limit_reads()
        self.limits_changed.emit(self.lim, False)

    def _do_enable(self):
        if self._scan is not None:
            self._msg("扫描中，不能使能")
            return
        if not self.limits_ok:
            self._msg("尚未从电机读到 p/v/t 量程，禁止使能（可在参数页重读）")
            return
        if self._send(self.motor_id, P.special_frame(P.SPECIAL_ENABLE)):
            self._msg("已发送使能 FC，电机确认使能后改发 MIT 控制帧")

    def _abort_signal(self):
        if self._sig:
            self._sig = None
            self.signal_finished.emit()

    def _went_disabled(self):
        """发出 FD 后立即切回查询帧，不等应答；以后续状态帧为准。"""
        if self.motor_code == P.CODE_ENABLED:
            self.motor_code = P.CODE_DISABLED

    def _do_disable(self):
        self._abort_signal()
        if self._send(self.motor_id, P.special_frame(P.SPECIAL_DISABLE)):
            self._went_disabled()
        self._msg("已发送失能 FD")

    def _do_estop(self, reason: str):
        self._abort_signal()
        self.safety.trip(reason)
        self._send(self.motor_id, P.pack_mit(P.MitCommand(), self.lim))
        self._msg(f"急停（{reason}）：已发送零扭矩帧并锁存，电机仍使能；失能请手动点击")

    def _do_reset_safety(self):
        if self.safety.reset():
            self._msg("已解除急停/超限锁存")
        else:
            self._msg("不能解除：" + self.safety.reason)

    def _do_clear(self):
        self._send(self.motor_id, P.special_frame(P.SPECIAL_CLEAR_ERROR))

    def _do_zero(self):
        if self._config_blocked():
            self._msg("不能设零点：" + self.BLOCK_TIP)
            return
        self._send(self.motor_id, P.special_frame(P.SPECIAL_ZERO))
        self._msg("已发送设零点")

    def _do_shutdown(self):
        if self.motor_code == P.CODE_ENABLED:
            self._do_disable()
        self._running = False

    # ---------- 配置 ----------
    def _do_read(self, rid: int):
        reg = P.REG_BY_RID.get(rid)
        if any(r.kind == "read" and r.rid == rid for r in self._reqs):
            return
        self._reqs.append(_Req("read", rid, reg.typ if reg else "u", P.read_reg_frame(self.motor_id, rid)))

    def _do_write(self, rid: int, value):
        reg = P.REG_BY_RID.get(rid)
        typ = reg.typ if reg else "u"
        if reg and reg.ro:
            self.reg_done.emit("write", rid, None, "只读寄存器")
            return
        if self._config_blocked():
            self.reg_done.emit("write", rid, None, self.BLOCK_TIP)
            return
        try:
            frame = P.write_reg_frame(self.motor_id, rid, typ, value)
        except Exception as e:  # struct.error / ValueError
            self.reg_done.emit("write", rid, None, f"数值无效: {e}")
            return
        self._reqs.append(_Req("write", rid, typ, frame))

    def _do_save(self):
        if self._config_blocked():
            self.reg_done.emit("save", 0, None, "保存参数只在失能模式下生效：" + self.BLOCK_TIP)
            return
        self._reqs.append(_Req("save", 0, "u", P.save_frame(self.motor_id)))

    def _do_scan(self, ids: list[int], window: float):
        if self._config_blocked():
            self._msg("不能扫描：" + self.BLOCK_TIP)
            self.scan_done.emit([])
            return
        self._scan = {"ids": ids, "i": -1, "cur": None, "deadline": 0.0,
                      "window": window, "found": {}}

    def _do_stop_scan(self):
        if self._scan is not None:
            self._finish_scan()

    def _do_start_signal(self, gen: SignalGen, channel: str):
        if self.motor_code != P.CODE_ENABLED or not self.limits_ok:
            self._msg("请先使能电机（且量程已读取）再开始测试信号")
            self.signal_finished.emit()
            return
        self._sig = (gen, channel, time.perf_counter())

    def _do_stop_signal(self):
        if self._sig:
            gen, ch, _ = self._sig
            self._finish_signal(gen, ch, hold=gen.final())

    def _finish_signal(self, gen: SignalGen, ch: str, hold: float):
        tgt = self.get_target()
        setattr(tgt, ch, hold)
        self.set_target(tgt)
        self._sig = None
        self.signal_finished.emit()

    # ---------- 周期任务 ----------
    def _compute_cmd(self, now: float) -> P.MitCommand:
        if self.safety.tripped:
            return P.MitCommand()      # 锁存：只发零扭矩
        cmd = self.get_target()
        if self._sig:
            gen, ch, t0 = self._sig
            v = gen.value(now - t0)
            if v is None:
                self._finish_signal(gen, ch, gen.final())
                cmd = self.get_target()
            else:
                setattr(cmd, ch, v)
        cmd.t = self.safety.filter_output(now, cmd.t)
        if self.safety.tripped:
            self._on_trip()
            return P.MitCommand()
        L = self.lim
        return P.MitCommand(P.clamp(cmd.p, L.p_min, L.p_max), P.clamp(cmd.v, L.v_min, L.v_max),
                            P.clamp(cmd.kp, L.kp_min, L.kp_max), P.clamp(cmd.kd, L.kd_min, L.kd_max),
                            P.clamp(cmd.t, L.t_min, L.t_max))

    def _on_trip(self):
        if not getattr(self, "_trip_reported", False):
            self._trip_reported = True
            self._abort_signal()
            self._msg("安全限制触发零扭矩：" + self.safety.reason)

    def _on_rx(self, f: Frame) -> P.Status | None:
        d = f.data
        if len(d) != 8 or f.can_id == P.CONFIG_ID:
            return None
        p = self._pending
        if p and f.can_id == self.master_id and P.matches_request(d, p.frame):
            self._finish_req(d)
            return None
        if self._scan is not None:
            cur = self._scan["cur"]
            if cur is not None and cur not in self._scan["found"]:
                self._scan["found"][cur] = (f.can_id, P.unpack_status(d, self.lim))
            return None
        if f.can_id != self.master_id:
            return None
        mid = self.motor_id
        if d[2] in P.CONFIG_CMDS and d[0] == (mid & 0xFF) and d[1] == ((mid >> 8) & 0xFF):
            return None     # 迟到的配置应答
        st = P.unpack_status(d, self.lim)
        if self.strict_id and st.motor_id4 != (mid & 0x0F):
            return None
        if st.err != self.motor_code and self.motor_code != -1 and P.is_fault(st.err):
            self._msg(f"电机报告 {P.code_text(st.err)}")
        if st.err != P.CODE_ENABLED and self._sig:
            self._abort_signal()
            self._msg("电机已退出使能，测试信号中止")
        self.motor_code = st.err
        self.last_status = st
        self.last_status_t = f.t
        self.safety.on_feedback(f.t, st.pos, st.vel, max(st.t_mos, st.t_rotor))
        if self.safety.tripped and st.err == P.CODE_ENABLED:
            self._on_trip()
        elif not self.safety.tripped:
            self._trip_reported = False
        return st

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

    def _service_scan(self, now: float):
        s = self._scan
        if s is None or self._pending is not None or now < s["deadline"]:
            return
        s["i"] += 1
        if s["i"] >= len(s["ids"]):
            self._finish_scan()
            return
        cur = s["ids"][s["i"]]
        s["cur"] = cur
        s["deadline"] = now + s["window"]
        self._send(P.CONFIG_ID, P.query_state_frame(cur))
        if s["i"] % 4 == 0:
            self.scan_progress.emit(s["i"], len(s["ids"]))

    def _finish_scan(self):
        s = self._scan
        self._scan = None
        found = [(mid, cid, st) for mid, (cid, st) in sorted(s["found"].items())]
        self.scan_progress.emit(len(s["ids"]), len(s["ids"]))
        self.scan_done.emit(found)

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
                # 1) UI 指令
                while True:
                    try:
                        fn, a = self._ops.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        fn(*a)
                    except Exception as e:  # 防止单个操作异常杀死线程
                        self._err(f"{type(e).__name__}: {e}")
                # 2) 接收
                try:
                    rx = self.bus.recv_all()
                except Exception as e:
                    self._err(f"接收失败: {e}")
                    rx = []
                for f in rx:
                    if self.log_enabled:
                        self._frames.append(f)
                    st = self._on_rx(f)
                    if st is not None:
                        samples.append(Sample(f.t, replace(self._last_cmd), st, st.err == P.CODE_ENABLED))
                # 3) 按通信频率发送：已使能发 MIT 帧，否则发 0xCC 查询帧
                now = time.perf_counter()
                if now >= next_tx:
                    period = 1.0 / max(1.0, self.rate_hz)
                    next_tx += period
                    if next_tx < now:
                        next_tx = now + period
                    if self.motor_code == P.CODE_ENABLED:
                        if self.limits_ok:
                            cmd = self._compute_cmd(now)
                        else:
                            cmd = P.MitCommand()     # 量程未知：只发与量程无关的零扭矩帧
                            if not self._warned_nolim:
                                self._warned_nolim = True
                                self._msg("电机已使能但量程未读取，仅发送零扭矩帧，请先失能")
                        if self._send(self.motor_id, P.pack_mit(cmd, self.lim)):
                            self._last_cmd = cmd
                            rate_n += 1
                    elif self._scan is None and self._pending is None:
                        if self._send(P.CONFIG_ID, P.query_state_frame(self.motor_id)):
                            rate_n += 1
                # 4) 配置 / 扫描
                self._service_config(now)
                self._service_scan(now)
                # 5) 周期上报
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
                    link_ok = self.last_status is not None and (now - self.last_status_t) < 0.2
                    self.state.emit({
                        "code": self.motor_code if link_ok else -1,
                        "enabled": link_ok and self.motor_code == P.CODE_ENABLED,
                        "link_ok": link_ok,
                        "bus": bus_text,
                        "tx_rate": tx_rate,
                        "signal": self._sig is not None,
                        "scanning": self._scan is not None,
                        "queue": len(self._reqs) + (1 if self._pending else 0),
                        "limits_ok": self.limits_ok,
                        "blocked": self._config_blocked(),
                        "tripped": self.safety.tripped,
                        "trip_reason": self.safety.reason,
                        "pos_mt": self.safety.pos,
                    })
                time.sleep(0.0005)
        finally:
            if winmm is not None:
                winmm.timeEndPeriod(1)
