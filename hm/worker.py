"""后台通信线程：接收解析、MIT 周期下发、配置请求串行调度、总线扫描、测试信号。

状态约定（实机验证）：
- 电机是否使能以状态帧 d0 高 4 位为准：0=失能，1=使能，>=2 故障。
- “周期下发”是独立开关：打开时按频率发送 MIT 帧；关闭时用 0xCC 轮询状态。
- 电机使能或周期下发期间不发 0x7FF 配置帧（读请求排队等待，写/保存/扫描拒绝）。
- p/v/t 量程必须从电机读取（0x15/0x16/0x17）后才允许使能/周期下发。
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

from . import protocol as P
from .bus import BusError, CanBus, Frame
from .siggen import SignalGen

LIMIT_RIDS = (0x15, 0x16, 0x17)


@dataclass
class Sample:
    t: float
    cmd: P.MitCommand       # 最近一次下发的 MIT 指令（未周期下发时无意义）
    st: P.Status
    streaming: bool


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
        self.rate_hz = 200.0
        self.poll_hz = 50.0
        self.req_timeout = 0.1
        self.req_retries = 2
        self.log_enabled = True

        self._lock = threading.Lock()
        self._target = P.MitCommand()
        self._last_cmd = P.MitCommand()
        self._streaming = False
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

    @property
    def streaming(self) -> bool:
        return self._streaming

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

    def estop(self):
        self._post(self._do_estop)

    def set_streaming(self, on: bool):
        self._post(self._do_set_streaming, bool(on))

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
        """安全停止：停止周期下发并发失能，再退出线程。"""
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
        return self._streaming or self.motor_code == P.CODE_ENABLED

    BLOCK_TIP = "电机使能或周期下发中，请先失能并关闭周期下发"

    # ---------- 量程 ----------
    def _queue_limit_reads(self):
        self.limits_ok = False
        self._lim_vals = {}
        self._reqs = collections.deque(r for r in self._reqs if r.rid not in LIMIT_RIDS)
        for rid in LIMIT_RIDS:
            self._reqs.append(_Req("read", rid, "f", P.read_reg_frame(self.motor_id, rid)))

    def _do_reload_limits(self):
        if self._streaming:
            self._msg("周期下发中不能重读量程")
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
            self._msg("已发送使能 FC" + ("" if self._streaming else "（周期下发未打开，不会发送控制帧）"))

    def _stop_streaming(self):
        if self._sig:
            self._sig = None
            self.signal_finished.emit()
        self._streaming = False

    def _do_disable(self):
        self._stop_streaming()
        self._send(self.motor_id, P.special_frame(P.SPECIAL_DISABLE))
        self._msg("已发送失能 FD，周期下发已关闭")

    def _do_estop(self):
        self._stop_streaming()
        self._send(self.motor_id, P.pack_mit(P.MitCommand(), self.lim))
        self._send(self.motor_id, P.special_frame(P.SPECIAL_DISABLE))
        self._msg("急停：已发送零力矩帧 + 失能，周期下发已关闭")

    def _do_set_streaming(self, on: bool):
        if on == self._streaming:
            return
        if on:
            if not self.limits_ok:
                self._msg("尚未读到量程，不能周期下发")
                return
            if self._scan is not None:
                self._msg("扫描中，不能周期下发")
                return
            self._streaming = True
            self._last_cmd = self._compute_cmd(time.perf_counter())
            self._msg("周期下发已打开")
        else:
            self._stop_streaming()
            self._msg("周期下发已关闭（电机若仍使能将保持最后指令或触发超时保护）")

    def _do_clear(self):
        self._send(self.motor_id, P.special_frame(P.SPECIAL_CLEAR_ERROR))

    def _do_zero(self):
        if self._config_blocked():
            self._msg("不能设零点：" + self.BLOCK_TIP)
            return
        self._send(self.motor_id, P.special_frame(P.SPECIAL_ZERO))
        self._msg("已发送设零点")

    def _do_shutdown(self):
        if self._streaming or self.motor_code == P.CODE_ENABLED:
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
        if not self._streaming:
            self._msg("请先打开周期下发再开始测试信号")
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
        cmd = self.get_target()
        if self._sig:
            gen, ch, t0 = self._sig
            v = gen.value(now - t0)
            if v is None:
                self._finish_signal(gen, ch, gen.final())
                cmd = self.get_target()
            else:
                setattr(cmd, ch, v)
        L = self.lim
        return P.MitCommand(P.clamp(cmd.p, L.p_min, L.p_max), P.clamp(cmd.v, L.v_min, L.v_max),
                            P.clamp(cmd.kp, L.kp_min, L.kp_max), P.clamp(cmd.kd, L.kd_min, L.kd_max),
                            P.clamp(cmd.t, L.t_min, L.t_max))

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
        self.motor_code = st.err
        self.last_status = st
        self.last_status_t = f.t
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
        next_tx = next_poll = next_emit = now
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
                        samples.append(Sample(f.t, replace(self._last_cmd), st, self._streaming))
                # 3) 周期下发 MIT 或状态轮询
                now = time.perf_counter()
                if self._streaming:
                    if now >= next_tx:
                        cmd = self._compute_cmd(now)
                        if self._streaming and self._send(self.motor_id, P.pack_mit(cmd, self.lim)):
                            self._last_cmd = cmd
                            rate_n += 1
                        period = 1.0 / max(1.0, self.rate_hz)
                        next_tx += period
                        if next_tx < now:
                            next_tx = now + period
                else:
                    next_tx = now
                    if (self.poll_hz > 0 and self._scan is None and self._pending is None
                            and now >= next_poll):
                        self._send(P.CONFIG_ID, P.query_state_frame(self.motor_id))
                        next_poll = now + 1.0 / self.poll_hz
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
                        "streaming": self._streaming,
                        "link_ok": link_ok,
                        "bus": bus_text,
                        "tx_rate": tx_rate if self._streaming else 0.0,
                        "signal": self._sig is not None,
                        "scanning": self._scan is not None,
                        "queue": len(self._reqs) + (1 if self._pending else 0),
                        "limits_ok": self.limits_ok,
                        "blocked": self._config_blocked(),
                    })
                time.sleep(0.0005)
        finally:
            if winmm is not None:
                winmm.timeEndPeriod(1)
