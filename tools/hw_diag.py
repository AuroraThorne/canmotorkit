"""HM 电机实机诊断脚本（独立于 GUI）。

安全限制：只用 MIT 扭矩模式（kp=kd=p=v=0），|t| <= 1 Nm，单次 <= 5 s，
结束/异常时必发零扭矩帧 + FD。

用法:
  python tools/hw_diag.py info
  python tools/hw_diag.py torque --t 0 --dur 1
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.stdout.reconfigure(encoding="utf-8")

from canmotorkit.drivers import dm_protocol as P  # noqa: E402
from canmotorkit.core.pcan_basic import BAUDRATES, PCAN_ERROR_QRCVEMPTY, PCANBasic, USB_CHANNELS  # noqa: E402

T_LIMIT = 1.0
DUR_LIMIT = 5.0


class Link:
    def __init__(self, ch_name):
        self.api = PCANBasic()
        self.ch = USB_CHANNELS[ch_name]
        r = self.api.initialize(self.ch, BAUDRATES["1M"])
        if r:
            raise SystemExit(f"打开 {ch_name} 失败: {self.api.error_text(r)}（GUI 是否还开着？）")
        self.t0 = time.perf_counter()
        print(f"[已打开 {ch_name} 1M] channels={self.api.attached_channels()}")

    def ts(self):
        return time.perf_counter() - self.t0

    def send(self, cid, data, show=True):
        r = self.api.write(self.ch, cid, bytes(data))
        if show or r:
            print(f"{self.ts():8.4f} TX {cid:03X} {P.hexdata(data)}" + (f"  !! {self.api.error_text(r)}" if r else ""))
        return r == 0

    def recv(self, timeout):
        out = []
        end = time.perf_counter() + timeout
        while True:
            r, msg, _ = self.api.read(self.ch)
            if r & PCAN_ERROR_QRCVEMPTY or r:
                if time.perf_counter() >= end:
                    break
                time.sleep(0.0003)
                continue
            if msg.MSGTYPE & 0xC1:   # status/err/rtr
                print(f"{self.ts():8.4f} RX [状态/错误帧 type=0x{msg.MSGTYPE:X} id={msg.ID:X} {bytes(msg.DATA[:msg.LEN]).hex()}]")
                continue
            out.append((self.ts(), msg.ID, bytes(msg.DATA[:msg.LEN])))
        return out

    def close(self):
        self.api.uninitialize(self.ch)


def show_status(t, cid, d, lim, tag=""):
    if len(d) != 8:
        print(f"{t:8.4f} RX {cid:03X} {P.hexdata(d)} (len {len(d)})")
        return None
    s = P.unpack_status(d, lim)
    print(f"{t:8.4f} RX {cid:03X} {P.hexdata(d)} | id4={s.motor_id4} err={s.err} pos={s.pos:+.4f} "
          f"vel={s.vel:+.3f} tq={s.torque:+.3f} d6={s.t_mos} d7={s.t_rotor} {tag}")
    return s


def read_reg(link, mid, rid, show=False):
    link.recv(0.005)
    req = P.read_reg_frame(mid, rid)
    link.send(P.CONFIG_ID, req, show=show)
    for t, cid, d in link.recv(0.12):
        if P.matches_request(d, req):
            return cid, d
    return None, None


INFO_REGS = [0x00, 0x01, 0x02, 0x03, 0x07, 0x08, 0x09, 0x0B, 0x0C, 0x0D, 0x0E, 0x10, 0x14,
             0x15, 0x16, 0x17, 0x18, 0x1D, 0x23, 0x3B, 0x3C, 0x3D, 0x3E, 0x50, 0x51]


def cmd_info(link, a):
    lim = P.DEFAULT_LIMITS
    print("== 静默监听 0.3 s ==")
    fr = link.recv(0.3)
    print(f"收到 {len(fr)} 帧", [f"{c:03X}:{d.hex()}" for _, c, d in fr[:10]])
    print("== 发 FD（失能）==")
    link.send(a.id, P.special_frame(P.SPECIAL_DISABLE))
    for t, c, d in link.recv(0.1):
        show_status(t, c, d, lim, "<- FD 应答")
    print("== 0xCC 查询状态 x3 ==")
    for _ in range(3):
        link.send(P.CONFIG_ID, P.query_state_frame(a.id))
        for t, c, d in link.recv(0.05):
            show_status(t, c, d, lim)
    print("== 读寄存器 ==")
    for rid in INFO_REGS:
        reg = P.REG_BY_RID[rid]
        cid, d = read_reg(link, a.id, rid)
        if d is None:
            print(f"  0x{rid:02X} {reg.name:<24} 超时无应答")
        else:
            v = P.decode_value(reg.typ, d[4:8])
            print(f"  0x{rid:02X} {reg.name:<24} = {v:<14.6g} raw={d[4:8].hex()} 应答ID=0x{cid:X}  ({reg.desc})")


def cmd_torque(link, a):
    t_cmd = float(a.t)
    if abs(t_cmd) > T_LIMIT:
        raise SystemExit(f"|t| 不能超过 {T_LIMIT} Nm")
    dur = min(float(a.dur), DUR_LIMIT)
    # 读映射量程
    vals = {}
    for rid in (0x15, 0x16, 0x17):
        _, d = read_reg(link, a.id, rid)
        if d is None:
            raise SystemExit(f"读寄存器 0x{rid:02X} 失败，为安全起见不进行扭矩测试")
        vals[rid] = P.decode_value("f", d[4:8])
    lim = P.DEFAULT_LIMITS.with_maps(vals[0x15], vals[0x16], vals[0x17])
    print(f"量程: p±{lim.p_max} v±{lim.v_max} t±{lim.t_max}")
    frame = P.pack_mit(P.MitCommand(0, 0, 0, 0, t_cmd), lim)
    zero = P.pack_mit(P.MitCommand(), lim)
    print(f"指令帧(t={t_cmd}): {P.hexdata(frame)}   零扭矩帧: {P.hexdata(zero)}")
    link.send(P.CONFIG_ID, P.query_state_frame(a.id))
    for t, c, d in link.recv(0.05):
        show_status(t, c, d, lim, "<- 使能前状态")
    if a.clear:
        link.send(a.id, P.special_frame(P.SPECIAL_CLEAR_ERROR))
        for t, c, d in link.recv(0.05):
            show_status(t, c, d, lim, "<- FB 应答")
    try:
        if a.zero_first:
            print("== 先发一帧零扭矩 MIT（未使能）==")
            link.send(a.id, zero)
            for t, c, d in link.recv(0.05):
                show_status(t, c, d, lim)
        print("== 发 FC（使能）==")
        link.send(a.id, P.special_frame(P.SPECIAL_ENABLE))
        for t, c, d in link.recv(0.05):
            show_status(t, c, d, lim, "<- FC 应答")
        print(f"== 以 {a.rate} Hz 发送 t={t_cmd} Nm，持续 {dur} s ==")
        period = 1.0 / a.rate
        end = time.perf_counter() + dur
        nxt = time.perf_counter()
        n_tx = n_rx = 0
        last_d0 = None
        last_print = -1.0
        first = 0
        while time.perf_counter() < end:
            now = time.perf_counter()
            if now >= nxt:
                link.send(a.id, frame, show=(n_tx < 3))
                n_tx += 1
                nxt += period
            for t, c, d in link.recv(0.0):
                n_rx += 1
                if len(d) == 8 and (d[0] != last_d0 or first < 5 or t - last_print >= 0.25):
                    tag = "← d0 变化" if (last_d0 is not None and d[0] != last_d0) else ""
                    show_status(t, c, d, lim, tag)
                    last_d0 = d[0]
                    last_print = t
                    first += 1
            time.sleep(0.0002)
        print(f"发送 {n_tx} 帧，接收 {n_rx} 帧")
    finally:
        print("== 结束：零扭矩帧 + FD ==")
        link.send(a.id, zero)
        link.send(a.id, P.special_frame(P.SPECIAL_DISABLE))
        for t, c, d in link.recv(0.05):
            show_status(t, c, d, lim, "<- 停止后")
        link.send(P.CONFIG_ID, P.query_state_frame(a.id))
        for t, c, d in link.recv(0.05):
            show_status(t, c, d, lim, "<- 失能后状态")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["info", "torque"])
    ap.add_argument("--ch", default="PCAN_USBBUS1")
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--t", type=float, default=0.0)
    ap.add_argument("--dur", type=float, default=1.0)
    ap.add_argument("--rate", type=float, default=200.0)
    ap.add_argument("--clear", action="store_true", help="使能前先发 FB 清错")
    ap.add_argument("--zero-first", action="store_true", help="使能前先发一帧零扭矩 MIT")
    a = ap.parse_args()
    link = Link(a.ch)
    try:
        (cmd_info if a.cmd == "info" else cmd_torque)(link, a)
    finally:
        link.close()


if __name__ == "__main__":
    main()
