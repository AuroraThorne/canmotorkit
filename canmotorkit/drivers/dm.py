"""达妙 / HM 关节电机驱动（MIT 控制帧 + 0x7FF 参数读写）。"""
from __future__ import annotations

from . import dm_protocol as P
from .base import Caps, Driver, Feedback, Frames

# 型号 -> (寄存器家族, 说明)
MODELS = {
    "HM9816G9": ("hm", "HM 关节电机，对标达妙 J8009P"),
    "DM-J8009P": ("dm", "达妙 J8009P-2EC"),
    "达妙/HM 通用": ("both", "型号未知，按协议族通用处理"),
}


class DmDriver(Driver):
    protocol = "dm"
    protocol_name = "达妙/HM MIT"
    caps = Caps(mit=True, registers=True, enable_cmd=True, zero=True, clear=True, status_code=True)
    out_name = "扭矩"
    out_unit = "Nm"

    def __init__(self, model: str = "HM9816G9", motor_id: int = 1, master_id: int = 0):
        super().__init__(model, motor_id)
        self.master_id = master_id
        self.family = MODELS.get(model, ("both", ""))[0]
        self.note = MODELS.get(model, ("", ""))[1]
        self.lim = P.DEFAULT_LIMITS
        self.limits_ok = False

    @property
    def ready(self) -> bool:
        return self.limits_ok

    @property
    def out_max(self) -> float:
        return self.lim.t_max

    @property
    def wrap_span(self) -> float:
        return 2 * self.lim.p_max

    def title(self) -> str:
        return f"{self.model}  ID {self.motor_id} / Master 0x{self.master_id:X}"

    def set_limits(self, p_max: float, v_max: float, t_max: float):
        self.lim = P.DEFAULT_LIMITS.with_maps(p_max, v_max, t_max)
        self.limits_ok = True

    def invalidate_limits(self):
        self.lim = P.DEFAULT_LIMITS
        self.limits_ok = False

    # ---- 解析 ----
    def is_config_reply(self, data: bytes, motor_id: int | None = None) -> bool:
        mid = self.motor_id if motor_id is None else motor_id
        return len(data) >= 4 and data[2] in P.CONFIG_CMDS and (data[0] | (data[1] << 8)) == mid

    def accepts(self, can_id: int, data: bytes) -> bool:
        return can_id == self.master_id and len(data) == 8 and not self.is_config_reply(data)

    def parse(self, can_id: int, data: bytes) -> Feedback | None:
        st = P.unpack_status(data, self.lim)
        return Feedback(pos=st.pos, vel=st.vel, out=st.torque, temp=float(st.t_mos),
                        temp2=float(st.t_rotor), code=st.err, enabled=st.err == P.CODE_ENABLED,
                        motor_id=st.motor_id4, raw=bytes(data))

    # ---- 指令 ----
    def output_frames(self, out: float) -> Frames:
        """电流（扭矩）模式 = MIT kp=kd=p=v=0，t=out。"""
        return [(self.motor_id, P.pack_mit(P.MitCommand(t=out), self.lim))]

    def stop_frames(self) -> Frames:
        # 全 0 MIT 帧与量程无关（对称量程中点），量程未知时也安全
        return [(self.motor_id, P.pack_mit(P.MitCommand(), self.lim))]

    def mit_frames(self, cmd: P.MitCommand) -> Frames:
        return [(self.motor_id, P.pack_mit(cmd, self.lim))]

    def idle_frames(self) -> Frames:
        return [(P.CONFIG_ID, P.query_state_frame(self.motor_id))]

    def enable_frames(self) -> Frames:
        return [(self.motor_id, P.special_frame(P.SPECIAL_ENABLE))]

    def disable_frames(self) -> Frames:
        return [(self.motor_id, P.special_frame(P.SPECIAL_DISABLE))]

    def clear_frames(self) -> Frames:
        return [(self.motor_id, P.special_frame(P.SPECIAL_CLEAR_ERROR))]

    def zero_frames(self) -> Frames:
        return [(self.motor_id, P.special_frame(P.SPECIAL_ZERO))]

    def code_text(self, code: int) -> str:
        return P.code_text(code)

    def is_fault(self, code: int) -> bool:
        return P.is_fault(code)

    def describe(self, can_id: int, data: bytes, tx: bool) -> str:
        d = data
        if len(d) != 8:
            return ""
        if tx:
            if can_id == P.CONFIG_ID and (d[0] | (d[1] << 8)) == self.motor_id:
                return {P.CMD_READ: f"读 0x{d[3]:02X}", P.CMD_WRITE: f"写 0x{d[3]:02X}",
                        P.CMD_SAVE: "保存" if d[3] == 1 else f"0xAA/{d[3]:02X}",
                        P.CMD_STATE: "查询状态"}.get(d[2], "")
            if can_id == self.motor_id:
                if P.is_special(d):
                    return P.SPECIAL_NAMES[d[7]]
                return "MIT控制"
            return ""
        if can_id != self.master_id:
            return ""
        if self.is_config_reply(d):
            if d[2] == P.CMD_SAVE:
                return "保存应答"
            r = P.REG_BY_RID.get(d[3])
            return f"应答 {r.name} = {P.decode_value(r.typ, d[4:8]):.7g}" if r else "应答"
        st = P.unpack_status(d, self.lim)
        position_text = f"{st.pos:>8.5f}"
        speed_text = f"{st.vel:>8.3f}"
        torque_text = f"{st.torque:>+8.5f}"
        temperature_text = f", 温度: {st.t_mos:>3d}/{st.t_rotor:>3d} °C"
        return (f"位置: {position_text} rad, 速度: {speed_text} rad/s, "
                f"扭矩: {torque_text} Nm{temperature_text}")
