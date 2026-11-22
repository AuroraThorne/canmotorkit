"""协议与型号目录，用于手动添加电机和扫描结果建驱动。"""
from __future__ import annotations

from . import dji, dm, gd
from .base import Driver

PROTOCOLS: dict[str, tuple[str, list[str]]] = {
    "dm": (dm.DmDriver.protocol_name, list(dm.MODELS)),
    "dji": (dji.DjiDriver.protocol_name, list(dji.MODELS)),
    "gd": (gd.GdDriver.protocol_name, list(gd.MODELS)),
}


def make_driver(protocol: str, model: str, motor_id: int, master_id: int = 0) -> Driver:
    if protocol == "dm":
        return dm.DmDriver(model, motor_id, master_id)
    if protocol == "dji":
        return dji.DjiDriver(model, motor_id)
    if protocol == "gd":
        return gd.GdDriver(model, motor_id)
    raise ValueError(f"未知协议 {protocol}")
