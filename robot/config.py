"""Robot configuration loaded from environment / .env.

Only hardware + safety params live here. The model, API key and chat channels are
configured in OpenClaw's own config (openclaw.config.json5), not here.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

from dotenv import load_dotenv

load_dotenv()


def _f(name: str, default: float) -> float:
    return float(os.getenv(name, default))


def _i(name: str, default: int) -> int:
    return int(os.getenv(name, default))


@dataclass(frozen=True)
class Config:
    # Robot network
    trik_ip: str
    trik_port: int
    bridge_port: int

    # Safety caps
    max_linear: float
    max_angular: float
    cmd_timeout_s: float
    estop_distance_m: float
    lidar_stale_s: float

    @classmethod
    def load(cls) -> "Config":
        return cls(
            trik_ip=os.getenv("TRIK_IP", "192.168.50.248"),
            trik_port=_i("TRIK_PORT", 5005),
            bridge_port=_i("BRIDGE_PORT", 5006),
            max_linear=_f("MAX_LINEAR", 0.6),
            max_angular=_f("MAX_ANGULAR", 1.0),
            cmd_timeout_s=_f("CMD_TIMEOUT_S", 2.0),
            estop_distance_m=_f("ESTOP_DISTANCE_M", 0.35),
            lidar_stale_s=_f("LIDAR_STALE_S", 1.5),
        )
