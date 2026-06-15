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


def _b(name: str, default: bool) -> bool:
    val = os.getenv(name)
    if val is None:
        return default
    return val.strip().lower() in ("1", "true", "yes", "on")


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

    # Motor PWM gains: max linear maps to fwd_pwm %, max angular to turn_pwm %.
    fwd_pwm: int
    turn_pwm: int

    # Lidar present? When False, the lidar listener is not started and the
    # e-stop / fail-safe forward-block is skipped (set per deployment).
    lidar_enabled: bool

    @classmethod
    def load(cls) -> "Config":
        return cls(
            trik_ip=os.getenv("TRIK_IP", "192.168.50.237"),
            trik_port=_i("TRIK_PORT", 9090),
            bridge_port=_i("BRIDGE_PORT", 5006),
            max_linear=_f("MAX_LINEAR", 0.6),
            max_angular=_f("MAX_ANGULAR", 1.0),
            cmd_timeout_s=_f("CMD_TIMEOUT_S", 2.0),
            estop_distance_m=_f("ESTOP_DISTANCE_M", 0.35),
            lidar_stale_s=_f("LIDAR_STALE_S", 1.5),
            lidar_enabled=_b("LIDAR_ENABLED", False),
            fwd_pwm=_i("FWD_PWM", 60),
            turn_pwm=_i("TURN_PWM", 40),
        )
