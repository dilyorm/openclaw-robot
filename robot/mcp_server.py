"""OpenClaw robot MCP server.

Exposes three tools to OpenClaw (over stdio):
    get_status()                      -> robot + lidar status
    drive(linear, angular, duration)  -> safe motion command
    stop()                            -> immediate halt

Run standalone:  python -m robot.mcp_server
OpenClaw launches it via the mcp.servers entry in openclaw.config.json5.
"""
from __future__ import annotations

import json

from mcp.server.fastmcp import FastMCP

from .config import Config
from .hardware.lidar import LidarClient
from .hardware.motors import MotorClient
from .safety import SafetySupervisor

cfg = Config.load()
_lidar = None
if cfg.lidar_enabled:
    _lidar = LidarClient(cfg)
    _lidar.start()
_motors = MotorClient(cfg)
_motors.start()  # persistent TCP link to TRIK (reconnects on its own)
_supervisor = SafetySupervisor(_motors, _lidar, cfg)

mcp = FastMCP("openclaw-robot")


@mcp.tool()
def get_status() -> str:
    """Report the robot's current state and surroundings.

    Returns a JSON object with:
      - motors.connected: TCP link to the TRIK controller is up
      - motors.left_enc / right_enc: wheel encoder counts
      - motors.gyro_z: heading gyro reading
      - motors.telemetry_age_s: seconds since last telemetry frame
      - lidar.enabled: whether a lidar is fitted on this robot
      - lidar.connected / sectors / nearest: obstacle data (only if a lidar is fitted)
      - last_command: the most recent drive result, if any
      - safety_caps: configured limits
    Call this before moving when you are unsure of the robot's state.
    """
    summary = (_lidar.get_summary() if _lidar is not None else None) or {}
    last = _supervisor.last_result
    status = {
        "motors": _motors.get_state(),
        "lidar": {
            "enabled": cfg.lidar_enabled,
            "connected": _lidar.is_connected() if _lidar is not None else False,
            "sectors": summary.get("sectors"),
            "nearest": summary.get("nearest"),
            "num_points": summary.get("num_points"),
        },
        "last_command": (
            None
            if last is None
            else {
                "linear": last.linear,
                "angular": last.angular,
                "duration": last.duration,
                "blocked": last.blocked,
                "reason": last.reason,
            }
        ),
        "safety_caps": {
            "max_linear": cfg.max_linear,
            "max_angular": cfg.max_angular,
            "cmd_timeout_s": cfg.cmd_timeout_s,
            "estop_distance_m": cfg.estop_distance_m,
        },
    }
    return json.dumps(status, indent=2)


@mcp.tool()
def drive(linear: float, angular: float, duration: float = 1.0) -> str:
    """Move the robot.

    Args:
        linear: forward/back speed in m/s. Positive = forward, negative = reverse.
            Clamped to +/- the max_linear safety cap.
        angular: turn rate in rad/s. Positive = turn left (CCW), negative = right.
            Clamped to +/- the max_angular safety cap.
        duration: how long to apply the command, in seconds. Capped at
            cmd_timeout_s; the robot auto-stops afterwards (deadman).

    When a lidar is fitted, forward motion is blocked if an obstacle is within
    the e-stop distance or lidar data is unavailable (fail-safe). On a robot
    without a lidar there is no obstacle sensing, so move in short, low-speed
    steps. The returned JSON states exactly what was executed, whether it was
    blocked, and why.
    """
    r = _supervisor.drive(linear, angular, duration)
    return json.dumps(
        {
            "executed": {"linear": r.linear, "angular": r.angular, "duration": r.duration},
            "blocked": r.blocked,
            "clamped": r.clamped,
            "reason": r.reason,
        },
        indent=2,
    )


@mcp.tool()
def stop() -> str:
    """Immediately stop the robot. Always succeeds; bypasses all checks."""
    _supervisor.stop()
    return json.dumps({"stopped": True})


def main() -> None:
    mcp.run()


if __name__ == "__main__":
    main()
