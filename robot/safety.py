"""Motion safety: clamping, lidar e-stop, and a deadman watchdog.

`evaluate_command` is a pure function (easy to unit test). `SafetySupervisor`
adds the stateful watchdog and the actual MotorClient calls.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional

from .config import Config


@dataclass
class CommandResult:
    """Outcome of a drive request after safety processing."""
    linear: float          # value actually sent (0.0 if blocked)
    angular: float
    duration: float        # capped run time, seconds
    blocked: bool          # forward motion refused by e-stop / fail-safe
    reason: Optional[str]  # human-readable explanation when blocked or clamped
    clamped: bool          # requested speed exceeded caps and was reduced


def _clamp(value: float, limit: float) -> tuple[float, bool]:
    if value > limit:
        return limit, True
    if value < -limit:
        return -limit, True
    return value, False


def evaluate_command(
    linear: float,
    angular: float,
    duration: float,
    front_distance: Optional[float],
    lidar_connected: bool,
    cfg: Config,
) -> CommandResult:
    """Decide the safe form of a drive command. Pure: no side effects.

    Args:
        linear: requested linear speed, m/s (+forward).
        angular: requested angular speed, rad/s (+left/CCW).
        duration: requested run time, seconds.
        front_distance: nearest obstacle in the forward sectors, metres,
            or None if unknown.
        lidar_connected: whether fresh lidar data is available.
        cfg: safety caps.
    """
    reasons: list[str] = []

    linear, lc = _clamp(linear, cfg.max_linear)
    angular, ac = _clamp(angular, cfg.max_angular)
    clamped = lc or ac
    if clamped:
        reasons.append("speed clamped to safety caps")

    duration = max(0.0, min(duration, cfg.cmd_timeout_s))

    moving_forward = linear > 0.0
    blocked = False
    # Lidar-based e-stop / fail-safe only applies when a lidar is fitted.
    if moving_forward and cfg.lidar_enabled:
        if not lidar_connected or front_distance is None:
            blocked = True
            reasons.append("forward motion blocked: lidar unavailable (fail-safe)")
        elif front_distance < cfg.estop_distance_m:
            blocked = True
            reasons.append(
                f"forward motion blocked: obstacle {front_distance:.2f} m ahead "
                f"(< {cfg.estop_distance_m:.2f} m e-stop)"
            )

    if blocked:
        # Refuse forward component; allow no motion at all (caller halts).
        linear = 0.0
        angular = 0.0

    return CommandResult(
        linear=linear,
        angular=angular,
        duration=duration,
        blocked=blocked,
        reason="; ".join(reasons) if reasons else None,
        clamped=clamped,
    )


class SafetySupervisor:
    """Stateful wrapper: evaluates commands, drives motors, enforces deadman."""

    def __init__(self, motors, lidar, cfg: Config):
        self._motors = motors
        self._lidar = lidar
        self._cfg = cfg
        self._timer: Optional[threading.Timer] = None
        self._lock = threading.Lock()
        self.last_result: Optional[CommandResult] = None

    def drive(self, linear: float, angular: float, duration: float) -> CommandResult:
        connected = False
        front = None
        if self._lidar is not None:
            summary = self._lidar.get_summary()
            connected = self._lidar.is_connected()
            if summary:
                # Lazy import to avoid a hard dep cycle; pure helper.
                from bridge.scan_summary import min_forward_distance
                front = min_forward_distance(summary.get("sectors", {}))

        result = evaluate_command(
            linear, angular, duration, front, connected, self._cfg
        )

        with self._lock:
            self._cancel_timer()
            if result.blocked or (result.linear == 0.0 and result.angular == 0.0):
                self._motors.stop()
            else:
                self._motors.send(result.linear, result.angular)
                # Deadman: auto-stop after the (capped) duration.
                self._timer = threading.Timer(result.duration, self._deadman_stop)
                self._timer.daemon = True
                self._timer.start()
            self.last_result = result
        return result

    def stop(self) -> None:
        with self._lock:
            self._cancel_timer()
            self._motors.stop()

    # -- internal -----------------------------------------------------------
    def _deadman_stop(self) -> None:
        with self._lock:
            self._motors.stop()
            self._timer = None

    def _cancel_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
