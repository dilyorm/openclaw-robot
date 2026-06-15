"""Unit tests for safety: command evaluation + deadman supervisor."""
import threading
import time

from robot.config import Config
from robot.safety import SafetySupervisor, evaluate_command


def make_cfg(**over):
    base = dict(
        trik_ip="127.0.0.1",
        trik_port=5005,
        bridge_port=5006,
        max_linear=0.6,
        max_angular=1.0,
        cmd_timeout_s=2.0,
        estop_distance_m=0.35,
        lidar_stale_s=1.5,
        lidar_enabled=True,
        fwd_pwm=60,
        turn_pwm=40,
    )
    base.update(over)
    return Config(**base)


# -- evaluate_command (pure) -----------------------------------------------

def test_clamps_to_caps():
    r = evaluate_command(5.0, 9.0, 1.0, front_distance=2.0, lidar_connected=True, cfg=make_cfg())
    assert r.linear == 0.6
    assert r.angular == 1.0
    assert r.clamped is True
    assert r.blocked is False


def test_negative_clamp():
    r = evaluate_command(-5.0, -9.0, 1.0, front_distance=None, lidar_connected=True, cfg=make_cfg())
    assert r.linear == -0.6  # reverse not blocked by front obstacle
    assert r.angular == -1.0


def test_duration_capped():
    r = evaluate_command(0.0, 0.5, 99.0, front_distance=2.0, lidar_connected=True, cfg=make_cfg())
    assert r.duration == 2.0


def test_estop_blocks_forward_when_obstacle_close():
    r = evaluate_command(0.3, 0.0, 1.0, front_distance=0.2, lidar_connected=True, cfg=make_cfg())
    assert r.blocked is True
    assert r.linear == 0.0
    assert r.angular == 0.0
    assert "obstacle" in r.reason


def test_forward_allowed_when_clear():
    r = evaluate_command(0.3, 0.0, 1.0, front_distance=2.0, lidar_connected=True, cfg=make_cfg())
    assert r.blocked is False
    assert r.linear == 0.3


def test_failsafe_blocks_forward_when_lidar_down():
    r = evaluate_command(0.3, 0.0, 1.0, front_distance=None, lidar_connected=False, cfg=make_cfg())
    assert r.blocked is True
    assert "lidar unavailable" in r.reason


def test_reverse_allowed_when_lidar_down():
    r = evaluate_command(-0.3, 0.0, 1.0, front_distance=None, lidar_connected=False, cfg=make_cfg())
    assert r.blocked is False
    assert r.linear == -0.3


def test_pure_turn_allowed_when_obstacle_ahead():
    # No forward component -> not blocked even with an obstacle in front.
    r = evaluate_command(0.0, 0.8, 1.0, front_distance=0.1, lidar_connected=True, cfg=make_cfg())
    assert r.blocked is False
    assert r.angular == 0.8


# -- no-lidar deployment (lidar_enabled=False) -----------------------------

def test_no_lidar_forward_allowed():
    # Robot without a lidar: forward motion is NOT blocked by the fail-safe.
    cfg = make_cfg(lidar_enabled=False)
    r = evaluate_command(0.3, 0.0, 1.0, front_distance=None, lidar_connected=False, cfg=cfg)
    assert r.blocked is False
    assert r.linear == 0.3


def test_no_lidar_still_clamps_and_caps_duration():
    cfg = make_cfg(lidar_enabled=False)
    r = evaluate_command(5.0, 0.0, 99.0, front_distance=None, lidar_connected=False, cfg=cfg)
    assert r.linear == 0.6        # still clamped
    assert r.duration == 2.0      # still deadman-capped
    assert r.blocked is False


def test_supervisor_drives_without_lidar():
    # SafetySupervisor with no lidar object must still send motor commands.
    motors = FakeMotors()
    sup = SafetySupervisor(motors, None, make_cfg(lidar_enabled=False))
    r = sup.drive(0.3, 0.0, 0.05)
    assert r.blocked is False
    assert ("send", 0.3, 0.0) in motors.calls
    sup.stop()


# -- SafetySupervisor (stateful) -------------------------------------------

class FakeMotors:
    def __init__(self):
        self.calls = []
        self.lock = threading.Lock()

    def send(self, linear, angular):
        with self.lock:
            self.calls.append(("send", round(linear, 3), round(angular, 3)))

    def stop(self):
        with self.lock:
            self.calls.append(("stop",))


class FakeLidar:
    def __init__(self, summary, connected=True):
        self._summary = summary
        self._connected = connected

    def get_summary(self):
        return self._summary

    def is_connected(self):
        return self._connected


def clear_sectors(dist=5.0):
    names = ["front", "front_left", "left", "rear_left", "rear", "rear_right", "right", "front_right"]
    return {"sectors": {n: dist for n in names}}


def test_supervisor_sends_when_clear():
    motors = FakeMotors()
    sup = SafetySupervisor(motors, FakeLidar(clear_sectors()), make_cfg())
    r = sup.drive(0.3, 0.0, 0.05)
    assert r.blocked is False
    assert ("send", 0.3, 0.0) in motors.calls
    sup.stop()


def test_supervisor_blocks_and_stops_on_obstacle():
    motors = FakeMotors()
    blocked = clear_sectors()
    blocked["sectors"]["front"] = 0.1
    sup = SafetySupervisor(motors, FakeLidar(blocked), make_cfg())
    r = sup.drive(0.3, 0.0, 1.0)
    assert r.blocked is True
    assert motors.calls == [("stop",)]


def test_deadman_auto_stops_after_duration():
    motors = FakeMotors()
    sup = SafetySupervisor(motors, FakeLidar(clear_sectors()), make_cfg())
    sup.drive(0.3, 0.0, 0.15)
    time.sleep(0.3)  # wait past the deadman duration
    assert ("send", 0.3, 0.0) in motors.calls
    assert motors.calls[-1] == ("stop",)


def test_stop_passthrough():
    motors = FakeMotors()
    sup = SafetySupervisor(motors, FakeLidar(clear_sectors()), make_cfg())
    sup.stop()
    assert motors.calls == [("stop",)]
