"""Unit tests for the linear/angular -> differential PWM mapping."""
from robot.config import Config
from robot.hardware.motors import mix_to_pwm


def make_cfg(**over):
    base = dict(
        trik_ip="127.0.0.1",
        trik_port=9090,
        bridge_port=5006,
        max_linear=0.6,
        max_angular=1.0,
        cmd_timeout_s=2.0,
        estop_distance_m=0.35,
        lidar_stale_s=1.5,
        lidar_enabled=False,
        fwd_pwm=60,
        turn_pwm=40,
    )
    base.update(over)
    return Config(**base)


def test_stop_is_zero():
    assert mix_to_pwm(0.0, 0.0, make_cfg()) == (0, 0)


def test_full_forward_maps_to_fwd_pwm():
    # linear at the cap -> both wheels at fwd_pwm, equal.
    assert mix_to_pwm(0.6, 0.0, make_cfg()) == (60, 60)


def test_full_reverse():
    assert mix_to_pwm(-0.6, 0.0, make_cfg()) == (-60, -60)


def test_left_turn_in_place():
    # +angular = turn left: left wheel back, right wheel forward.
    left, right = mix_to_pwm(0.0, 1.0, make_cfg())
    assert left == -40
    assert right == 40


def test_right_turn_in_place():
    left, right = mix_to_pwm(0.0, -1.0, make_cfg())
    assert left == 40
    assert right == -40


def test_forward_arc_left():
    # forward + slight left -> right wheel faster than left, both forward-ish.
    left, right = mix_to_pwm(0.6, 0.5, make_cfg())  # fwd=60, turn=20
    assert (left, right) == (40, 80)


def test_pwm_clipped_to_100():
    # Beyond the cap (caller normally clamps first) still clips at +/-100.
    left, right = mix_to_pwm(2.0, 0.0, make_cfg())  # fwd would be 200
    assert left == 100 and right == 100
