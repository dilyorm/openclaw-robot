"""Unit tests for the pure lidar-summary logic."""
import math

from bridge.scan_summary import (
    SECTOR_NAMES,
    min_forward_distance,
    sector_for,
    summarize_scan,
)


def test_sector_for_cardinals():
    assert sector_for(0.0) == "front"
    assert sector_for(359.0) == "front"
    assert sector_for(45.0) == "front_left"
    assert sector_for(90.0) == "left"
    assert sector_for(180.0) == "rear"
    assert sector_for(270.0) == "right"
    assert sector_for(315.0) == "front_right"


def test_sector_boundaries_wrap():
    # 22.5 is the front/front_left boundary -> falls into front_left
    assert sector_for(22.5) == "front_left"
    # just under wraps back into front
    assert sector_for(22.49) == "front"
    assert sector_for(337.5) == "front"


def _scan_from_degrees(points):
    """Build (angle_min, angle_increment, ranges) putting one return per degree."""
    ranges = [0.0] * 360  # 0.0 -> out of range, skipped
    for deg, dist in points:
        ranges[deg] = dist
    angle_min = 0.0
    angle_increment = math.radians(1.0)
    return angle_min, angle_increment, ranges


def test_summarize_picks_min_per_sector():
    angle_min, inc, ranges = _scan_from_degrees(
        [(0, 1.0), (1, 0.5), (90, 2.0), (90, 2.0), (180, 3.0)]
    )
    out = summarize_scan(angle_min, inc, ranges)
    assert out["sectors"]["front"] == 0.5  # min of 1.0 and 0.5
    assert out["sectors"]["left"] == 2.0
    assert out["sectors"]["rear"] == 3.0
    assert out["sectors"]["right"] is None  # nothing there
    assert out["num_points"] == 4


def test_summarize_nearest():
    angle_min, inc, ranges = _scan_from_degrees([(10, 1.2), (200, 0.4), (300, 0.9)])
    out = summarize_scan(angle_min, inc, ranges)
    assert out["nearest"]["distance"] == 0.4
    assert out["nearest"]["angle_deg"] == 200.0


def test_summarize_filters_out_of_range_and_nan():
    angle_min, inc, ranges = _scan_from_degrees(
        [(0, 0.01), (1, 99.0), (2, float("nan")), (3, float("inf")), (4, 1.0)]
    )
    out = summarize_scan(angle_min, inc, ranges, range_min=0.05, range_max=8.0)
    # only the 1.0 at deg 4 is valid
    assert out["num_points"] == 1
    assert out["sectors"]["front"] == 1.0


def test_summarize_all_sectors_present():
    out = summarize_scan(0.0, math.radians(1.0), [0.0] * 360)
    assert set(out["sectors"].keys()) == set(SECTOR_NAMES)
    assert all(v is None for v in out["sectors"].values())
    assert out["nearest"] is None


def test_min_forward_distance():
    sectors = {"front": 1.0, "front_left": 0.6, "front_right": None, "left": 0.1}
    assert min_forward_distance(sectors) == 0.6  # ignores `left`, ignores None
    assert min_forward_distance({"front": None, "front_left": None, "front_right": None}) is None
