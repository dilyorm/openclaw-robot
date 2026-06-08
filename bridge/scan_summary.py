"""Pure lidar-scan -> sector-summary logic.

No ROS / no hardware deps so it is importable on any machine and unit-testable.
Used by bridge/scan_bridge.py (inside the ROS2 docker) to compress a full
LaserScan into a small JSON payload before broadcasting on the LAN.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence

# 8 sectors, each 45deg wide. `front` is centred on the robot heading (0deg).
SECTOR_NAMES = [
    "front",        # 337.5 .. 22.5
    "front_left",   # 22.5  .. 67.5
    "left",         # 67.5  .. 112.5
    "rear_left",    # 112.5 .. 157.5
    "rear",         # 157.5 .. 202.5
    "rear_right",   # 202.5 .. 247.5
    "right",        # 247.5 .. 292.5
    "front_right",  # 292.5 .. 337.5
]

# Sectors a forward (+linear) command can collide with.
FORWARD_SECTORS = ("front", "front_left", "front_right")


def sector_for(angle_deg: float) -> str:
    """Map an angle in degrees [0,360) to a sector name."""
    idx = int(((angle_deg + 22.5) % 360.0) // 45.0)
    return SECTOR_NAMES[idx]


def summarize_scan(
    angle_min: float,
    angle_increment: float,
    ranges: Sequence[Optional[float]],
    range_min: float = 0.05,
    range_max: float = 8.0,
) -> dict:
    """Reduce a LaserScan to per-sector minimum distances + nearest point.

    Args:
        angle_min: first sample angle, radians.
        angle_increment: angle step between samples, radians.
        ranges: distances in metres (None / out-of-range entries skipped).
        range_min/range_max: valid distance window in metres.

    Returns a JSON-serialisable dict.
    """
    sectors: dict[str, Optional[float]] = {name: None for name in SECTOR_NAMES}
    nearest_dist: Optional[float] = None
    nearest_angle: Optional[float] = None
    num_points = 0

    for i, r in enumerate(ranges):
        if r is None:
            continue
        try:
            r = float(r)
        except (TypeError, ValueError):
            continue
        if math.isnan(r) or math.isinf(r):
            continue
        if not (range_min <= r <= range_max):
            continue

        deg = math.degrees(angle_min + i * angle_increment) % 360.0
        name = sector_for(deg)
        cur = sectors[name]
        if cur is None or r < cur:
            sectors[name] = round(r, 3)
        if nearest_dist is None or r < nearest_dist:
            nearest_dist = r
            nearest_angle = deg
        num_points += 1

    nearest = None
    if nearest_dist is not None:
        nearest = {"distance": round(nearest_dist, 3), "angle_deg": round(nearest_angle, 1)}

    return {
        "sectors": sectors,
        "nearest": nearest,
        "num_points": num_points,
        "range_max": range_max,
    }


def min_forward_distance(sectors: dict) -> Optional[float]:
    """Smallest distance across the forward-facing sectors, or None if unknown."""
    vals = [sectors.get(s) for s in FORWARD_SECTORS]
    vals = [v for v in vals if v is not None]
    return min(vals) if vals else None
