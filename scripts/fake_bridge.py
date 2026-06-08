#!/usr/bin/env python3
"""Emit canned scan JSON on the bridge UDP port — for local testing w/o lidar.

Lets you exercise the MCP server's get_status / drive e-stop without the Pi.
    python scripts/fake_bridge.py            # clear all around (forward allowed)
    python scripts/fake_bridge.py --front 0.2  # obstacle ahead (forward blocked)
"""
import argparse
import json
import socket
import time

SECTORS = ["front", "front_left", "left", "rear_left", "rear", "rear_right", "right", "front_right"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5006)
    ap.add_argument("--front", type=float, default=2.0, help="front distance (m)")
    ap.add_argument("--default", type=float, default=3.0, help="other sectors (m)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Sending fake scans to {args.host}:{args.port} (front={args.front} m). Ctrl+C to stop.")
    while True:
        sectors = {s: args.default for s in SECTORS}
        sectors["front"] = args.front
        payload = {
            "ts": time.time(),
            "sectors": sectors,
            "nearest": {"distance": min(sectors.values()), "angle_deg": 0.0},
            "num_points": 240,
            "range_max": 8.0,
        }
        sock.sendto(json.dumps(payload).encode(), (args.host, args.port))
        time.sleep(0.1)


if __name__ == "__main__":
    main()
