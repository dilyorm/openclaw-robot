#!/usr/bin/env python3
"""scan_bridge — ROS2 node: /scan -> compact JSON over UDP on the LAN.

Runs INSIDE the ros2_jazzy_3irobotics docker container (where rclpy and the
/scan topic live). Subscribes to the Delta-2A LaserScan, reduces it to 8 sector
minimum-distances via summarize_scan(), and broadcasts the JSON to BRIDGE_HOST:
BRIDGE_PORT. The OpenClaw MCP server's LidarClient receives it.

Env:
    BRIDGE_HOST   target host/broadcast for the UDP datagrams (default 127.0.0.1)
    BRIDGE_PORT   target UDP port (default 5006)
    SCAN_TOPIC    topic to subscribe (default /scan)
    BRIDGE_HZ     max publish rate (default 10)

Run (inside the container):
    source /opt/ros/jazzy/setup.bash
    python3 bridge/scan_bridge.py
"""
from __future__ import annotations

import json
import os
import socket
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from scan_summary import summarize_scan


class ScanBridge(Node):
    def __init__(self):
        super().__init__("scan_bridge")
        self._host = os.getenv("BRIDGE_HOST", "127.0.0.1")
        self._port = int(os.getenv("BRIDGE_PORT", "5006"))
        topic = os.getenv("SCAN_TOPIC", "/scan")
        self._min_period = 1.0 / float(os.getenv("BRIDGE_HZ", "10"))
        self._last_sent = 0.0

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self._sub = self.create_subscription(LaserScan, topic, self._on_scan, 10)
        self.get_logger().info(
            f"scan_bridge: {topic} -> udp://{self._host}:{self._port} "
            f"@ max {1.0/self._min_period:.0f} Hz"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.time()
        if now - self._last_sent < self._min_period:
            return
        self._last_sent = now

        summary = summarize_scan(
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            ranges=list(msg.ranges),
            range_min=msg.range_min,
            range_max=msg.range_max,
        )
        summary["ts"] = now
        payload = json.dumps(summary).encode()
        try:
            self._sock.sendto(payload, (self._host, self._port))
        except OSError as e:
            self.get_logger().warn(f"UDP send failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ScanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
