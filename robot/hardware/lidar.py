"""LidarClient — receives sector summaries from scan_bridge over UDP.

A background thread listens on BRIDGE_PORT and keeps the most recent scan
summary (the JSON produced by bridge/scan_summary.summarize_scan). Thread-safe
reads via get_summary(); freshness via is_connected().
"""
from __future__ import annotations

import json
import socket
import threading
import time
from typing import Optional

from ..config import Config


class LidarClient:
    def __init__(self, cfg: Config):
        self._cfg = cfg
        self._port = cfg.bridge_port
        self._stale_s = cfg.lidar_stale_s
        self._lock = threading.Lock()
        self._latest: Optional[dict] = None
        self._last_seen: float = 0.0
        self._stop = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", self._port))
        self._sock.settimeout(0.5)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._sock:
            self._sock.close()

    def get_summary(self) -> Optional[dict]:
        with self._lock:
            return self._latest

    def is_connected(self) -> bool:
        with self._lock:
            return (time.time() - self._last_seen) <= self._stale_s and self._latest is not None

    # -- internal -----------------------------------------------------------
    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                data = json.loads(raw.decode())
            except (ValueError, UnicodeDecodeError):
                continue
            with self._lock:
                self._latest = data
                self._last_seen = time.time()
