"""MotorClient — sends velocity commands to the TRIK controller over UDP.

Wire format (matches the existing motorcrtludp.py on the Pi):
    "<linear>,<angular>"   e.g. "0.5,0.0"
Stop is "0.0,0.0". Fire-and-forget: UDP, no ACK.
"""
from __future__ import annotations

import socket

from ..config import Config


class MotorClient:
    def __init__(self, cfg: Config):
        self._addr = (cfg.trik_ip, cfg.trik_port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, linear: float, angular: float) -> None:
        payload = f"{linear:.3f},{angular:.3f}".encode()
        self._sock.sendto(payload, self._addr)

    def stop(self) -> None:
        self._sock.sendto(b"0.0,0.0", self._addr)

    def close(self) -> None:
        self._sock.close()
