"""MotorClient — persistent TCP link to the TRIK controller.

Direct ethernet link (Pi eth0 <-> TRIK). Wire protocol (matches the proven
trik_motor_client.py / trik_bridge_daemon.py on the Pi):

    send:  "<left>,<right>\\n"   left/right = motor PWM, -100..100, +=forward
    stop:  "0,0\\n"
    recv:  "<left_enc>,<right_enc>,<gyro_z>[:<scan>]\\n"  (telemetry, line-based)

The agent speaks linear (m/s, +forward) / angular (rad/s, +left/CCW); we map
that onto differential PWM here so the rest of the stack is unchanged. A
background thread keeps the socket connected and parses telemetry.
"""
from __future__ import annotations

import socket
import threading
import time

from ..config import Config


def mix_to_pwm(linear: float, angular: float, cfg: Config) -> tuple[int, int]:
    """Map linear/angular onto (left, right) motor PWM, each clamped to +/-100.

    linear is scaled by fwd_pwm relative to max_linear; angular by turn_pwm
    relative to max_angular. Pure + deterministic, so it is unit-tested.
    """
    fwd = (linear / cfg.max_linear) * cfg.fwd_pwm if cfg.max_linear else 0.0
    turn = (angular / cfg.max_angular) * cfg.turn_pwm if cfg.max_angular else 0.0
    left = fwd - turn   # +angular (left turn) -> left wheel back, right wheel fwd
    right = fwd + turn

    def _clip(v: float) -> int:
        return int(max(-100, min(100, round(v))))

    return _clip(left), _clip(right)


class MotorClient:
    """Holds a persistent TCP connection to TRIK; reconnects on failure."""

    def __init__(self, cfg: Config):
        self._cfg = cfg
        self._addr = (cfg.trik_ip, cfg.trik_port)
        self._sock: socket.socket | None = None
        self._running = False
        self._lock = threading.Lock()
        self._reader: threading.Thread | None = None

        # Latest telemetry.
        self.left_enc = 0
        self.right_enc = 0
        self.gyro_z = 0
        self.last_rx = 0.0
        self._state_lock = threading.Lock()

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        """Begin the background connect/read loop."""
        if self._running:
            return
        self._running = True
        self._reader = threading.Thread(target=self._connect_loop, daemon=True)
        self._reader.start()

    def _connect_loop(self) -> None:
        while self._running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect(self._addr)
                sock.settimeout(0.5)
                with self._lock:
                    self._sock = sock
                self._read_until_dead(sock)
            except OSError:
                pass  # connect failed / link down; retry below
            finally:
                with self._lock:
                    if self._sock is not None:
                        try:
                            self._sock.close()
                        except OSError:
                            pass
                        self._sock = None
            if self._running:
                time.sleep(2.0)  # backoff before reconnect

    def _read_until_dead(self, sock: socket.socket) -> None:
        buffer = ""
        while self._running:
            try:
                data = sock.recv(4096).decode("utf-8", "ignore")
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return  # peer closed
            buffer += data
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                self._parse_telemetry(line)

    def _parse_telemetry(self, line: str) -> None:
        # "enc_l,enc_r,gyro_z" optionally followed by ":<scan>".
        enc = line.split(":", 1)[0]
        parts = enc.split(",")
        if len(parts) >= 3:
            try:
                l, r, gz = int(parts[0]), int(parts[1]), int(parts[2])
            except ValueError:
                return
            with self._state_lock:
                self.left_enc, self.right_enc, self.gyro_z = l, r, gz
                self.last_rx = time.time()

    # -- commands -----------------------------------------------------------
    def _send_raw(self, payload: bytes) -> bool:
        with self._lock:
            sock = self._sock
        if sock is None:
            return False
        try:
            sock.sendall(payload)
            return True
        except OSError:
            return False

    def send(self, linear: float, angular: float) -> None:
        left, right = mix_to_pwm(linear, angular, self._cfg)
        self._send_raw(f"{left},{right}\n".encode())

    def stop(self) -> None:
        self._send_raw(b"0,0\n")

    # -- status -------------------------------------------------------------
    def is_connected(self) -> bool:
        with self._lock:
            return self._sock is not None

    def get_state(self) -> dict:
        with self._state_lock:
            return {
                "connected": self.is_connected(),
                "left_enc": self.left_enc,
                "right_enc": self.right_enc,
                "gyro_z": self.gyro_z,
                "telemetry_age_s": (round(time.time() - self.last_rx, 2)
                                    if self.last_rx else None),
            }

    def close(self) -> None:
        self._running = False
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
