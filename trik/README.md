# TRIK brick server

`trik_brick_server.py` runs **on the TRIK controller** (not the Pi), via TRIK
Studio's Python. It is the other half of the motor link: the Pi's MCP server
(`robot/hardware/motors.py`) is the TCP **client**; this is the **server**.

## Protocol
- TCP, port **9090**, single client.
- Pi → brick: `"<left>,<right>\n"` motor PWM, −100..100, + = forward (M3/M4).
- brick → Pi: `"<encL>,<encR>,<gyroZ>\n"` at ~20 Hz (E3/E4 + gyro Z).

## Run it
1. Open the file in **TRIK Studio**, connect to the controller.
2. Upload + run it on the brick (or run directly on the brick).
3. The display shows `Waiting for Pi...`, then `Link LIVE (no lidar)` once the
   Pi connects.

The Pi side connects automatically (it loops on `TRIK_IP:9090`, set to
`10.0.0.1:9090` over the direct ethernet link). Verify on the Pi with:
`ss -tnH dst 10.0.0.1 | grep 9090` → an `ESTAB` line means the link is up.

## Wiring
Wheels assumed on **M3/M4**, encoders on **E3/E4**. If your robot differs,
change those port names in `trik_brick_server.py`.
