# TRIK brick motor server — runs ON the TRIK controller (TRIK Studio Python).
#
# Upload + run this on the TRIK brick (NOT the Raspberry Pi). It opens a TCP
# server on port 9090, accepts motor commands "<left>,<right>\n" (PWM -100..100)
# from the Pi, and streams telemetry "<encL>,<encR>,<gyroZ>\n" at ~20 Hz.
#
# This is the lidar-free version: the lidar was removed, so brick.lidar() is NOT
# called (calling it with no lidar throws and stops the loop). Wheels = M3/M4,
# encoders = E3/E4 — adjust the port names if your wiring differs.

import socket

# Direct socket server.
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', 9090))
server.listen(1)

brick.display().clear()
brick.display().addLabel("Waiting for Pi...", 10, 10)
brick.display().redraw()

# One client at a time (the Pi MCP server).
conn, addr = server.accept()
conn.setblocking(False)

brick.display().clear()
brick.display().addLabel("Link LIVE (no lidar)", 10, 10)
brick.display().redraw()

brick.encoder("E3").reset()
brick.encoder("E4").reset()

while True:
    try:
        # 1. SEND TELEMETRY (encoders + gyro Z). No lidar on this robot.
        e3 = brick.encoder("E3").read()
        e4 = brick.encoder("E4").read()
        gz = brick.gyroscope().read()[2]   # Z-axis (rotation)

        # Format the Pi expects: "LeftEnc,RightEnc,GyroZ\n"
        conn.sendall(("%d,%d,%d\n" % (e3, e4, gz)).encode('utf-8'))

        # 2. RECEIVE MOTOR COMMAND from the Pi (non-blocking).
        try:
            cmd = conn.recv(1024).decode('utf-8')
            if cmd:
                # Use only the most recent line if several arrived.
                line = cmd.strip().split('\n')[-1]
                parts = line.split(',')
                if len(parts) == 2:
                    brick.motor("M3").setPower(int(parts[0]))
                    brick.motor("M4").setPower(int(parts[1]))
        except:
            pass  # no command this tick

        script.wait(50)  # ~20 Hz

    except Exception as e:
        # Connection lost / send failed -> stop the wheels and bail out.
        brick.motor("M3").powerOff()
        brick.motor("M4").powerOff()
        break
