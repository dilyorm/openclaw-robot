# TRIK brick motor server — runs ON the TRIK controller (TRIK Studio Python).
#
# Run this on the TRIK brick (NOT the Raspberry Pi). It opens a TCP server on
# port 9090, accepts motor commands "<left>,<right>\n" (PWM -100..100) from the
# Pi, and streams telemetry "<encL>,<encR>,<gyroZ>\n" at ~20 Hz.
#
# Lidar-free: brick.lidar() is NOT called (no lidar fitted; calling it would
# throw and stop the loop). Wheels = M3/M4, encoders = E3/E4.
#
# Re-accepts after a client disconnects, so it keeps running across Pi
# reconnects (the Pi's MCP server may respawn) instead of dying on first drop.

import socket

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', 9090))
server.listen(1)

while True:
    brick.display().clear()
    brick.display().addLabel("Waiting for Pi...", 10, 10)
    brick.display().redraw()

    conn, addr = server.accept()
    conn.setblocking(False)

    brick.display().clear()
    brick.display().addLabel("Link LIVE (no lidar)", 10, 10)
    brick.display().redraw()

    brick.encoder("E3").reset()
    brick.encoder("E4").reset()

    while True:
        # 1. SEND TELEMETRY. A send failure means the client is gone.
        try:
            e3 = brick.encoder("E3").read()
            e4 = brick.encoder("E4").read()
            gz = brick.gyroscope().read()[2]   # Z-axis (rotation)
            conn.sendall(("%d,%d,%d\n" % (e3, e4, gz)).encode('utf-8'))
        except:
            break  # disconnected -> stop motors and re-accept

        # 2. RECEIVE MOTOR COMMAND (non-blocking).
        try:
            cmd = conn.recv(1024).decode('utf-8')
            if cmd == '':
                break  # peer closed the connection
            line = cmd.strip().split('\n')[-1]
            parts = line.split(',')
            if len(parts) == 2:
                brick.motor("M3").setPower(int(parts[0]))
                brick.motor("M4").setPower(int(parts[1]))
        except:
            pass  # no command this tick (non-blocking recv)

        script.wait(50)  # ~20 Hz

    # Client gone: stop the wheels, close, loop back to accept the next one.
    brick.motor("M3").powerOff()
    brick.motor("M4").powerOff()
    try:
        conn.close()
    except:
        pass
