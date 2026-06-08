# OpenClaw Agent — Design

**Date:** 2026-06-08
**Status:** Approved (pending spec review)

## Purpose

An LLM-driven agent that runs alongside a Raspberry Pi robot. It lets a human
operator, through a web chat, ask about the robot's status and command its
motion in natural language. The model (DeepSeek V4 Pro via OpenRouter) interprets
instructions and calls a small, safe set of robot tools.

## Hardware / existing stack (discovered on the Pi)

- **Brain:** Raspberry Pi 5, Ubuntu 24.04, Python 3.12, Docker, 8 GB RAM.
  Host `192.168.50.250`, user `raspberrypi`.
- **Lidar:** 3iRobotics **Delta-2A** on `/dev/ttyUSB0` (CP2102 bridge @ 230400 baud,
  range 0.05–8.0 m). ROS2 driver `~/delta2a_lidar_ros2` publishes `sensor_msgs/LaserScan`
  on topic `/scan`.
- **ROS2:** Jazzy, runs **only inside Docker** — container `ros2_jazzy_3irobotics`
  (`network_mode: host`, `ROS_DOMAIN_ID=42`, mounts `/dev/ttyUSB0`, privileged).
  No native ROS2 on the host. The lidar driver owns the serial port while running,
  so nothing else may read `/dev/ttyUSB0` directly.
- **Motors:** a separate **TRIK** controller at `192.168.50.248`. The Pi commands it:
  - **UDP** to `192.168.50.248:5005`, ASCII payload `"<linear>,<angular>"`
    (e.g. `"0.5,0.0"`); stop = `"0.0,0.0"`. (Primary path; from `motorcrtludp.py`.)
  - ROS2 `/cmd_vel` `geometry_msgs/Twist` (standard alternative).
  - TRIK also sends telemetry back to the Pi via UDP (`0.0.0.0:5005`).

## Key constraint

The agent must read lidar `/scan`, which only exists inside the ROS2 Docker
container. To keep the agent itself plain Python (so identical code runs on a
Windows dev machine **and** on the Pi), we do **not** put the agent in ROS2.
Instead a tiny ROS2 bridge node re-publishes scan data onto the LAN as JSON.

## Architecture

Three deployable pieces:

### 1. `scan_bridge` (ROS2 node, runs in the existing Docker container, on the Pi)

- Subscribes `/scan` (`LaserScan`, `ROS_DOMAIN_ID=42`).
- On each scan, reduces the full point list to a compact summary and broadcasts
  it as a JSON datagram over **UDP `:5006`** on the LAN (broadcast/known host).
- JSON payload (one datagram per scan, ~20 Hz capped to e.g. 10 Hz):
  ```json
  {
    "ts": 1733650000.12,
    "sectors": { "front": 0.84, "front_left": 1.20, "left": 2.10,
                 "rear_left": 3.0, "rear": 2.4, "rear_right": 1.9,
                 "right": 0.7, "front_right": 0.55 },
    "nearest": { "distance": 0.55, "angle_deg": 312.0 },
    "num_points": 240,
    "range_max": 8.0
  }
  ```
- Sectors: 8 wedges of 45° each, value = min valid range in that wedge (metres),
  `null` if no return. `front` centred on 0°/360° (robot heading).
- Only ROS-coupled code in the system. ~60 lines. Added under `bridge/`.

### 2. OpenClaw backend (plain Python, runs anywhere on the LAN)

Modules under `agent/`:

- **`config.py`** — loads `.env`. Fields: `OPENROUTER_API_KEY`, `OPENROUTER_BASE_URL`
  (default `https://openrouter.ai/api/v1`), `OPENCLAW_MODEL`
  (default `deepseek/deepseek-v4-pro`), `TRIK_IP` (`192.168.50.248`), `TRIK_PORT`
  (`5005`), `BRIDGE_PORT` (`5006`), and safety caps:
  `MAX_LINEAR` (e.g. 0.6), `MAX_ANGULAR` (e.g. 1.0), `CMD_TIMEOUT_S` (e.g. 2.0),
  `ESTOP_DISTANCE_M` (e.g. 0.35).
- **`hardware/lidar.py`** — `LidarClient`: background thread listening on UDP
  `:5006`, keeps the latest scan summary + a `last_seen` timestamp. Exposes
  `get_summary()` and `is_connected()` (fresh within N seconds).
- **`hardware/motors.py`** — `MotorClient`: `send(linear, angular)` → UDP datagram
  to TRIK; `stop()` → `"0.0,0.0"`. Stateless, fire-and-forget.
- **`safety.py`** — `SafetySupervisor` wraps `MotorClient`:
  - Clamps `linear`/`angular` to configured maxima.
  - **Deadman / command timeout:** every motion command runs for at most
    `duration` (capped at `CMD_TIMEOUT_S`); a background watchdog sends `stop()`
    when it elapses, and refuses to "hold" motion indefinitely.
  - **Lidar e-stop:** if commanded `linear > 0` and the lidar `front` (or
    `front_left`/`front_right`) sector is below `ESTOP_DISTANCE_M`, the forward
    command is blocked / converted to stop, and the reason is reported back.
  - Hard `stop()` always passes through immediately.
- **`state.py`** — shared in-memory robot state (last command, last status,
  e-stop flag, connection health) for the web UI and tools to read.
- **`tools.py`** — tool schemas (OpenAI function-calling format) + handlers:
  - `get_status()` → returns lidar sectors, nearest obstacle, connection health,
    last command, safety state. Read-only.
  - `drive(linear: float, angular: float, duration: float)` → through
    `SafetySupervisor`. Returns what actually happened (clamped values, blocked?).
  - `stop()` → immediate halt.
- **`llm.py`** — `Agent`: OpenRouter client (OpenAI SDK pointed at base URL).
  System prompt describes the robot, the tools, the safety rules, and units
  (linear m/s, angular rad/s, +linear = forward, +angular = left/CCW).
  Runs the tool-calling loop: user message → model → tool calls → execute →
  feed results back → final natural-language reply. Streams reply tokens + tool
  events to the web layer.
- **`server.py`** — FastAPI app:
  - Serves the static web UI.
  - `GET /api/status` — current robot status (JSON).
  - `WS /ws` — bidirectional: client sends chat messages; server streams agent
    replies, tool-call events, and periodic status pushes.
  - `POST /api/estop` — hard stop, independent of the LLM (the red button).

### 3. Web UI (`web/`, vanilla HTML/CSS/JS, no build step)

Single page, clean and functional:

- **Chat panel** — conversation with the agent; shows tool calls inline
  ("🔧 drive(0.3, 0.0, 1.5) → moved forward").
- **Status panel** — live lidar sector readout (8 sectors, colour-coded by
  distance), nearest obstacle, connection indicators (lidar bridge / TRIK / model),
  last command.
- **E-STOP button** — large, always visible; calls `POST /api/estop` directly
  (does not wait on the model).

## Data flow

```
Operator → web chat → WS → Agent loop → DeepSeek (OpenRouter) → tool call
        → SafetySupervisor → MotorClient → UDP → TRIK → motors
Lidar → /scan (ROS2 docker) → scan_bridge → UDP :5006 → LidarClient
        → state → /api/status + WS push → web UI
```

## Safety model

1. **Speed caps** — every `drive` clamped to `MAX_LINEAR` / `MAX_ANGULAR`.
2. **Command timeout (deadman)** — motion auto-stops after `duration`
   (≤ `CMD_TIMEOUT_S`); no open-ended motion.
3. **Lidar e-stop** — forward motion blocked when an obstacle is within
   `ESTOP_DISTANCE_M` ahead.
4. **Manual e-stop** — web button + `stop()` tool, both bypass the model.
5. **Fail-safe on disconnect** — if the lidar bridge goes stale while moving
   forward, the supervisor stops (treat unknown as blocked).

## Error handling

- **No lidar data:** `get_status` reports `lidar: disconnected`; forward motion
  is refused by the supervisor (fail-safe). Other tools still work.
- **TRIK unreachable:** UDP is fire-and-forget, so sends never block; status shows
  TRIK link unknown. (No ACK channel in v1.)
- **Model/API errors:** surfaced in chat as an error message; no motion issued.
- **Malformed tool args:** validated in `tools.py`; rejected with a clear message
  fed back to the model.

## Configuration & deployment

- All secrets/params in `.env` (`.env.example` committed, `.env` gitignored).
- **Dev (Windows):** run the backend locally; it reaches TRIK and the scan_bridge
  over the LAN when the Pi is powered and the bridge is running. No mock layer.
- **Prod (Pi):** clone the repo, create `.env`, run `scan_bridge` in the ROS2
  container and the backend on the host (systemd unit or `run` script).
- Identical agent code in both environments; only `.env` differs.

## Testing

- **Unit:** `SafetySupervisor` clamping, e-stop logic, deadman timeout (with a
  fake clock + fake MotorClient that records datagrams). Sector summarisation in
  the bridge (pure function over a synthetic `LaserScan`). Tool arg validation.
- **Integration (local, no hardware):** a small UDP "fake TRIK" that records
  received datagrams + a fake bridge that emits canned scan JSON, to exercise
  `LidarClient`/`MotorClient` and the status endpoint.
- **Manual on Pi:** chat commands ("creep forward", "what's around you?",
  "stop") with the robot on blocks/wheels off the ground first.

## Out of scope (v1, YAGNI)

- Autonomous navigation / path planning / SLAM (only reactive safety).
- Mapping or persistent state across sessions.
- Auth on the web UI (LAN-local; add later if exposed).
- Closed-loop odometry / TRIK telemetry ACK.
- Multi-robot, voice, mobile app.

## Tech stack

Python 3.12 · FastAPI · uvicorn · openai SDK (→ OpenRouter) · pydantic ·
pyserial-free agent (serial only inside the ROS2 bridge, which uses the existing
driver — the bridge itself only needs `rclpy`). Frontend: vanilla HTML/CSS/JS.
