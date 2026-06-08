# OpenClaw Robot Agent — Design

**Date:** 2026-06-08
**Status:** Approved (OpenClaw-runtime architecture)

## Purpose

Run the **OpenClaw** agent framework on a Raspberry Pi robot so an operator can,
in natural language, ask about the robot's status and command its motion.
OpenClaw provides the model wiring (DeepSeek V4 Pro via OpenRouter), the chat
interfaces (web dashboard, Telegram, etc.) and the agent loop. We supply only the
robot-specific capabilities as a **Model Context Protocol (MCP) server**, plus a
tiny ROS2 bridge so the agent can read the lidar.

## Why OpenClaw instead of a hand-rolled agent

OpenClaw is a model-agnostic agent runtime (`npm i -g openclaw`) with built-in
chat UIs, multi-channel messaging, and first-class MCP tool support. Building a
bespoke LLM loop + web UI would duplicate all of that. We keep our custom surface
minimal: the hardware/safety logic, exposed as MCP tools, and a lidar bridge.

## Hardware / existing stack (discovered on the Pi)

- **Brain:** Raspberry Pi 5, Ubuntu 24.04, Python 3.12, Node available, Docker,
  8 GB RAM. Host `192.168.50.250`, user `raspberrypi`.
- **Lidar:** 3iRobotics **Delta-2A** on `/dev/ttyUSB0` (CP2102 @ 230400 baud,
  range 0.05–8.0 m). ROS2 driver `~/delta2a_lidar_ros2` publishes
  `sensor_msgs/LaserScan` on `/scan`.
- **ROS2:** Jazzy, runs **only inside Docker** — container `ros2_jazzy_3irobotics`
  (`network_mode: host`, `ROS_DOMAIN_ID=42`, mounts `/dev/ttyUSB0`, privileged).
  The driver owns the serial port; nothing else may read it directly.
- **Motors:** separate **TRIK** controller at `192.168.50.248`. Commanded by:
  - **UDP** to `192.168.50.248:5005`, ASCII `"<linear>,<angular>"`
    (e.g. `"0.5,0.0"`); stop = `"0.0,0.0"`.
  - (ROS2 `/cmd_vel` Twist also exists; we use UDP.)

## Architecture

Three pieces. OpenClaw is installed (not built); pieces 1 and 2 are our code.

### 1. `scan_bridge` — ROS2 node, runs in the existing Docker container (Pi)

- Subscribes `/scan` (`LaserScan`, `ROS_DOMAIN_ID=42`).
- Reduces each scan to 8 sector minimum-distances + nearest point via the pure
  `summarize_scan()` function, broadcasts as JSON over **UDP `:5006`** (rate-capped
  to ~10 Hz).
- Only ROS-coupled code. `bridge/scan_bridge.py` (rclpy) + `bridge/scan_summary.py`
  (pure, no deps — shared/testable).
- JSON payload:
  ```json
  {"ts": 1733650000.1, "sectors": {"front": 0.84, "front_left": 1.2, "left": 2.1,
   "rear_left": 3.0, "rear": 2.4, "rear_right": 1.9, "right": 0.7,
   "front_right": 0.55}, "nearest": {"distance": 0.55, "angle_deg": 312.0},
   "num_points": 240, "range_max": 8.0}
  ```

### 2. Robot MCP server — Python stdio process (our custom capability layer)

`robot/mcp_server.py` (built on the `mcp` Python SDK / FastMCP). OpenClaw launches
it as a stdio MCP server and exposes its tools to the model. Modules under `robot/`:

- **`config.py`** — `.env` config: `TRIK_IP`/`TRIK_PORT`, `BRIDGE_PORT`, and safety
  caps `MAX_LINEAR`, `MAX_ANGULAR`, `CMD_TIMEOUT_S`, `ESTOP_DISTANCE_M`,
  `LIDAR_STALE_S`. (Model/key live in OpenClaw config, not here.)
- **`hardware/lidar.py`** — `LidarClient`: background thread on UDP `:5006`, keeps
  latest scan summary + `last_seen`. `get_summary()`, `is_connected()`.
- **`hardware/motors.py`** — `MotorClient`: `send(linear, angular)` UDP→TRIK;
  `stop()`→`"0.0,0.0"`. Fire-and-forget.
- **`safety.py`** — pure `evaluate_command(...)` (clamp + e-stop decision) plus
  `SafetySupervisor` wrapping `MotorClient`:
  - clamps to maxima; caps `duration` at `CMD_TIMEOUT_S`;
  - **deadman:** background watchdog `stop()`s when `duration` elapses;
  - **lidar e-stop:** blocks forward motion when forward sectors < `ESTOP_DISTANCE_M`,
    or when lidar data is stale/disconnected (fail-safe);
  - `stop()` always passes through immediately.
- **`mcp_server.py`** — defines 3 MCP tools:
  - `get_status()` → lidar sectors, nearest obstacle, connection health, last
    command, safety state (read-only).
  - `drive(linear, angular, duration)` → via `SafetySupervisor`; returns what
    actually happened (clamped values, blocked + reason).
  - `stop()` → immediate halt.
  Units in tool docstrings: linear m/s (+forward), angular rad/s (+left/CCW).

### 3. OpenClaw configuration (`openclaw.config.json5`)

- `env.OPENROUTER_API_KEY` (user fills in later).
- `agents.defaults.model.primary = "openrouter/deepseek/deepseek-v4-pro"`.
- `mcp.servers["openclaw-robot"]` → `command: "python"`, `args: ["-m", "robot.mcp_server"]`,
  `cwd` = repo path, `env` for the robot `.env` values.
- Agent system prompt: describes the robot, the tools, safety rules, and that it
  must call `stop` when unsure or asked to halt.

## Data flow

```
Operator → OpenClaw chat (dashboard / Telegram) → DeepSeek V4 Pro (OpenRouter)
        → MCP tool call (drive/stop/get_status) → SafetySupervisor
        → MotorClient → UDP → TRIK → motors
Lidar → /scan (ROS2 docker) → scan_bridge → UDP :5006 → LidarClient
        → get_status → back to the model
```

## Safety model

1. Speed caps on every `drive`.
2. Deadman: motion auto-stops after `duration` (≤ `CMD_TIMEOUT_S`).
3. Lidar e-stop: forward motion blocked when an obstacle is within
   `ESTOP_DISTANCE_M`.
4. Fail-safe: stale/disconnected lidar blocks forward motion.
5. `stop` tool always bypasses to an immediate halt.

## Error handling

- No lidar data → `get_status` reports `lidar: disconnected`; forward motion
  refused. Other tools still work.
- TRIK unreachable → UDP fire-and-forget never blocks; no ACK in v1.
- MCP server crash → OpenClaw reports the tool as unavailable; no motion issued.
- Bad tool args → validated in the MCP layer; rejected with a clear message.

## Deployment

- **Local (Windows dev):** build/lint/test the MCP server and bridge logic. The
  MCP server can run against real TRIK + bridge over the LAN when the Pi is on.
  OpenClaw itself is exercised on the Pi.
- **Pi (prod):**
  1. `git clone` the repo, `pip install -r requirements.txt` (in a venv).
  2. Create `.env` (robot params) and the OpenClaw config (key + model + MCP server).
  3. Run `scan_bridge` inside the ROS2 docker container.
  4. `npm i -g openclaw`; `openclaw onboard`; start the dashboard / channels.
  OpenClaw spawns the MCP server automatically.

## Testing

- **Unit:** `summarize_scan` / `sector_for` / `min_forward_distance`;
  `evaluate_command` (clamping, e-stop, fail-safe); deadman timeout with a fake
  clock + fake MotorClient; tool arg validation.
- **Integration (local, no hardware):** fake-TRIK UDP sink + fake bridge emitting
  canned scan JSON to exercise `LidarClient`/`MotorClient` and `get_status`.
- **Manual on Pi:** chat commands ("creep forward", "what's around you?", "stop")
  with wheels off the ground first.

## Out of scope (v1, YAGNI)

Autonomous nav / SLAM / mapping; odometry / TRIK telemetry ACK; auth (LAN-local);
multi-robot; custom web UI (use OpenClaw's). Reactive safety only.

## Tech stack

OpenClaw (Node) · Python 3.12 · `mcp` SDK (FastMCP) · python-dotenv · pytest.
ROS2 bridge uses `rclpy` (already present in the docker image). No FastAPI / no
custom frontend.
