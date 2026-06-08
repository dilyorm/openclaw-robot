# OpenClaw Robot

Natural-language control + status for a Raspberry Pi lidar robot, using the
[OpenClaw](https://docs.openclaw.ai) agent framework with **DeepSeek V4 Pro via
OpenRouter** as the model. You chat with OpenClaw ("what's around you?", "creep
forward", "stop"); it calls robot tools we expose over MCP.

## How it fits together

```
You ── OpenClaw chat (dashboard / Telegram) ── DeepSeek V4 Pro (OpenRouter)
                                                      │  MCP tool calls
                                                      ▼
                                        robot MCP server  (robot/mcp_server.py)
                                          ├─ get_status ─ lidar sectors, health
                                          ├─ drive ─ safety-checked motion ─► UDP ─► TRIK motors
                                          └─ stop  ─ immediate halt
                                                      ▲
                          UDP :5006  ◄── scan_bridge (ROS2 docker) ◄── /scan (Delta-2A lidar)
```

- **OpenClaw** — installed via npm; provides the model wiring, chat UI, agent loop.
- **robot MCP server** (`robot/`) — our custom tools + all safety logic.
- **scan_bridge** (`bridge/`) — tiny ROS2 node, runs in the existing lidar docker
  container, re-publishes `/scan` as compact JSON over UDP so the (ROS-free) MCP
  server can read it.

The robot: Pi 5 (Ubuntu 24.04), 3iRobotics **Delta-2A** lidar, **TRIK** motor
controller at `192.168.50.248` (UDP `"linear,angular"` on :5005).

## Safety

Every `drive` goes through `robot/safety.py`:
- speed clamped to `MAX_LINEAR` / `MAX_ANGULAR`;
- **deadman** — auto-stops after `duration` (≤ `CMD_TIMEOUT_S`);
- **lidar e-stop** — forward motion blocked within `ESTOP_DISTANCE_M`;
- **fail-safe** — stale/missing lidar blocks forward motion;
- `stop` always halts immediately.

## Layout

```
robot/                MCP server (our code, plain Python)
  config.py           hardware + safety params from .env
  hardware/lidar.py   UDP listener for scan_bridge
  hardware/motors.py  UDP sender to TRIK
  safety.py           clamp + e-stop + deadman
  mcp_server.py       FastMCP tools: get_status, drive, stop
bridge/
  scan_summary.py     pure /scan -> 8-sector summary (tested)
  scan_bridge.py      ROS2 node (runs in the lidar docker)
openclaw.config.json5 OpenClaw config template (model + MCP server)
AGENTS.md             robot operating brief -> agent system prompt
scripts/
  run_bridge.sh       launch scan_bridge in the docker container
  fake_bridge.py      emit canned scans for local testing
tests/                unit tests (summary + safety)
```

## Local dev (no robot)

```bash
python -m venv .venv && .venv/Scripts/activate   # Windows
# source .venv/bin/activate                       # Linux/macOS
pip install -r requirements.txt
pytest                                            # 19 unit tests

# exercise the live MCP server without hardware:
python scripts/fake_bridge.py --front 2.0 &       # fake clear lidar
# then point an MCP client (or OpenClaw) at:  python -m robot.mcp_server
```

With a fake clear scan, `drive(0.3,0,1)` succeeds; with `--front 0.2` it's blocked.

## Deploy on the Pi

```bash
# 1. Code
git clone https://github.com/dilyorm/openclaw-robot.git
cd openclaw-robot
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env            # adjust TRIK/safety if needed

# 2. Lidar bridge (in the existing ROS2 container)
./scripts/run_bridge.sh         # keep running; or wrap in systemd/tmux

# 3. OpenClaw
npm install -g openclaw@latest  # Node 22.19+/24
openclaw onboard
#   put your config at ~/.openclaw/openclaw.json based on openclaw.config.json5:
#   - set OPENROUTER_API_KEY
#   - fix the absolute paths (workspace, MCP command/cwd) to this repo
openclaw mcp doctor openclaw-robot --probe   # verify the 3 tools load
openclaw dashboard                            # chat at http://127.0.0.1:18789/
```

Then chat: *"what do you see?"*, *"creep forward half a metre"*, *"turn left"*,
*"stop"*. **Test with the wheels off the ground first.**

## Configuration

- Robot/safety params: `.env` (see `.env.example`).
- Model, OpenRouter key, chat channels, MCP registration: `~/.openclaw/openclaw.json`
  (template: `openclaw.config.json5`). Model id: `openrouter/deepseek/deepseek-v4-pro`.
