#!/usr/bin/env bash
# Run scan_bridge inside the existing ROS2 docker container on the Pi.
# The bridge subscribes /scan and broadcasts sector JSON to BRIDGE_HOST:BRIDGE_PORT.
#
# Container has host networking, so 127.0.0.1 reaches the MCP server on the Pi host.
set -euo pipefail

CONTAINER="${CONTAINER:-ros2_jazzy_3irobotics}"
BRIDGE_HOST="${BRIDGE_HOST:-127.0.0.1}"
BRIDGE_PORT="${BRIDGE_PORT:-5006}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"

echo "Ensuring container '$CONTAINER' is running..."
docker start "$CONTAINER" >/dev/null

echo "Copying bridge into container..."
docker cp "$HERE/bridge/scan_summary.py" "$CONTAINER:/tmp/scan_summary.py"
docker cp "$HERE/bridge/scan_bridge.py"  "$CONTAINER:/tmp/scan_bridge.py"

echo "Launching scan_bridge ($SCAN_TOPIC -> udp://$BRIDGE_HOST:$BRIDGE_PORT)..."
docker exec -i \
  -e BRIDGE_HOST="$BRIDGE_HOST" \
  -e BRIDGE_PORT="$BRIDGE_PORT" \
  -e SCAN_TOPIC="$SCAN_TOPIC" \
  "$CONTAINER" \
  bash -lc 'source /opt/ros/jazzy/setup.bash && cd /tmp && python3 scan_bridge.py'
