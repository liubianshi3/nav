#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/a2_system_ws}"
IFACE="${A2_NETWORK_INTERFACE:-eth0}"
MAP_YAML="${A2_MAP_YAML:-${WORKSPACE}/runtime/maps/test_map_20260423_1059/map.yaml}"
WEB_SERVICE="${A2_WEB_SERVICE:-a2-web-console.service}"
WEB_URL="${A2_WEB_URL:-http://127.0.0.1:8080}"
START_SCRIPT="${WORKSPACE}/install/a2_system/share/a2_system/start_real_stack.sh"
STOP_SCRIPT="${WORKSPACE}/install/a2_system/share/a2_system/stop_stack.sh"
FRONTEND_BUILD_SCRIPT="${WORKSPACE}/web_console/scripts/build_frontend.sh"
STATIC_INDEX="${WORKSPACE}/web_console/backend/static/index.html"
RESIDUAL_PATTERN="bringup.launch.py|a2_sdk_bridge|a2_control_bridge|manual_localization_publisher|goal_bridge|occupancy_mapper|map_manager|map_server|controller_server|planner_server|bt_navigator|velocity_smoother|lifecycle_manager"
INTERFERENCE_CONTAINER="${A2_INTERFERENCE_CONTAINER:-festive_johnson}"

SET_INITIAL_POSE=0
POSE_X=""
POSE_Y=""
POSE_YAW="0.0"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--iface eth0] [--map-yaml /abs/path/map.yaml]
                     [--initial-pose X Y [YAW]]

What it does:
  1. Stops known interfering Docker/Web processes
  2. Clears old ROS stack processes
  3. Starts real1 (real bringup + Nav2)
  4. Starts Web backend
  5. Verifies health and optionally sends initial pose

Examples:
  $(basename "$0")
  $(basename "$0") --map-yaml /home/unitree/a2_system_ws/runtime/maps/test_map_20260423_1059/map.yaml
  $(basename "$0") --initial-pose -0.37 0.30 0.0
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface)
      IFACE="$2"
      shift 2
      ;;
    --map-yaml)
      MAP_YAML="$2"
      shift 2
      ;;
    --initial-pose)
      SET_INITIAL_POSE=1
      POSE_X="$2"
      POSE_Y="$3"
      if [[ $# -ge 4 ]] && [[ ! "$4" =~ ^-- ]]; then
        POSE_YAW="$4"
        shift 4
      else
        shift 3
      fi
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

log() {
  printf '[INFO] %s\n' "$*"
}

warn() {
  printf '[WARN] %s\n' "$*" >&2
}

die() {
  printf '[ERROR] %s\n' "$*" >&2
  exit 1
}

require_file() {
  local path="$1"
  [[ -f "$path" ]] || die "Missing required file: $path"
}

require_cmd() {
  local cmd="$1"
  command -v "$cmd" >/dev/null 2>&1 || die "Missing required command: $cmd"
}

wait_http_ok() {
  local url="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if curl -fsS "$url" >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_sec )); then
      return 1
    fi
    sleep 1
  done
}

wait_topic_ready() {
  local topic="$1"
  local pattern="$2"
  local timeout_sec="$3"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    local out
    out="$(ros2 topic echo --once "$topic" 2>/dev/null || true)"
    if [[ -n "$out" && "$out" == *"$pattern"* ]]; then
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_sec )); then
      return 1
    fi
    sleep 1
  done
}

cleanup_residuals() {
  pkill -f "bringup.launch.py" >/dev/null 2>&1 || true
  pkill -f "a2_sdk_bridge_node" >/dev/null 2>&1 || true
  pkill -f "a2_control_bridge_node" >/dev/null 2>&1 || true
  pkill -f "manual_localization_publisher" >/dev/null 2>&1 || true
  pkill -f "goal_bridge" >/dev/null 2>&1 || true
  pkill -f "occupancy_mapper" >/dev/null 2>&1 || true
  pkill -f "map_manager_node" >/dev/null 2>&1 || true
  pkill -f "map_server" >/dev/null 2>&1 || true
  pkill -f "controller_server" >/dev/null 2>&1 || true
  pkill -f "planner_server" >/dev/null 2>&1 || true
  pkill -f "bt_navigator" >/dev/null 2>&1 || true
  pkill -f "velocity_smoother" >/dev/null 2>&1 || true
  pkill -f "lifecycle_manager" >/dev/null 2>&1 || true
}

show_failure_context() {
  warn "Residual processes:"
  pgrep -af "$RESIDUAL_PATTERN" || true
  warn "Web service status:"
  systemctl status "$WEB_SERVICE" --no-pager || true
  warn "Recent web logs:"
  journalctl -u "$WEB_SERVICE" -n 80 --no-pager || true
  warn "Recent bringup logs:"
  ls -lt "${WORKSPACE}/runtime/logs"/bringup_real_*.log 2>/dev/null | head -3 || true
  local latest_log=""
  latest_log="$(ls -t "${WORKSPACE}/runtime/logs"/bringup_real_*.log 2>/dev/null | head -1 || true)"
  if [[ -n "$latest_log" ]]; then
    warn "Tail of ${latest_log}:"
    tail -n 120 "$latest_log" || true
  fi
}

require_cmd docker
require_cmd systemctl
require_cmd curl
require_cmd ros2
require_file "$START_SCRIPT"
require_file "$STOP_SCRIPT"
require_file "$MAP_YAML"
require_file "$FRONTEND_BUILD_SCRIPT"

log "workspace=${WORKSPACE}"
log "iface=${IFACE}"
log "map_yaml=${MAP_YAML}"

log "Stopping dockerized web stack if present"
(
  cd "$WORKSPACE"
  docker compose -f docker/docker-compose.a2.yml down
) >/dev/null 2>&1 || true

log "Stopping host web service"
sudo systemctl stop "$WEB_SERVICE" >/dev/null 2>&1 || true

if docker ps --format '{{.Names}}' | grep -Fxq "$INTERFERENCE_CONTAINER"; then
  log "Stopping known interference container: ${INTERFERENCE_CONTAINER}"
  docker update --restart=no "$INTERFERENCE_CONTAINER" >/dev/null 2>&1 || true
  docker stop "$INTERFERENCE_CONTAINER" >/dev/null 2>&1 || true
fi

log "Sourcing ROS environment"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

log "Stopping old ROS stack"
"$STOP_SCRIPT" >/dev/null 2>&1 || true
cleanup_residuals
sleep 2

if pgrep -af "$RESIDUAL_PATTERN" >/dev/null 2>&1; then
  show_failure_context
  die "Residual ROS stack processes still exist after cleanup"
fi

log "Starting real1 stack"
START_OUTPUT="$(
  A2_ENABLE_NAV2=true \
  A2_MAP_YAML="$MAP_YAML" \
  "$START_SCRIPT" "$IFACE" 2>&1
)" || {
  printf '%s\n' "$START_OUTPUT" >&2
  show_failure_context
  die "start_real_stack.sh failed"
}
printf '%s\n' "$START_OUTPUT"

log "Waiting for core ROS state"
if ! wait_topic_ready "/a2/real/report" "sdk=true" 25; then
  show_failure_context
  die "Timed out waiting for /a2/real/report"
fi

if [[ ! -f "$STATIC_INDEX" ]]; then
  log "Frontend static files missing, building frontend"
  if ! command -v node >/dev/null 2>&1; then
    die "Frontend build required but node is missing. Install Node.js on A2 first."
  fi
  if ! command -v npm >/dev/null 2>&1; then
    die "Frontend build required but npm is missing. Install npm on A2 first."
  fi
  if ! "$FRONTEND_BUILD_SCRIPT"; then
    show_failure_context
    die "Frontend build failed"
  fi
  [[ -f "$STATIC_INDEX" ]] || die "Frontend build finished but ${STATIC_INDEX} is still missing"
fi

log "Starting web service"
sudo systemctl start "$WEB_SERVICE"

if ! wait_http_ok "${WEB_URL}/api/health" 20; then
  show_failure_context
  die "Web API did not become ready at ${WEB_URL}/api/health"
fi

if (( SET_INITIAL_POSE == 1 )); then
  log "Sending initial pose x=${POSE_X} y=${POSE_Y} yaw=${POSE_YAW}"
  curl -fsS -X POST "${WEB_URL}/api/localization/initialpose" \
    -H "Content-Type: application/json" \
    -d "{\"pose\":{\"x\":${POSE_X},\"y\":${POSE_Y},\"yaw\":${POSE_YAW},\"frame_id\":\"map\"}}" >/tmp/a2_real1_initialpose.json || {
      cat /tmp/a2_real1_initialpose.json 2>/dev/null || true
      show_failure_context
      die "Failed to send initial pose"
    }
  cat /tmp/a2_real1_initialpose.json

  if ! wait_topic_ready "/a2/localization/status" "ready=true" 20; then
    show_failure_context
    die "Initial pose sent, but localization did not become ready"
  fi
fi

log "Final health"
curl -fsS "${WEB_URL}/api/health"
echo
log "Final real report"
ros2 topic echo --once /a2/real/report || true
log "Final localization status"
ros2 topic echo --once /a2/localization/status || true
log "Final control status"
ros2 topic echo --once /a2/control/status || true

echo
log "real1 suite started successfully"
log "Open: http://192.168.31.49:8080/"
