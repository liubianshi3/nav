#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/a2_system_ws}"
IFACE="${A2_NETWORK_INTERFACE:-eth0}"
MAP_YAML="${A2_MAP_YAML:-${WORKSPACE}/runtime/maps/test_map_20260423_1059/map.yaml}"
WEB_SERVICE="${A2_WEB_SERVICE:-a2-web-console.service}"
WEB_URL="${A2_WEB_URL:-http://127.0.0.1:8080}"
START_SCRIPT="${WORKSPACE}/install/a2_system/share/a2_system/start_real_stack.sh"
STOP_SCRIPT="${WORKSPACE}/install/a2_system/share/a2_system/stop_stack.sh"
BOOTSTRAP_BACKEND_SCRIPT="${WORKSPACE}/web_console/scripts/bootstrap_backend.sh"
FRONTEND_BUILD_SCRIPT="${WORKSPACE}/web_console/scripts/build_frontend.sh"
STATIC_INDEX="${WORKSPACE}/web_console/backend/static/index.html"
WEB_SERVICE_UNIT_FILE="${A2_WEB_SERVICE_UNIT_FILE:-${WORKSPACE}/web_console/systemd/a2-web-console.service}"
WEB_VENV_PYTHON="${WORKSPACE}/web_console/.venv/bin/python"
RESIDUAL_PATTERN="bringup.launch.py|a2_sdk_bridge|a2_control_bridge|task_manager.py|pointcloud_relay|pointcloud_to_laserscan|slam_toolbox|native_map_relay|localization_gate|manual_localization_publisher|amcl|goal_bridge|occupancy_mapper|map_manager|map_server|controller_server|smoother_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|lifecycle_manager"
INTERFERENCE_CONTAINER="${A2_INTERFERENCE_CONTAINER:-festive_johnson}"
GRAPH_PID_WS="${A2_GRAPH_PID_WS:-$HOME/graph_pid_ws}"
UNITREE_SLAM_SERVICE="${A2_UNITREE_SLAM_SERVICE:-unitree_slam.service}"
NATIVE_LIDAR_TOPIC="${A2_NATIVE_LIDAR_TOPIC:-/jt128/front/points}"
NATIVE_NAV_INTERFERENCE_PATTERN="${A2_NATIVE_NAV_INTERFERENCE_PATTERN:-navigation_mapping.py|dwa_obstacle_avoidance.py}"
ROS1_INTERFERENCE_PATTERN="${A2_ROS1_INTERFERENCE_PATTERN:-rosmaster|roslaunch x_nav_control|foxglove_bridge|a2_ros1_sdk}"
REAL_LIDAR_CONFIG="${WORKSPACE}/src/a2_system/config/real_lidar.yaml"
NETWORK_CONFIG="${WORKSPACE}/src/a2_system/config/network.yaml"

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
  1. Sanitizes native-lidar routing/config when using Unitree external pointcloud
  2. Starts the native lidar source if needed
  3. Stops known interfering Docker/Web processes
  4. Clears old ROS stack processes
  5. Starts real1 (real bringup + Nav2)
  6. Starts Web backend
  7. Verifies health and optionally sends initial pose

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
    out="$(timeout 2s ros2 topic echo --once "$topic" 2>/dev/null || true)"
    if [[ -n "$out" && "$out" == *"$pattern"* ]]; then
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_sec )); then
      return 1
    fi
    sleep 1
  done
}

wait_topic_message() {
  local topic="$1"
  local timeout_sec="$2"
  if timeout "${timeout_sec}"s ros2 topic echo --once "$topic" >/dev/null 2>&1; then
    return 0
  fi
  return 1
}

wait_action_ready() {
  local action_name="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if ros2 action list 2>/dev/null | grep -Fxq "$action_name"; then
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_sec )); then
      return 1
    fi
    sleep 1
  done
}

topic_exists() {
  local topic="$1"
  ros2 topic list 2>/dev/null | grep -Fxq "$topic"
}

read_real_lidar_mode() {
  python3 - "$REAL_LIDAR_CONFIG" <<'PY'
from pathlib import Path
import sys
import yaml

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
params = data.get("real_lidar", {}).get("ros__parameters", {}) or {}
print(str(params.get("profile", "") or ""))
print(str(params.get("driver_mode", "") or ""))
PY
}

read_sensor_host_addr() {
  python3 - "$NETWORK_CONFIG" <<'PY'
from pathlib import Path
import sys
import yaml

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
network = data.get("network", {}) or {}
print(str(network.get("mid360_host_ip", "") or ""))
print(str(network.get("mid360_prefix_len", 24) or 24))
PY
}

sanitize_native_lidar_multicast() {
  local cfg
  for cfg in \
    "${GRAPH_PID_WS}/config_files/hs_lidar_jt128/config.yaml" \
    "${GRAPH_PID_WS}/config_files/hs_lidar_jt128/config_new.yaml"; do
    [[ -f "$cfg" ]] || continue
    if grep -Eq 'multicast_ip_address:[[:space:]]*"?192\.168\.' "$cfg"; then
      cp "$cfg" "${cfg}.bak-$(date +%Y%m%d_%H%M%S)"
      sed -i 's/multicast_ip_address:.*/multicast_ip_address: ""  #multicast ip address/' "$cfg"
      log "Cleared invalid multicast_ip_address in $cfg"
    fi
  done
}

remove_conflicting_sensor_addr() {
  local sensor_ip="$1"
  local sensor_prefix="$2"
  if [[ -z "$sensor_ip" ]]; then
    return 0
  fi
  if ip -4 -o addr show dev "$IFACE" | awk '{print $4}' | grep -Fxq "${sensor_ip}/${sensor_prefix}"; then
    sudo ip addr del "${sensor_ip}/${sensor_prefix}" dev "$IFACE" || true
    log "Removed conflicting sensor subnet ${sensor_ip}/${sensor_prefix} from ${IFACE}"
  fi
}

prune_native_navigation_helpers() {
  pkill -f "navigation_mapping.py" >/dev/null 2>&1 || true
  pkill -f "dwa_obstacle_avoidance.py" >/dev/null 2>&1 || true
  sleep 1
  if pgrep -af "$NATIVE_NAV_INTERFERENCE_PATTERN" >/dev/null 2>&1; then
    warn "Native navigation helpers are still alive:"
    pgrep -af "$NATIVE_NAV_INTERFERENCE_PATTERN" || true
  else
    log "Stopped native navigation helper processes"
  fi
}

cleanup_residuals() {
  pkill -f "bringup.launch.py" >/dev/null 2>&1 || true
  pkill -f "a2_sdk_bridge_node" >/dev/null 2>&1 || true
  pkill -f "a2_control_bridge_node" >/dev/null 2>&1 || true
  pkill -f "task_manager.py" >/dev/null 2>&1 || true
  pkill -f "pointcloud_relay" >/dev/null 2>&1 || true
  pkill -f "pointcloud_to_laserscan" >/dev/null 2>&1 || true
  pkill -f "slam_toolbox" >/dev/null 2>&1 || true
  pkill -f "native_map_relay" >/dev/null 2>&1 || true
  pkill -f "localization_gate" >/dev/null 2>&1 || true
  pkill -f "manual_localization_publisher" >/dev/null 2>&1 || true
  pkill -f "amcl" >/dev/null 2>&1 || true
  pkill -f "goal_bridge" >/dev/null 2>&1 || true
  pkill -f "occupancy_mapper" >/dev/null 2>&1 || true
  pkill -f "map_manager_node" >/dev/null 2>&1 || true
  pkill -f "map_server" >/dev/null 2>&1 || true
  pkill -f "controller_server" >/dev/null 2>&1 || true
  pkill -f "smoother_server" >/dev/null 2>&1 || true
  pkill -f "planner_server" >/dev/null 2>&1 || true
  pkill -f "behavior_server" >/dev/null 2>&1 || true
  pkill -f "bt_navigator" >/dev/null 2>&1 || true
  pkill -f "waypoint_follower" >/dev/null 2>&1 || true
  pkill -f "velocity_smoother" >/dev/null 2>&1 || true
  pkill -f "lifecycle_manager" >/dev/null 2>&1 || true
}

cleanup_ros1_interference() {
  pkill -f "rosmaster" >/dev/null 2>&1 || true
  pkill -f "roslaunch x_nav_control" >/dev/null 2>&1 || true
  pkill -f "foxglove_bridge" >/dev/null 2>&1 || true
  pkill -f "a2_ros1_sdk" >/dev/null 2>&1 || true
}

ensure_web_backend_ready() {
  if [[ ! -x "$WEB_VENV_PYTHON" ]]; then
    log "Web backend virtualenv missing, bootstrapping backend"
    "$BOOTSTRAP_BACKEND_SCRIPT"
  fi
}

ensure_web_service_installed() {
  require_file "$WEB_SERVICE_UNIT_FILE"
  sudo install -m 644 "$WEB_SERVICE_UNIT_FILE" "/etc/systemd/system/${WEB_SERVICE}"
  sudo systemctl daemon-reload
  sudo systemctl enable "$WEB_SERVICE" >/dev/null 2>&1 || true
}

show_local_urls() {
  ip -4 -o addr show scope global | awk '{print $2 " " $4}' | while read -r name cidr; do
    case "$name" in
      lo|docker0|br-*|veth*)
        continue
        ;;
    esac
    printf '[INFO] Open: http://%s:8080/ (%s)\n' "${cidr%%/*}" "$name"
  done
}

show_failure_context() {
  warn "Residual processes:"
  pgrep -af "$RESIDUAL_PATTERN" || true
  warn "ROS1/native interference processes:"
  pgrep -af "$ROS1_INTERFERENCE_PATTERN|$NATIVE_NAV_INTERFERENCE_PATTERN" || true
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
require_file "$START_SCRIPT"
require_file "$STOP_SCRIPT"
require_file "$MAP_YAML"
require_file "$BOOTSTRAP_BACKEND_SCRIPT"
require_file "$FRONTEND_BUILD_SCRIPT"
require_file "$REAL_LIDAR_CONFIG"
require_file "$NETWORK_CONFIG"

log "Sourcing ROS environment"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

require_cmd ros2

log "workspace=${WORKSPACE}"
log "iface=${IFACE}"
log "map_yaml=${MAP_YAML}"

readarray -t REAL_LIDAR_MODE < <(read_real_lidar_mode)
REAL_LIDAR_PROFILE="${REAL_LIDAR_MODE[0]:-}"
REAL_LIDAR_DRIVER_MODE="${REAL_LIDAR_MODE[1]:-}"
USE_NATIVE_LIDAR_SOURCE=0
if [[ "${REAL_LIDAR_PROFILE}" == "unitree_native_fused" || "${REAL_LIDAR_DRIVER_MODE}" == "external_pointcloud" ]]; then
  USE_NATIVE_LIDAR_SOURCE=1
fi

if (( USE_NATIVE_LIDAR_SOURCE == 1 )); then
  readarray -t SENSOR_ADDR < <(read_sensor_host_addr)
  SENSOR_HOST_IP="${SENSOR_ADDR[0]:-}"
  SENSOR_PREFIX_LEN="${SENSOR_ADDR[1]:-24}"
  log "Using native lidar source topic=${NATIVE_LIDAR_TOPIC}"
  sanitize_native_lidar_multicast
  remove_conflicting_sensor_addr "${SENSOR_HOST_IP}" "${SENSOR_PREFIX_LEN}"
  if systemctl is-active --quiet "${UNITREE_SLAM_SERVICE}"; then
    log "Native lidar service already active"
  else
    log "Starting ${UNITREE_SLAM_SERVICE} for native lidar source"
    sudo systemctl start "${UNITREE_SLAM_SERVICE}"
    if ! wait_topic_message "${NATIVE_LIDAR_TOPIC}" 15; then
      systemctl status "${UNITREE_SLAM_SERVICE}" --no-pager || true
      journalctl -u "${UNITREE_SLAM_SERVICE}" -n 80 --no-pager || true
      die "Native lidar topic did not become active: ${NATIVE_LIDAR_TOPIC}"
    fi
  fi
  if topic_exists "${NATIVE_LIDAR_TOPIC}"; then
    log "Native lidar topic is visible"
  else
    warn "Native lidar topic is not visible before bringup; continuing because ${UNITREE_SLAM_SERVICE} is active"
  fi
  prune_native_navigation_helpers
fi

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

log "Stopping known ROS1/native interference processes"
cleanup_ros1_interference

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
  A2_REAL_LOCALIZATION_MODE=amcl \
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

if ! wait_topic_ready "/a2/lidar/status" "ready=true" 20; then
  show_failure_context
  die "Timed out waiting for /a2/lidar/status ready=true"
fi

if ! wait_topic_message "/scan" 20; then
  show_failure_context
  die "Timed out waiting for /scan"
fi

if ! wait_action_ready "/navigate_to_pose" 20; then
  show_failure_context
  die "Timed out waiting for /navigate_to_pose action"
fi

ensure_web_backend_ready

ensure_web_service_installed

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
show_local_urls
