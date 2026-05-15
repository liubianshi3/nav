#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/a2_system_ws}"
JT128_IFACE="${A2_JT128_INTERFACE:-net1}"
JT128_IP="${A2_JT128_IP:-192.168.124.20}"
GRAPH_PID_WS="${A2_GRAPH_PID_WS:-$HOME/graph_pid_ws}"
UNITREE_SLAM_SERVICE="${A2_UNITREE_SLAM_SERVICE:-unitree_slam.service}"
START_WEB=1
DRIVER_ONLY=0
ALLOW_MISSING_DLIO=0
MAP_ROOT="${A2_MAP_ROOT:-${WORKSPACE}/runtime/maps}"
LOG_DIR="${WORKSPACE}/runtime/logs"
STATE_FILE="${WORKSPACE}/runtime/jt128_dlio_stack_state.yaml"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--iface net1] [--driver-only] [--no-web] [--allow-missing-dlio]

Starts the JT128 + DLIO mapping stack:
  - stops Unitree native SLAM/DWA and old 2D mapping/localization interference
  - validates JT128 network reachability on the sensor interface
  - launches Hesai JT128 driver on /jt128/front/points and /jt128/front/imu
  - launches DLIO to publish /jt128/dlio/odom and /jt128/dlio/map_points
  - launches map_manager so Web/CLI can save pointcloud_map_3d.pcd

Install DLIO first when needed:
  ${WORKSPACE}/install/a2_system/share/a2_system/install_dlio_ros2.sh
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface)
      JT128_IFACE="$2"
      shift 2
      ;;
    --driver-only)
      DRIVER_ONLY=1
      shift
      ;;
    --no-web)
      START_WEB=0
      shift
      ;;
    --allow-missing-dlio)
      ALLOW_MISSING_DLIO=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] unknown argument: $1" >&2
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

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "missing command: $1"
}

run_privileged() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    "$@"
  fi
}

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f "${GRAPH_PID_WS}/install/setup.bash" ]]; then
    source "${GRAPH_PID_WS}/install/setup.bash"
  fi
  if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    source "${WORKSPACE}/install/setup.bash"
  fi
  if ! ros2 pkg prefix direct_lidar_inertial_odometry >/dev/null 2>&1 && \
    [[ -f "${WORKSPACE}/install/direct_lidar_inertial_odometry/share/direct_lidar_inertial_odometry/local_setup.bash" ]]; then
    source "${WORKSPACE}/install/direct_lidar_inertial_odometry/share/direct_lidar_inertial_odometry/local_setup.bash"
  fi
  set -u
}

kill_pattern() {
  local signal="$1"
  local pattern="$2"
  local pids=()
  local pid
  while IFS= read -r pid; do
    [[ -n "$pid" ]] || continue
    [[ "$pid" == "$$" || "$pid" == "$BASHPID" ]] && continue
    pids+=("$pid")
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
  ((${#pids[@]} > 0)) || return 0
  kill "-${signal}" "${pids[@]}" >/dev/null 2>&1 || true
  if [[ "$(id -u)" -ne 0 ]] && command -v sudo >/dev/null 2>&1; then
    sudo kill "-${signal}" "${pids[@]}" >/dev/null 2>&1 || true
  fi
}

stop_interference() {
  log "Stopping Unitree native SLAM service and known interference"
  if command -v systemctl >/dev/null 2>&1 && [[ ! -f /.dockerenv ]]; then
    run_privileged systemctl stop "$UNITREE_SLAM_SERVICE" >/dev/null 2>&1 || true
  fi
  local pattern
  for pattern in \
    "rosmaster" \
    "roslaunch x_nav_control" \
    "foxglove_bridge" \
    "foxglove_nodelet_manager" \
    "livox_ros_driver2_node" \
    "a2_ros1_sdk" \
    "navigation_mapping.py" \
    "dwa_obstacle_avoidance.py" \
    "point_cloud_fusion" \
    "hesai_ros_driver_node" \
    "unitree_slam" \
    "pointcloud_to_laserscan" \
    "slam_toolbox" \
    "amcl" \
    "map_server" \
    "controller_server" \
    "planner_server" \
    "bt_navigator" \
    "pointcloud_accumulator" \
    "octomap_server" \
    "octomap_mapping_node.py" \
    "dlio_odom_node" \
    "dlio_map_node" \
    "jt128_dlio_watchdog.py" \
    "map_manager_node"; do
    kill_pattern TERM "$pattern"
  done
  sleep 1
  for pattern in \
    "rosmaster" \
    "roslaunch x_nav_control" \
    "foxglove_bridge" \
    "foxglove_nodelet_manager" \
    "livox_ros_driver2_node" \
    "a2_ros1_sdk" \
    "navigation_mapping.py" \
    "dwa_obstacle_avoidance.py" \
    "point_cloud_fusion" \
    "hesai_ros_driver_node" \
    "unitree_slam" \
    "pointcloud_to_laserscan" \
    "slam_toolbox" \
    "amcl" \
    "pointcloud_accumulator" \
    "octomap_server" \
    "octomap_mapping_node.py" \
    "dlio_odom_node" \
    "dlio_map_node" \
    "jt128_dlio_watchdog.py" \
    "map_manager_node"; do
    kill_pattern KILL "$pattern"
  done
}

check_network() {
  ip link show "$JT128_IFACE" >/dev/null 2>&1 || die "interface not found: $JT128_IFACE"
  local iface_ip
  iface_ip="$(ip -4 -o addr show dev "$JT128_IFACE" scope global | awk '{print $4}' | cut -d/ -f1 | head -1)"
  [[ -n "$iface_ip" ]] || die "interface ${JT128_IFACE} has no IPv4 address"

  if ! ip route get "$JT128_IP" | grep -q "dev ${JT128_IFACE}"; then
    warn "JT128 route is not using ${JT128_IFACE}; installing host route for ${JT128_IP}/32"
    ip route get "$JT128_IP" || true
    run_privileged ip route replace "${JT128_IP}/32" dev "$JT128_IFACE" src "$iface_ip" || die "failed to install JT128 host route"
  fi
  ip route get "$JT128_IP" | grep -q "dev ${JT128_IFACE}" || {
    ip route get "$JT128_IP" || true
    die "JT128 route is still not using ${JT128_IFACE}"
  }
  ping -I "$JT128_IFACE" -c 2 -W 1 "$JT128_IP" >/dev/null || die "JT128 ${JT128_IP} is not reachable on ${JT128_IFACE}"
  log "JT128 reachable: ${JT128_IP} via ${JT128_IFACE}"
}

check_packages() {
  ros2 pkg prefix hesai_ros_driver >/dev/null 2>&1 || die "hesai_ros_driver is missing; source ${GRAPH_PID_WS}/install/setup.bash or install HesaiLidar_ROS_2.0"
  if [[ "$DRIVER_ONLY" -eq 0 ]]; then
    if ! ros2 pkg prefix direct_lidar_inertial_odometry >/dev/null 2>&1; then
      if [[ "$ALLOW_MISSING_DLIO" -eq 1 ]]; then
        warn "direct_lidar_inertial_odometry missing; continuing without DLIO"
      else
        die "direct_lidar_inertial_odometry missing. Run install_dlio_ros2.sh first, or use --driver-only for driver validation."
      fi
    fi
  fi
}

start_web() {
  if [[ "$START_WEB" -eq 0 ]]; then
    return 0
  fi
  if systemctl list-unit-files | grep -q '^a2-web-console.service'; then
    sudo systemctl restart a2-web-console.service || warn "failed to restart a2-web-console.service"
  else
    warn "a2-web-console.service is not installed; start Web manually if needed"
  fi
}

require_cmd ip
require_cmd ping
require_cmd pkill

mkdir -p "$LOG_DIR" "$MAP_ROOT"
source_ros
require_cmd ros2
run_privileged sysctl -w net.core.rmem_max=2147483647 >/dev/null 2>&1 || true
stop_interference
check_network
check_packages
start_web

START_DLIO=true
if [[ "$DRIVER_ONLY" -eq 1 ]] || ! ros2 pkg prefix direct_lidar_inertial_odometry >/dev/null 2>&1; then
  START_DLIO=false
fi

LOG_FILE="${LOG_DIR}/jt128_dlio_mapping_$(date +%Y%m%d_%H%M%S).log"
log "Starting JT128 DLIO mapping launch"
nohup bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  if [ -f '${GRAPH_PID_WS}/install/setup.bash' ]; then source '${GRAPH_PID_WS}/install/setup.bash'; fi
  source '${WORKSPACE}/install/setup.bash'
  export A2_WORKSPACE='${WORKSPACE}'
  ros2 launch a2_bringup dlio_mapping.launch.py \
    start_driver:=true \
    start_dlio:=${START_DLIO} \
    start_map_manager:=true \
    map_root:='${MAP_ROOT}' \
    use_sim_time:=false
" >"$LOG_FILE" 2>&1 &
PID=$!

cat > "$STATE_FILE" <<EOF
mode: jt128_dlio_mapping
pid: ${PID}
log_file: ${LOG_FILE}
jt128_interface: ${JT128_IFACE}
jt128_ip: ${JT128_IP}
start_dlio: ${START_DLIO}
map_root: ${MAP_ROOT}
started_at: $(date --iso-8601=seconds)
EOF

sleep 3
if ! kill -0 "$PID" >/dev/null 2>&1; then
  tail -80 "$LOG_FILE" >&2 || true
  die "JT128 DLIO launch exited early; see ${LOG_FILE}"
fi

log "Started JT128 DLIO mapping pid=${PID}"
log "Log file: ${LOG_FILE}"
log "Verify:"
log "  ros2 topic hz /jt128/front/points"
log "  ros2 topic hz /jt128/front/imu"
log "  ros2 topic info /jt128/dlio/odom"
log "  ros2 topic info /jt128/dlio/map_points"
log "Save PCD:"
log "  ros2 service call /map_manager/manage_map a2_interfaces/srv/ManageMap \"{command: save, map_id: jt128_test}\""
