#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/ws/device-navigation}"
JT128_IFACE="${A2_JT128_INTERFACE:-net1}"
JT128_IP="${A2_JT128_IP:-192.168.124.20}"
GRAPH_PID_WS="${A2_GRAPH_PID_WS:-}"
ALLOW_GRAPH_PID_WS="${A2_ALLOW_GRAPH_PID_WS:-0}"
UNITREE_SLAM_SERVICE="${A2_UNITREE_SLAM_SERVICE:-unitree_slam.service}"
START_WEB=1
DRIVER_ONLY=0
ALLOW_MISSING_DLIO=0
OCTOMAP_REQUESTED=true
MAP_ROOT="${A2_MAP_ROOT:-${WORKSPACE}/runtime/maps}"
LOG_DIR="${WORKSPACE}/runtime/logs"
STATE_FILE="${WORKSPACE}/runtime/jt128_dlio_stack_state.yaml"
FAST_DDS_TRANSPORTS="${A2_FASTDDS_BUILTIN_TRANSPORTS:-${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}}"
ROS_RMW_IMPLEMENTATION="${A2_RMW_IMPLEMENTATION:-${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}}"
EFFECTIVE_GRAPH_PID_WS=""
case "${ALLOW_GRAPH_PID_WS,,}" in
  1|true|yes|on)
    EFFECTIVE_GRAPH_PID_WS="${GRAPH_PID_WS}"
    ;;
esac

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--iface net1] [--driver-only] [--no-web] [--allow-missing-dlio] [--no-octomap] [--start-octomap]

Starts the JT128 + DLIO mapping stack:
  - stops Unitree native SLAM/DWA and old 2D mapping/localization interference
  - validates JT128 network reachability on the sensor interface
  - launches Hesai JT128 driver on /jt128/front/points and /jt128/front/imu
  - launches DLIO to publish /jt128/dlio/odom and /jt128/dlio/map_points
  - when DLIO is enabled, launches OctoMap mapping and saves ${MAP_ROOT}/octomap_live.bt
  - launches map_manager so Web/CLI can save pointcloud_map_3d.pcd

Notes:
  - normal mapping mode starts OctoMap by default (pass --no-octomap to skip)
  - --no-octomap disables OctoMap launch (used by navigation mode to save CPU)
  - --start-octomap explicitly enables OctoMap launch (default for standalone mapping)
  - --driver-only, or missing DLIO with --allow-missing-dlio, disables both DLIO and OctoMap

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
    --no-octomap)
      OCTOMAP_REQUESTED=false
      shift
      ;;
    --start-octomap)
      OCTOMAP_REQUESTED=true
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

export_child_ros_env() {
  printf 'export RMW_IMPLEMENTATION=%q\n' "${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    printf 'export CYCLONEDDS_URI=%q\n' "${CYCLONEDDS_URI}"
  fi
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
  if [[ -n "${GRAPH_PID_WS}" && -z "${EFFECTIVE_GRAPH_PID_WS}" ]]; then
    warn "Ignoring A2_GRAPH_PID_WS=${GRAPH_PID_WS}; set A2_GRAPH_PID_WS explicitly with A2_ALLOW_GRAPH_PID_WS=1 only for external overlay debugging"
  fi
  if [[ -n "${EFFECTIVE_GRAPH_PID_WS}" && -f "${EFFECTIVE_GRAPH_PID_WS}/install/setup.bash" ]]; then
    source "${EFFECTIVE_GRAPH_PID_WS}/install/setup.bash"
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

configure_ros_transport() {
  export RMW_IMPLEMENTATION="${ROS_RMW_IMPLEMENTATION}"
  unset FASTDDS_BUILTIN_TRANSPORTS
  log "Using ROS RMW implementation: ${RMW_IMPLEMENTATION}"
}

reset_ros2_daemon() {
  timeout 4 ros2 daemon stop >/dev/null 2>&1 || true
}

wait_topic_message() {
  local topic="$1"
  local timeout_sec="$2"
  local attempt
  for attempt in 1 2; do
    reset_ros2_daemon
    if timeout "${timeout_sec}" ros2 topic echo --once --qos-reliability best_effort "$topic" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

require_a2_system_executable() {
  local name="$1"
  local install_path="${WORKSPACE}/install/a2_system/lib/a2_system/${name}"
  local source_path="${WORKSPACE}/src/a2_system/scripts/${name}"
  if [[ -x "$install_path" ]]; then
    return 0
  fi
  if [[ -x "$source_path" ]]; then
    warn "install executable missing for ${name}; launch will fall back to source path ${source_path}"
    return 0
  fi
  die "required a2_system executable is unavailable: ${name} (checked ${install_path} and ${source_path})"
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
  local INTERFERENCE_PATTERNS=(
    "rosmaster"
    "roslaunch x_nav_control"
    "foxglove_bridge"
    "foxglove_nodelet_manager"
    "livox_ros_driver2_node"
    "a2_ros1_sdk"
    "navigation_mapping.py"
    "dwa_obstacle_avoidance.py"
    "point_cloud_fusion"
    "pointcloud_preview_node.py"
    "dlio_mapping.launch.py"
    "jt128_driver.launch.py"
    "jt128_3d_navigation.launch.py"
    "hesai_ros_driver_node"
    "imu_to_si_converter.py"
    "unitree_slam"
    "pointcloud_to_laserscan"
    "slam_toolbox"
    "amcl"
    "map_server"
    "controller_server"
    "planner_server"
    "bt_navigator"
    "smoother_server"
    "behavior_server"
    "waypoint_follower"
    "velocity_smoother"
    "lifecycle_manager"
    "local_costmap"
    "global_costmap"
    "pointcloud_accumulator"
    "pointcloud_guard"
    "pointcloud_map_loader"
    "octomap_server"
    "octomap_server_node"
    "octomap_saver_node"
    "octomap_mapping_node.py"
    "dlio_odom_node"
    "dlio_map_node"
    "odometry_tf_broadcaster.py"
    "jt128_dlio_watchdog.py"
    "map_manager_node"
    "static_tf_manager"
    "ndt_scan_matcher"
    "autoware_ndt_scan_matcher_node"
    "ndt_adapter"
    "ndt_health_monitor"
    "pcd_relocalizer_3d"
    "sensor_covariance_injector.py"
    "body_imu_covariance_injector"
    "ekf_node"
    "localization_gate"
    "goal_bridge"
    "pose_goal_controller_3d"
    "ground_segmentation_cpp_node"
    "traversability_to_obstacle_cloud.py"
    "global_traversability_integrator"
    "collision_monitor"
    "auto_scan_mission.py"
    "task_manager.py"
    "real_readiness_monitor"
    "safety_supervisor"
    "a2_sdk_bridge_node"
    "a2_state_publisher_node"
    "a2_control_bridge_node"
  )
  for pattern in "${INTERFERENCE_PATTERNS[@]}"; do
    kill_pattern TERM "$pattern"
  done
  sleep 1
  for pattern in "${INTERFERENCE_PATTERNS[@]}"; do
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
  ros2 pkg prefix hesai_ros_driver >/dev/null 2>&1 || die "hesai_ros_driver is missing from ${WORKSPACE}; install/build hesai_ros_driver there. External graph_pid overlay is disabled unless A2_ALLOW_GRAPH_PID_WS=1."
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
require_cmd ss

mkdir -p "$LOG_DIR" "$MAP_ROOT"
source_ros
configure_ros_transport
require_cmd ros2
require_a2_system_executable "octomap_mapping_node.py"
require_a2_system_executable "pointcloud_preview_node.py"
run_privileged sysctl -w net.core.rmem_max=2147483647 >/dev/null 2>&1 || true
stop_interference
check_network
check_packages
if ss -H -lun | grep -Eq '(^|[[:space:]])[^[:space:]]*:2368[[:space:]]'; then
  warn "UDP port 2368 is already bound; another JT128/Hesai driver may be running"
  ss -lunp | grep -E '(^|[[:space:]])[^[:space:]]*:2368[[:space:]]' >&2 || true
  die "JT128 UDP port 2368 is occupied; stop the other mapping/navigation stack before starting this one"
fi
start_web

START_DLIO=true
if [[ "$DRIVER_ONLY" -eq 1 ]] || ! ros2 pkg prefix direct_lidar_inertial_odometry >/dev/null 2>&1; then
  START_DLIO=false
fi
REQUEST_START_OCTOMAP="${OCTOMAP_REQUESTED}"
EFFECTIVE_START_OCTOMAP=false
if [[ "$START_DLIO" == "true" && "$OCTOMAP_REQUESTED" == "true" ]]; then
  EFFECTIVE_START_OCTOMAP=true
fi

LOG_FILE="${LOG_DIR}/jt128_dlio_mapping_$(date +%Y%m%d_%H%M%S).log"
log "Starting JT128 DLIO mapping launch"
nohup bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  if [ -n '${EFFECTIVE_GRAPH_PID_WS}' ] && [ -f '${EFFECTIVE_GRAPH_PID_WS}/install/setup.bash' ]; then source '${EFFECTIVE_GRAPH_PID_WS}/install/setup.bash'; fi
  source '${WORKSPACE}/install/setup.bash'
  export A2_WORKSPACE='${WORKSPACE}'
  $(export_child_ros_env)
  unset FASTDDS_BUILTIN_TRANSPORTS
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
requested_start_octomap: ${REQUEST_START_OCTOMAP}
effective_start_octomap: ${EFFECTIVE_START_OCTOMAP}
map_root: ${MAP_ROOT}
started_at: $(date --iso-8601=seconds)
EOF

sleep 3
if ! kill -0 "$PID" >/dev/null 2>&1; then
  tail -80 "$LOG_FILE" >&2 || true
  die "JT128 DLIO launch exited early; see ${LOG_FILE}"
fi
if grep -Eiq "bind failed|open udp source failed|\\[FATAL\\]" "$LOG_FILE"; then
  tail -80 "$LOG_FILE" >&2 || true
  die "JT128 Hesai driver failed to bind UDP source; see ${LOG_FILE}"
fi

log "Waiting for first JT128 pointcloud"
if ! wait_topic_message /jt128/front/points 12; then
  grep -Eiq "bind failed|open udp source failed|\\[FATAL\\]" "$LOG_FILE" && tail -80 "$LOG_FILE" >&2 || true
  die "JT128 pointcloud /jt128/front/points did not publish after two 12s checks; check sensor packets and UDP port 2368"
fi

OCTOMAP_PID=""
OCTOMAP_LOG_FILE=""
if [[ "$START_DLIO" == "true" && "$OCTOMAP_REQUESTED" == "true" ]]; then
  OCTOMAP_LOG_FILE="${LOG_DIR}/octomap_mapping_$(date +%Y%m%d_%H%M%S).log"
  log "Starting OctoMap mapping launch"
  nohup bash -lc "
    set -e
    source /opt/ros/humble/setup.bash
    if [ -n '${EFFECTIVE_GRAPH_PID_WS}' ] && [ -f '${EFFECTIVE_GRAPH_PID_WS}/install/setup.bash' ]; then source '${EFFECTIVE_GRAPH_PID_WS}/install/setup.bash'; fi
    source '${WORKSPACE}/install/setup.bash'
    export A2_WORKSPACE='${WORKSPACE}'
    $(export_child_ros_env)
    ros2 launch a2_bringup octomap_mapping.launch.py \
      use_sim_time:=false \
      odom_topic:=/jt128/dlio/odom \
      cloud_topic:=/jt128/front/points \
      save_path:='${MAP_ROOT}/octomap_live.bt'
  " >"$OCTOMAP_LOG_FILE" 2>&1 &
  OCTOMAP_PID=$!

  sleep 3
  if ! kill -0 "$OCTOMAP_PID" >/dev/null 2>&1; then
    tail -80 "$OCTOMAP_LOG_FILE" >&2 || true
    die "OctoMap mapping launch exited early; see ${OCTOMAP_LOG_FILE}"
  fi
  log "Started OctoMap mapping pid=${OCTOMAP_PID}"
  log "OctoMap log file: ${OCTOMAP_LOG_FILE}"
elif [[ "$START_DLIO" != "true" ]]; then
  warn "Skipping OctoMap mapping because DLIO is disabled"
else
  log "Skipping OctoMap mapping because --no-octomap was requested"
fi

cat >> "$STATE_FILE" <<EOF
octomap_pid: ${OCTOMAP_PID:-null}
octomap_log_file: ${OCTOMAP_LOG_FILE:-null}
octomap_save_path: ${MAP_ROOT}/octomap_live.bt
EOF

log "Started JT128 DLIO mapping pid=${PID}"
log "Log file: ${LOG_FILE}"
log "Verify:"
log "  ros2 topic hz /jt128/front/points"
log "  ros2 topic hz /jt128/front/imu"
log "  ros2 topic hz /jt128/front/points_preview"
if [[ "$START_DLIO" == "true" ]]; then
  log "  ros2 topic info /jt128/dlio/odom"
  log "  ros2 topic info /jt128/dlio/map_points"
  log "  ros2 topic hz /jt128/dlio/map_points_preview"
  log "  ros2 topic info /octomap_binary"
  log "  ros2 topic info /octomap_full"
  log "  ros2 topic info /projected_map"
  log "  ls -lh ${MAP_ROOT}/octomap_live.bt"
else
  log "  DLIO disabled -> OctoMap is also disabled in this mode"
fi
log "Save PCD:"
log "  ros2 service call /map_manager/manage_map a2_interfaces/srv/ManageMap \"{command: save, map_id: jt128_test}\""
