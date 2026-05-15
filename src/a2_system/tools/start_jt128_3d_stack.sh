#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/a2_system_ws}"
LIDAR_IFACE="${A2_JT128_INTERFACE:-net1}"
SDK_IFACE="${A2_SDK_INTERFACE:-eth0}"
CONTROL_IFACE="${A2_CONTROL_INTERFACE:-$SDK_IFACE}"
MODE="mapping"
MAP_ID=""
START_WEB=1
ENABLE_MOTION=false
DRY_RUN=true
ENABLE_NAV2_3D=true
NAV2_3D_MAP=""
START_ROBOT_STATE=true
START_SAFETY=true
LOG_DIR="${WORKSPACE}/runtime/logs"
NAV_STATE_FILE="${WORKSPACE}/runtime/jt128_3d_navigation_state.yaml"
WEB_SERVICE="${A2_WEB_SERVICE:-a2-web-console.service}"
WEB_URL="${A2_WEB_URL:-http://127.0.0.1:8080}"
WEB_BACKEND_PYTHON="${WORKSPACE}/web_console/.venv/bin/python"
WEB_BACKEND_CONFIG="${WORKSPACE}/web_console/backend/config.3d.yaml"
WEB_FALLBACK_LOG="${LOG_DIR}/web_console_fallback.log"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") --mode mapping [--lidar-iface net1] [--no-web]
  $(basename "$0") --mode navigation --map-id MAP_ID [--lidar-iface net1] [--sdk-iface eth0] [--enable-motion] [--live-motion]

Starts the 3D-first JT128 stack:
  mapping:
    JT128 Hesai driver -> /jt128/front/points + /jt128/front/imu
    DLIO -> /jt128/dlio/odom + /jt128/dlio/map_points
    map_manager saves pointcloud_map_3d.pcd

  navigation:
    mapping stack above stays live
    loads pointcloud_map_3d.pcd
    pointcloud_map_loader -> /a2/map/pointcloud_3d
    Autoware NDT adapter -> /a2/relocalization/pose + map->odom
    Nav2 3D global/local navigation -> collision_monitor -> /cmd_vel_safe
    optional a2_control_bridge -> Unitree motion

Safety defaults:
  - --enable-motion starts a2_control_bridge.
  - without --enable-motion, navigation remains a dry-run/control-disabled stack.
EOF
}

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

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="$2"
      shift 2
      ;;
    --map-id)
      MAP_ID="$2"
      shift 2
      ;;
    --lidar-iface|--iface)
      LIDAR_IFACE="$2"
      shift 2
      ;;
    --sdk-iface)
      SDK_IFACE="$2"
      CONTROL_IFACE="$2"
      shift 2
      ;;
    --control-iface)
      CONTROL_IFACE="$2"
      shift 2
      ;;
    --no-web)
      START_WEB=0
      shift
      ;;
    --enable-motion)
      ENABLE_MOTION=true
      shift
      ;;
    --live-motion)
      DRY_RUN=false
      shift
      ;;
    --enable-nav2-3d)
      ENABLE_NAV2_3D=true
      shift
      ;;
    --no-nav2-3d)
      ENABLE_NAV2_3D=false
      shift
      ;;
    --nav2-map)
      NAV2_3D_MAP="$2"
      shift 2
      ;;
    --no-robot-state)
      START_ROBOT_STATE=false
      shift
      ;;
    --no-safety)
      START_SAFETY=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

[[ "$MODE" == "mapping" || "$MODE" == "navigation" ]] || die "mode must be mapping or navigation"
if [[ "$MODE" == "navigation" && -z "$MAP_ID" ]]; then
  die "--map-id is required for navigation mode"
fi
if [[ "$DRY_RUN" == "false" && "$ENABLE_MOTION" != "true" ]]; then
  die "--live-motion requires --enable-motion"
fi
if [[ "$MODE" == "navigation" && "$ENABLE_NAV2_3D" == "true" && -z "$NAV2_3D_MAP" ]]; then
  candidate_map="${WORKSPACE}/runtime/maps/${MAP_ID}/map.yaml"
  if [[ -f "$candidate_map" ]]; then
    NAV2_3D_MAP="$candidate_map"
  else
    die "Nav2 3D requires a projected map YAML. Missing: ${candidate_map}. Use --nav2-map or --no-nav2-3d."
  fi
fi

mkdir -p "$LOG_DIR"

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    source "${WORKSPACE}/install/setup.bash"
  fi
  set -u
}

stop_navigation_components() {
  local pattern
  for pattern in \
    "jt128_3d_navigation.launch.py" \
    "pointcloud_guard" \
    "pointcloud_map_loader" \
    "ndt_scan_matcher" \
    "ndt_adapter" \
    "pcd_relocalizer_3d" \
    "localization_gate" \
    "goal_bridge" \
    "pose_goal_controller_3d" \
    "ground_segmentation_cpp_node" \
    "traversability_to_obstacle_cloud.py" \
    "collision_monitor" \
    "controller_server" \
    "planner_server" \
    "bt_navigator" \
    "velocity_smoother" \
    "safety_supervisor" \
    "real_readiness_monitor" \
    "a2_sdk_bridge_node" \
    "a2_state_publisher_node" \
    "a2_control_bridge_node"; do
    pkill -TERM -f "$pattern" >/dev/null 2>&1 || true
  done
  sleep 1
  for pattern in \
    "jt128_3d_navigation.launch.py" \
    "pointcloud_guard" \
    "pointcloud_map_loader" \
    "ndt_scan_matcher" \
    "ndt_adapter" \
    "pcd_relocalizer_3d" \
    "goal_bridge" \
    "pose_goal_controller_3d" \
    "ground_segmentation_cpp_node" \
    "traversability_to_obstacle_cloud.py" \
    "collision_monitor" \
    "a2_control_bridge_node"; do
    pkill -KILL -f "$pattern" >/dev/null 2>&1 || true
  done
}

start_web() {
  if [[ "$START_WEB" -eq 0 ]]; then
    return
  fi
  if systemctl list-unit-files --type=service 2>/dev/null | awk '{print $1}' | grep -Fxq "$WEB_SERVICE"; then
    if sudo systemctl restart "$WEB_SERVICE"; then
      log "Web console restarted via ${WEB_SERVICE}; url=${WEB_URL}"
      return
    fi
    warn "failed to restart ${WEB_SERVICE}; falling back to direct backend start"
  fi
  if [[ -x "$WEB_BACKEND_PYTHON" && -f "$WEB_BACKEND_CONFIG" ]]; then
    pkill -TERM -f "${WORKSPACE}/web_console/.venv/bin/python -m backend.main" >/dev/null 2>&1 || true
    sleep 1
    nohup bash -lc "
      cd '${WORKSPACE}/web_console'
      exec '${WEB_BACKEND_PYTHON}' -m backend.main --config '${WEB_BACKEND_CONFIG}'
    " >"$WEB_FALLBACK_LOG" 2>&1 &
    log "Web console backend started directly pid=$! url=${WEB_URL} log=${WEB_FALLBACK_LOG}"
  else
    warn "Web console is unavailable: missing ${WEB_BACKEND_PYTHON} or ${WEB_BACKEND_CONFIG}"
  fi
}

source_ros
command -v ros2 >/dev/null 2>&1 || die "ros2 not found after sourcing workspace"

log "Starting JT128 DLIO mapping base stack"
DLIO_MAPPING_SCRIPT="${WORKSPACE}/install/a2_system/share/a2_system/start_jt128_dlio_mapping.sh"
if [[ ! -x "$DLIO_MAPPING_SCRIPT" ]]; then
  DLIO_MAPPING_SCRIPT="${WORKSPACE}/src/a2_system/tools/start_jt128_dlio_mapping.sh"
fi
[[ -x "$DLIO_MAPPING_SCRIPT" ]] || die "DLIO mapping script not found: ${DLIO_MAPPING_SCRIPT}"
"$DLIO_MAPPING_SCRIPT" \
  --iface "$LIDAR_IFACE" \
  --no-web

start_web

if [[ "$MODE" == "mapping" ]]; then
  log "Mapping mode ready"
  log "Save map:"
  log "  ros2 service call /map_manager/manage_map a2_interfaces/srv/ManageMap \"{command: save, map_id: jt128_map}\""
  exit 0
fi

stop_navigation_components
NAV_LOG="${LOG_DIR}/jt128_3d_navigation_$(date +%Y%m%d_%H%M%S).log"
log "Starting JT128 3D navigation components map_id=${MAP_ID} dry_run=${DRY_RUN} enable_motion=${ENABLE_MOTION} enable_nav2_3d=${ENABLE_NAV2_3D}"
setsid bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  source '${WORKSPACE}/install/setup.bash'
  ros2 launch a2_bringup jt128_3d_navigation.launch.py \
    map_id:='${MAP_ID}' \
    start_static_tf:=true \
    start_robot_state:=${START_ROBOT_STATE} \
    start_safety:=${START_SAFETY} \
    enable_nav2_3d:=${ENABLE_NAV2_3D} \
    nav2_3d_map:='${NAV2_3D_MAP}' \
    enable_motion:=${ENABLE_MOTION} \
    dry_run:=${DRY_RUN} \
    sdk_interface:='${SDK_IFACE}' \
    control_interface:='${CONTROL_IFACE}'
" </dev/null >"$NAV_LOG" 2>&1 &
NAV_PID=$!

cat > "$NAV_STATE_FILE" <<EOF
mode: jt128_3d_navigation
pid: ${NAV_PID}
log_file: ${NAV_LOG}
map_id: ${MAP_ID}
lidar_interface: ${LIDAR_IFACE}
sdk_interface: ${SDK_IFACE}
control_interface: ${CONTROL_IFACE}
enable_motion: ${ENABLE_MOTION}
dry_run: ${DRY_RUN}
enable_nav2_3d: ${ENABLE_NAV2_3D}
nav2_3d_map: ${NAV2_3D_MAP}
started_at: $(date --iso-8601=seconds)
EOF

sleep 3
if ! kill -0 "$NAV_PID" >/dev/null 2>&1; then
  tail -120 "$NAV_LOG" >&2 || true
  die "JT128 3D navigation launch exited early; see ${NAV_LOG}"
fi

log "Navigation components started pid=${NAV_PID}"
log "Navigation log: ${NAV_LOG}"
log "Set initial pose before sending goals:"
log "  now=\$(date +%s%N); sec=\${now%?????????}; nsec=\${now: -9}"
log "  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \"{header: {stamp: {sec: \${sec}, nanosec: \${nsec}}, frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}\""
log "Send a short goal through Web or:"
log "  ros2 topic pub --once /a2/exploration/goal geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'"
