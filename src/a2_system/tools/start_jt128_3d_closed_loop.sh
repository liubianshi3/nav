#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/ws/device-navigation}"
MODE="auto"
REQUESTED_MODE="auto"
MAP_ID=""
LIDAR_IFACE="${A2_JT128_INTERFACE:-net1}"
SDK_IFACE="${A2_SDK_INTERFACE:-eth0}"
CONTROL_IFACE="${A2_CONTROL_INTERFACE:-$SDK_IFACE}"
ENABLE_MOTION=true
LIVE_MOTION=true
LOCALIZATION_MODE=ndt
COLLISION_MONITOR_PROFILE="${A2_COLLISION_MONITOR_PROFILE:-strict}"
ENABLE_GLOBAL_TRAVERSABILITY_LAYER="${A2_ENABLE_GLOBAL_TRAVERSABILITY_LAYER:-true}"
STOP_EXISTING=1
RUN_PREFLIGHT=1
RUN_ID=""
WEB_URL="${A2_WEB_URL:-http://127.0.0.1:8080}"
LOG_DIR="${WORKSPACE}/runtime/logs"
RECORD_DIR="${WORKSPACE}/runtime/test_records"
WEB_LOG="${LOG_DIR}/web_console_manual_$(date +%Y%m%d_%H%M%S).log"
WEB_STATE_FILE="${WORKSPACE}/runtime/web_stack_state.yaml"
WEB_RUN_SCRIPT="${WORKSPACE}/web_console/scripts/run_backend.sh"
STACK_SCRIPT="${WORKSPACE}/src/a2_system/tools/start_jt128_3d_stack.sh"
PREFLIGHT_SCRIPT="${WORKSPACE}/src/a2_system/scripts/industrial_3d_nav_preflight.py"
CORRIDOR_GATE_SCRIPT="${WORKSPACE}/src/a2_system/scripts/nav2_corridor_gate.py"
RECORD_SCRIPT="${WORKSPACE}/src/a2_system/scripts/append_3d_test_record.py"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--mode auto] [--lidar-iface net1]
  $(basename "$0") --mode auto [--lidar-iface net1] [--sdk-iface eth0]
  $(basename "$0") --mode mapping [--lidar-iface net1]
  $(basename "$0") --mode navigation --map-id MAP_ID [--lidar-iface net1] [--sdk-iface eth0] [--localization-mode ndt|odom_only] [--collision-profile strict|live-validation] [--enable-motion] [--live-motion]

Auto behavior:
  Finds the newest 3D pointcloud map under runtime/maps.
  - If found, starts navigation with real Unitree motion.
  - If not found, starts mapping mode.
  - Runs preflight and appends a CSV test record when navigation starts.

Safety:
  Navigation defaults to real /cmd_vel output. Keep the robot supervised.
  live-validation collision profile is only for supervised open-space tests.

Global traversability feedback is enabled by default.
Use --no-global-traversability-layer or A2_ENABLE_GLOBAL_TRAVERSABILITY_LAYER=false to disable it.
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
      REQUESTED_MODE="$2"
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
    --enable-motion)
      ENABLE_MOTION=true
      shift
      ;;
    --localization-mode)
      LOCALIZATION_MODE="$2"
      shift 2
      ;;
    --collision-profile)
      COLLISION_MONITOR_PROFILE="$2"
      shift 2
      ;;
    --live-motion)
      LIVE_MOTION=true
      ENABLE_MOTION=true
      shift
      ;;
    --no-stop-existing)
      STOP_EXISTING=0
      shift
      ;;
    --no-preflight)
      RUN_PREFLIGHT=0
      shift
      ;;
    --run-id)
      RUN_ID="$2"
      shift 2
      ;;
    --enable-global-traversability-layer)
      ENABLE_GLOBAL_TRAVERSABILITY_LAYER=true
      shift
      ;;
    --no-global-traversability-layer)
      ENABLE_GLOBAL_TRAVERSABILITY_LAYER=false
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

[[ "$MODE" == "standby" || "$MODE" == "auto" || "$MODE" == "mapping" || "$MODE" == "navigation" ]] || die "mode must be standby, auto, mapping, or navigation"
[[ "$LOCALIZATION_MODE" == "ndt" || "$LOCALIZATION_MODE" == "odom_only" ]] || die "localization mode must be ndt or odom_only"
[[ "$COLLISION_MONITOR_PROFILE" == "strict" || "$COLLISION_MONITOR_PROFILE" == "live-validation" ]] || die "collision profile must be strict or live-validation"
if [[ "$MODE" == "navigation" && -z "$MAP_ID" ]]; then
  die "--map-id is required for navigation mode"
fi
if [[ "$LIVE_MOTION" == "true" && "$ENABLE_MOTION" != "true" ]]; then
  die "--live-motion requires --enable-motion"
fi

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    source "${WORKSPACE}/install/setup.bash"
  fi
  set -u
}

find_latest_3d_map() {
  python3 - "$WORKSPACE" <<'PY'
from pathlib import Path
import sys

workspace = Path(sys.argv[1])
maps_root = workspace / "runtime" / "maps"
candidates = []
for metadata in maps_root.glob("*/metadata.yaml"):
    map_dir = metadata.parent
    pcd = map_dir / "pointcloud_map_3d.pcd"
    if not pcd.exists() or pcd.stat().st_size <= 0:
        continue
    text = metadata.read_text(encoding="utf-8", errors="ignore")
    if "pointcloud_map_3d" not in text:
        continue
    candidates.append((metadata.stat().st_mtime, map_dir.name))
if candidates:
    print(sorted(candidates)[-1][1])
PY
}

resolve_auto_mode() {
  [[ "$MODE" == "auto" ]] || return
  local latest_map
  latest_map="$(find_latest_3d_map || true)"
  if [[ -n "$latest_map" ]]; then
    MODE="navigation"
    MAP_ID="$latest_map"
    log "Auto mode selected navigation with latest 3D map: ${MAP_ID}"
  else
    MODE="mapping"
    log "Auto mode selected mapping because no 3D pointcloud map was found"
  fi
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

show_urls() {
  ip -4 -o addr show scope global 2>/dev/null | awk '{print $2 " " $4}' | while read -r name cidr; do
    case "$name" in
      lo|docker0|br-*|veth*)
        continue
        ;;
    esac
    printf '[INFO] Open: http://%s:8080/ (%s)\n' "${cidr%%/*}" "$name"
  done
}

write_web_state() {
  local mode="$1"
  local message="$2"
  mkdir -p "$(dirname "$WEB_STATE_FILE")"
  cat > "$WEB_STATE_FILE" <<EOF
mode: ${mode}
target_mode: null
selected_map_id: ${MAP_ID:-null}
selected_map_yaml: null
message: "${message}"
EOF
}

kill_own_pattern() {
  local signal="$1"
  local pattern="$2"
  local pids=()
  local pid
  while IFS= read -r pid; do
    [[ -n "$pid" ]] || continue
    [[ "$pid" == "$$" || "$pid" == "$BASHPID" ]] && continue
    pids+=("$pid")
  done < <(pgrep -u "$(id -u)" -f "$pattern" 2>/dev/null || true)
  ((${#pids[@]} > 0)) || return 0
  kill "-${signal}" "${pids[@]}" >/dev/null 2>&1 || true
}

stop_owned_robot_stack() {
  local pattern
  local stack_patterns=(
    "jt128_3d_navigation.launch.py"
    "dlio_mapping.launch.py"
    "jt128_driver.launch.py"
    "hesai_ros_driver_node"
    "jt128_hesai_driver"
    "dlio_odom_node"
    "dlio_map_node"
    "jt128_dlio_odom"
    "jt128_dlio_map"
    "jt128_dlio_odom_tf_broadcaster"
    "jt128_dlio_watchdog.py"
    "jt128_static_tf_manager"
    "jt128_navigation_static_tf_manager"
    "octomap_mapping_node.py"
    "octomap_server_node"
    "octomap_saver_node"
    "pointcloud_guard"
    "pointcloud_map_loader"
    "pcd_relocalizer_3d"
    "ndt_scan_matcher"
    "autoware_ndt_scan_matcher_node"
    "ndt_adapter"
    "ndt_health_monitor"
    "sensor_covariance_injector.py"
    "body_imu_covariance_injector"
    "ekf_node"
    "localization_gate"
    "goal_bridge"
    "pose_goal_controller_3d"
    "ground_segmentation_cpp_node"
    "traversability_to_obstacle_cloud.py"
    "global_traversability_integrator.py"
    "global_traversability_integrator"
    "collision_monitor"
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
    "map_server"
    "auto_scan_mission.py"
    "task_manager.py"
    "safety_supervisor"
    "real_readiness_monitor"
    "a2_sdk_bridge_node"
    "a2_state_publisher_node"
    "a2_control_bridge_node"
    "map_manager_node"
  )
  for pattern in "${stack_patterns[@]}"; do
    kill_own_pattern TERM "$pattern"
  done
  sleep 1
  for pattern in "${stack_patterns[@]}"; do
    kill_own_pattern KILL "$pattern"
  done
}

start_web_backend() {
  mkdir -p "$LOG_DIR"
  [[ -x "$WEB_RUN_SCRIPT" ]] || die "missing Web run script: $WEB_RUN_SCRIPT"

  if wait_http_ok "${WEB_URL}/api/health" 2; then
    log "Web backend is already healthy: ${WEB_URL}"
    return
  fi

  kill_own_pattern TERM "python.*-m backend.main"
  kill_own_pattern TERM "web_console/scripts/run_backend.sh"
  sleep 1

  log "Starting Web backend directly"
  nohup bash -lc "
    cd '${WORKSPACE}/web_console'
    exec '${WEB_RUN_SCRIPT}'
  " >"$WEB_LOG" 2>&1 &
  log "Web backend pid=$! log=${WEB_LOG}"

  if ! wait_http_ok "${WEB_URL}/api/health" 25; then
    tail -120 "$WEB_LOG" >&2 || true
    die "Web API did not become ready: ${WEB_URL}/api/health"
  fi
}

start_stack_mode() {
  local args=("--mode" "$MODE" "--lidar-iface" "$LIDAR_IFACE" "--no-web")
  [[ -x "$STACK_SCRIPT" ]] || die "missing stack script: $STACK_SCRIPT"

  if [[ "$MODE" == "navigation" ]]; then
    args+=(
      "--map-id" "$MAP_ID"
      "--sdk-iface" "$SDK_IFACE"
      "--control-iface" "$CONTROL_IFACE"
      "--localization-mode" "$LOCALIZATION_MODE"
      "--collision-profile" "$COLLISION_MONITOR_PROFILE"
    )
    if [[ "$ENABLE_MOTION" == "true" ]]; then
      args+=("--enable-motion")
    fi
    if [[ "$LIVE_MOTION" == "true" ]]; then
      args+=("--live-motion")
    fi
  fi

  if [[ "$ENABLE_GLOBAL_TRAVERSABILITY_LAYER" == "true" ]]; then
    args+=("--enable-global-traversability-layer")
  else
    args+=("--no-global-traversability-layer")
  fi

  if [[ "$ENABLE_GLOBAL_TRAVERSABILITY_LAYER" == "true" ]]; then
    args+=("--enable-global-traversability-layer")
  else
    args+=("--no-global-traversability-layer")
  fi

  log "Starting ${MODE} stack through ${STACK_SCRIPT}"
  "$STACK_SCRIPT" "${args[@]}"
  write_web_state "$MODE" "Started ${MODE} through start_jt128_3d_closed_loop.sh"
}

run_navigation_preflight_and_record() {
  [[ "$MODE" == "navigation" && "$RUN_PREFLIGHT" -eq 1 ]] || return
  [[ -f "$PREFLIGHT_SCRIPT" ]] || {
    warn "preflight script not found: $PREFLIGHT_SCRIPT"
    return
  }
  [[ -f "$RECORD_SCRIPT" ]] || {
    warn "record script not found: $RECORD_SCRIPT"
    return
  }

  mkdir -p "$RECORD_DIR" "$LOG_DIR"
  local effective_run_id="${RUN_ID:-auto_$(date +%Y%m%d_%H%M%S)}"
  local preflight_json="${RECORD_DIR}/${effective_run_id}_preflight.json"
  local preflight_log="${LOG_DIR}/${effective_run_id}_preflight.log"
  local corridor_json="${RECORD_DIR}/${effective_run_id}_corridor_gate.json"
  local corridor_log="${LOG_DIR}/${effective_run_id}_corridor_gate.log"
  local result="PASS"
  local notes

  log "Waiting briefly before preflight"
  sleep 8
  log "Running industrial 3D preflight; output=${preflight_json}"
  if ! bash -lc "
    source /opt/ros/humble/setup.bash
    source '${WORKSPACE}/install/setup.bash'
    python3 '${PREFLIGHT_SCRIPT}' --output '${preflight_json}' --timeout-sec 8
  " >"$preflight_log" 2>&1; then
    result="FAIL"
    warn "Preflight failed; see ${preflight_log}"
  fi

  if [[ -f "$CORRIDOR_GATE_SCRIPT" ]]; then
    log "Running 0.5m Nav2 corridor gate; output=${corridor_json}"
    if ! bash -lc "
      source /opt/ros/humble/setup.bash
      source '${WORKSPACE}/install/setup.bash'
      python3 '${CORRIDOR_GATE_SCRIPT}' --distance 0.5 --scan-directions --output-json '${corridor_json}' --timeout-sec 8
    " >"$corridor_log" 2>&1; then
      result="FAIL"
      warn "Corridor gate failed; see ${corridor_log}"
    fi
  else
    warn "corridor gate script not found: $CORRIDOR_GATE_SCRIPT"
  fi

  notes="auto launcher preflight result=${result}; preflight_log=${preflight_log}; preflight_json=${preflight_json}; corridor_log=${corridor_log}; corridor_json=${corridor_json}"
  python3 "$RECORD_SCRIPT" \
    --run-id "$effective_run_id" \
    --run-type real_robot \
    --robot-id "${A2_ROBOT_ID:-a2-jt128}" \
    --site "${A2_TEST_SITE:-unspecified}" \
    --operator "${A2_OPERATOR:-unspecified}" \
    --software-ref "$(git -C "$WORKSPACE" rev-parse --short HEAD 2>/dev/null || echo local-working-tree)" \
    --command "$(basename "$0") --mode ${REQUESTED_MODE} resolved=${MODE} map_id=${MAP_ID:-none}" \
    --map-id "${MAP_ID:-}" \
    --environment "${A2_TEST_ENVIRONMENT:-unspecified}" \
    --result "$result" \
    --collision-count 0 \
    --near-collision-count 0 \
    --estop-count 0 \
    --notes "$notes" \
    --next-action "monitor live-motion run and review preflight/corridor gate artifacts"

  if [[ "$LIVE_MOTION" == "true" && "$result" != "PASS" ]]; then
    stop_owned_robot_stack
    die "Live motion requested but preflight/corridor gate failed; robot stack stopped. Review ${preflight_log} and ${corridor_log}."
  fi
}

cd "$WORKSPACE"
command -v curl >/dev/null 2>&1 || die "missing command: curl"
source_ros
resolve_auto_mode

if [[ "$MODE" == "standby" ]]; then
  if [[ "$STOP_EXISTING" -eq 1 ]]; then
    log "Stopping user-owned JT128/3D stack processes for Web standby"
    stop_owned_robot_stack
  fi
  write_web_state "stopped" "Web console ready; choose mapping or navigation in the UI"
  start_web_backend
else
  start_web_backend
  start_stack_mode
  run_navigation_preflight_and_record
fi

echo
log "Web health"
curl -fsS "${WEB_URL}/api/health" || true
echo
log "Web stack status"
curl -fsS "${WEB_URL}/api/stack/status" || true
echo
show_urls
if [[ "$MODE" == "standby" ]]; then
  log "Standby ready: Web should show stopped. Choose mapping or navigation in the UI."
fi
