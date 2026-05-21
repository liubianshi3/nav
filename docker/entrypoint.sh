#!/usr/bin/env bash
set -euo pipefail

export A2_WORKSPACE="${A2_WORKSPACE:-/opt/a2_system_ws}"
export CONFIG_PATH="${CONFIG_PATH:-${A2_WORKSPACE}/web_console/backend/config.docker.yaml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

mkdir -p "${A2_WORKSPACE}/runtime/maps" "${A2_WORKSPACE}/runtime/logs"

set +u
source /opt/ros/humble/setup.bash
source "${A2_WORKSPACE}/install/setup.bash"
set -u

log() {
  printf '[a2-docker] %s\n' "$*"
}

configure_cyclonedds_interface() {
  if [[ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ]]; then
    return 0
  fi

  local iface="${A2_ROS_INTERFACE:-}"
  if [[ -z "$iface" ]]; then
    if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
      log "CycloneDDS URI provided by environment"
    fi
    return 0
  fi
  if ! ip link show "$iface" >/dev/null 2>&1; then
    log "CycloneDDS ROS interface skipped; interface not found: ${iface}"
    return 0
  fi

  local peers_xml=""
  local peer
  local peers="${A2_ROS_PEERS:-}"
  if [[ -n "$peers" ]]; then
    peers_xml="<Discovery><Peers>"
    peers="${peers//,/ }"
    for peer in $peers; do
      peers_xml="${peers_xml}<Peer Address=\"${peer}\" />"
    done
    peers_xml="${peers_xml}</Peers></Discovery>"
  fi

  export CYCLONEDDS_URI="<CycloneDDS xmlns=\"https://cdds.io/config\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd\">
  <Domain Id=\"any\">
    <General>
      <Interfaces>
        <NetworkInterface name=\"${iface}\" priority=\"default\" multicast=\"default\" />
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    ${peers_xml}
  </Domain>
</CycloneDDS>"
  log "CycloneDDS ROS traffic bound to iface=${iface} peers=${peers:-<none>}"
}

export_child_ros_env() {
  printf 'export RMW_IMPLEMENTATION=%q\n' "${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    printf 'export CYCLONEDDS_URI=%q\n' "${CYCLONEDDS_URI}"
  fi
}

is_true() {
  [[ "${1:-}" == "true" || "${1:-}" == "1" || "${1:-}" == "yes" || "${1:-}" == "on" ]]
}

wait_for_unitree_agent_socket() {
  if ! is_true "${A2_UNITREE_AGENT_EXTERNAL:-false}"; then
    return 0
  fi
  local socket_path="${A2_UNITREE_AGENT_SOCKET:-/run/a2/unitree_agent.sock}"
  local attempt
  for attempt in $(seq 1 30); do
    if [[ -S "$socket_path" ]]; then
      return 0
    fi
    sleep 0.2
  done
  log "unitree_agent socket not ready yet: ${socket_path}; ROS bridges will retry over UDS"
}

start_standby_control_bridge() {
  local enabled="${A2_CONTROL_BRIDGE_AUTOSTART:-false}"
  if ! is_true "$enabled"; then
    return 0
  fi

  local mode="${A2_DOCKER_START_MODE:-auto}"
  if [[ "$mode" != "web" && "$mode" != "standby" && "$mode" != "none" ]]; then
    log "control bridge autostart skipped mode=${mode}; stack startup owns it"
    return 0
  fi

  local params_file="${A2_WORKSPACE}/install/a2_system/share/a2_system/config/motion_limits.yaml"
  if [[ ! -f "$params_file" ]]; then
    params_file="${A2_WORKSPACE}/src/a2_system/config/motion_limits.yaml"
  fi
  if [[ ! -f "$params_file" ]]; then
    log "control bridge autostart skipped; params file not found"
    return 0
  fi

  local control_iface="${A2_CONTROL_INTERFACE:-${A2_SDK_INTERFACE:-${A2_NETWORK_INTERFACE:-eth0}}}"
  local cmd_topic="${A2_CONTROL_CMD_TOPIC:-/cmd_vel_safe}"
  local allow_without_map="${A2_CONTROL_ALLOW_WITHOUT_MAP:-false}"
  local allow_without_localization="${A2_CONTROL_ALLOW_WITHOUT_LOCALIZATION:-false}"
  local max_linear_x="${A2_CONTROL_MAX_LINEAR_X:-0.20}"
  local max_linear_y="${A2_CONTROL_MAX_LINEAR_Y:-0.10}"
  local max_yaw_rate="${A2_CONTROL_MAX_YAW_RATE:-0.30}"
  local cmd_timeout_sec="${A2_CONTROL_CMD_TIMEOUT_SEC:-0.30}"
  local socket_path="${A2_UNITREE_AGENT_SOCKET:-/run/a2/unitree_agent.sock}"
  local log_file="${A2_WORKSPACE}/runtime/logs/a2_control_bridge_standby.log"
  local env_args=()

  env_args+=("ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}")
  env_args+=("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")

  log "autostarting standby control bridge iface=${control_iface} topic=${cmd_topic}"
  nohup env "${env_args[@]}" ros2 run a2_control_bridge a2_control_bridge_node \
    --ros-args \
    --params-file "$params_file" \
    -p use_mock:=false \
    -p runtime_mode:=real \
    -p allow_loopback:=false \
    -p network_interface:="$control_iface" \
    -p cmd_topic:="$cmd_topic" \
    -p allow_motion_without_map:="$allow_without_map" \
    -p allow_motion_without_localization:="$allow_without_localization" \
    -p max_linear_x:="$max_linear_x" \
	    -p max_linear_y:="$max_linear_y" \
	    -p max_yaw_rate:="$max_yaw_rate" \
	    -p cmd_timeout_sec:="$cmd_timeout_sec" \
	    -p ipc_socket_path:="$socket_path" \
	    > "$log_file" 2>&1 &
  log "standby control bridge log=${log_file}"
}

start_standby_sdk_bridge() {
  local enabled="${A2_SDK_BRIDGE_AUTOSTART:-${A2_CONTROL_BRIDGE_AUTOSTART:-false}}"
  if ! is_true "$enabled"; then
    return 0
  fi

  local mode="${A2_DOCKER_START_MODE:-auto}"
  if [[ "$mode" != "web" && "$mode" != "standby" && "$mode" != "none" ]]; then
    log "sdk bridge autostart skipped mode=${mode}; stack startup owns it"
    return 0
  fi

  local state_topic="${A2_SDK_STATE_TOPIC:-/a2/raw_state}"
  local timer_hz="${A2_SDK_TIMER_HZ:-50.0}"
  local stale_timeout_sec="${A2_SDK_STALE_TIMEOUT_SEC:-0.5}"
  local socket_path="${A2_UNITREE_AGENT_SOCKET:-/run/a2/unitree_agent.sock}"
  local log_file="${A2_WORKSPACE}/runtime/logs/a2_sdk_bridge_standby.log"
  local env_args=()

  env_args+=("ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}")
  env_args+=("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")

  log "autostarting standby sdk bridge state_topic=${state_topic} socket=${socket_path}"
  nohup env "${env_args[@]}" ros2 run a2_sdk_bridge a2_sdk_bridge_node \
    --ros-args \
    -p use_mock:=false \
    -p state_topic:="$state_topic" \
	    -p ipc_socket_path:="$socket_path" \
	    -p timer_hz:="$timer_hz" \
    -p stale_timeout_sec:="$stale_timeout_sec" \
    > "$log_file" 2>&1 &
  log "standby sdk bridge log=${log_file}"
}

start_standby_lidar_preview() {
  local enabled="${A2_STANDBY_LIDAR_AUTOSTART:-true}"
  if ! is_true "$enabled"; then
    return 0
  fi

  local mode="${A2_DOCKER_START_MODE:-auto}"
  if [[ "$mode" != "standby" ]]; then
    log "standby lidar preview skipped mode=${mode}"
    return 0
  fi

  local lidar_iface="${A2_JT128_INTERFACE:-${A2_NETWORK_INTERFACE:-net1}}"
  local lidar_ip="${A2_JT128_IP:-192.168.124.20}"
  local log_file="${A2_WORKSPACE}/runtime/logs/jt128_lidar_standby.log"
  local iface_ip=""

  if ! ip link show "$lidar_iface" >/dev/null 2>&1; then
    log "standby lidar preview skipped; interface not found: ${lidar_iface}"
    return 0
  fi
  iface_ip="$(ip -4 -o addr show dev "$lidar_iface" scope global | awk '{print $4}' | cut -d/ -f1 | head -1)"
  if [[ -z "$iface_ip" ]]; then
    log "standby lidar preview skipped; ${lidar_iface} has no IPv4 address"
    return 0
  fi
  ip route replace "${lidar_ip}/32" dev "$lidar_iface" src "$iface_ip" >/dev/null 2>&1 || true
  if ! ping -I "$lidar_iface" -c 1 -W 1 "$lidar_ip" >/dev/null 2>&1; then
    log "standby lidar preview skipped; JT128 ${lidar_ip} is not reachable on ${lidar_iface}"
    return 0
  fi
  if ss -H -lun | grep -Eq '(^|[[:space:]])[^[:space:]]*:2368[[:space:]]'; then
    log "standby lidar preview skipped; UDP port 2368 is already bound"
    return 0
  fi

  log "autostarting standby JT128 lidar driver iface=${lidar_iface} ip=${lidar_ip}"
  nohup bash -lc "
    set -e
    source /opt/ros/humble/setup.bash
    source '${A2_WORKSPACE}/install/setup.bash'
    export A2_WORKSPACE='${A2_WORKSPACE}'
    $(export_child_ros_env)
    ros2 launch a2_bringup jt128_driver.launch.py use_sim_time:=false
  " >"$log_file" 2>&1 &
  log "standby JT128 lidar log=${log_file}"

  start_standby_pointcloud_preview
}

start_standby_pointcloud_preview() {
  local enabled="${A2_STANDBY_POINTCLOUD_PREVIEW_AUTOSTART:-true}"
  if ! is_true "$enabled"; then
    return 0
  fi
  if [[ ! -x "${A2_WORKSPACE}/install/a2_system/lib/a2_system/pointcloud_preview_node.py" ]]; then
    log "standby pointcloud preview skipped; pointcloud_preview_node.py is not installed"
    return 0
  fi

  local log_file="${A2_WORKSPACE}/runtime/logs/jt128_front_points_preview_standby.log"
  log "autostarting standby pointcloud preview /jt128/front/points_preview"
  nohup bash -lc "
    set -e
    source /opt/ros/humble/setup.bash
    source '${A2_WORKSPACE}/install/setup.bash'
    export A2_WORKSPACE='${A2_WORKSPACE}'
    $(export_child_ros_env)
    ros2 run a2_system pointcloud_preview_node.py --ros-args \
      -p input_topic:=/jt128/front/points \
      -p output_topic:=/jt128/front/points_preview \
      -p preview_rate_hz:=5.0 \
      -p voxel_size_m:=0.05 \
      -p min_range_m:=0.2 \
      -p max_range_m:=20.0 \
      -p max_points:=30000 \
      -p include_intensity:=true \
      -p qos_reliability:=best_effort
  " >"$log_file" 2>&1 &
  log "standby pointcloud preview log=${log_file}"
}

find_latest_3d_map() {
  python3 - "${A2_WORKSPACE}" "${A2_REQUIRE_NAV2_MAP:-true}" <<'PY'
from pathlib import Path
import sys

workspace = Path(sys.argv[1])
require_nav2_map = sys.argv[2].strip().lower() in {"1", "true", "t", "yes", "y", "on"}
maps_root = workspace / "runtime" / "maps"
candidates = []
for metadata in maps_root.glob("*/metadata.yaml"):
    map_dir = metadata.parent
    if not any((map_dir / name).exists() for name in ("pointcloud_map_3d.pcd", "native_map.pcd")):
        continue
    has_nav2_map = (map_dir / "map.yaml").exists()
    if require_nav2_map and not has_nav2_map:
        continue
    text = metadata.read_text(encoding="utf-8", errors="ignore")
    if "pointcloud_map_3d" not in text and "native_pointcloud_map_3d" not in text:
        continue
    candidates.append((has_nav2_map, metadata.stat().st_mtime, map_dir.name))
if candidates:
    print(sorted(candidates)[-1][2])
PY
}

start_a2_stack() {
  local mode="${A2_DOCKER_START_MODE:-auto}"
  local map_id="${A2_NAV_MAP_ID:-}"
  local stack_script="${A2_WORKSPACE}/install/a2_system/share/a2_system/start_jt128_3d_stack.sh"
  local lidar_iface="${A2_JT128_INTERFACE:-${A2_NETWORK_INTERFACE:-net1}}"
  local sdk_iface="${A2_SDK_INTERFACE:-eth0}"
  local control_iface="${A2_CONTROL_INTERFACE:-${sdk_iface}}"
  local enable_motion="${A2_ENABLE_MOTION:-true}"
  local live_motion="${A2_LIVE_MOTION:-true}"
  local enable_nav2_3d="${A2_ENABLE_NAV2_3D:-true}"
  local require_nav2_map="${A2_REQUIRE_NAV2_MAP:-true}"
  local stack_required="${A2_STACK_REQUIRED:-false}"

  if [[ "$mode" == "web" || "$mode" == "standby" || "$mode" == "none" ]]; then
    log "stack autostart disabled mode=${mode}; Web console only"
    return 0
  fi

  if [[ "$mode" == "auto" ]]; then
    if [[ -z "$map_id" ]]; then
      map_id="$(find_latest_3d_map || true)"
    fi
    if [[ -n "$map_id" ]]; then
      mode="navigation"
      log "auto mode selected navigation map_id=${map_id}"
    else
      mode="mapping"
      log "auto mode selected mapping because no saved 3D map was found"
    fi
  fi

  if [[ "$mode" != "mapping" && "$mode" != "navigation" ]]; then
    log "unknown A2_DOCKER_START_MODE=${mode}; use auto|mapping|navigation|standby"
    [[ "$stack_required" == "true" || "$stack_required" == "1" ]] && return 2
    return 0
  fi
  if [[ "$mode" == "navigation" && -z "$map_id" ]]; then
    log "navigation requested but A2_NAV_MAP_ID is empty"
    [[ "$stack_required" == "true" || "$stack_required" == "1" ]] && return 2
    return 0
  fi
  if [[ "$mode" == "navigation" && ( "$enable_nav2_3d" == "true" || "$enable_nav2_3d" == "1" ) && ( "$require_nav2_map" == "true" || "$require_nav2_map" == "1" ) ]]; then
    if [[ ! -f "${A2_WORKSPACE}/runtime/maps/${map_id}/map.yaml" ]]; then
      log "navigation map_id=${map_id} has no map.yaml required by Nav2 3D"
      [[ "$stack_required" == "true" || "$stack_required" == "1" ]] && return 2
      return 0
    fi
  fi
  if [[ ! -x "$stack_script" ]]; then
    stack_script="${A2_WORKSPACE}/src/a2_system/tools/start_jt128_3d_stack.sh"
  fi
  if [[ ! -x "$stack_script" ]]; then
    log "stack script not found; cannot autostart a2sys stack"
    [[ "$stack_required" == "true" || "$stack_required" == "1" ]] && return 2
    return 0
  fi

  local args=(--mode "$mode" --lidar-iface "$lidar_iface" --sdk-iface "$sdk_iface" --control-iface "$control_iface" --no-web)
  if [[ "$mode" == "navigation" ]]; then
    args+=(--map-id "$map_id")
    if [[ "$enable_nav2_3d" == "true" || "$enable_nav2_3d" == "1" ]]; then
      args+=(--enable-nav2-3d)
    else
      args+=(--no-nav2-3d)
    fi
    if [[ "$enable_motion" == "true" || "$enable_motion" == "1" ]]; then
      args+=(--enable-motion)
    fi
    if [[ "$live_motion" == "true" || "$live_motion" == "1" ]]; then
      args+=(--live-motion)
    fi
  fi

  log "autostarting a2sys stack: ${stack_script} ${args[*]}"
  set +e
  "$stack_script" "${args[@]}"
  local rc=$?
  set -e
  if [[ "$rc" -eq 0 ]]; then
    log "a2sys stack autostart command completed"
    return 0
  fi

  log "a2sys stack autostart failed rc=${rc}; Web console will still start"
  [[ "$stack_required" == "true" || "$stack_required" == "1" ]] && return "$rc"
  return 0
}

configure_cyclonedds_interface
wait_for_unitree_agent_socket
start_standby_sdk_bridge
start_standby_control_bridge
start_standby_lidar_preview
start_a2_stack

# Keep the container alive with the Web console backend in the foreground.
exec "${A2_WORKSPACE}/web_console/scripts/run_backend.sh" "$@"
