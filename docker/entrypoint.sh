#!/usr/bin/env bash
set -euo pipefail

export A2_WORKSPACE="${A2_WORKSPACE:-/opt/a2_system_ws}"
export CONFIG_PATH="${CONFIG_PATH:-${A2_WORKSPACE}/web_console/backend/config.docker.yaml}"
export LD_LIBRARY_PATH="/opt/unitree_robotics/lib:/opt/unitree_robotics/lib/x86_64:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

mkdir -p "${A2_WORKSPACE}/runtime/maps" "${A2_WORKSPACE}/runtime/logs"

set +u
source /opt/ros/humble/setup.bash
source "${A2_WORKSPACE}/install/setup.bash"
set -u

log() {
  printf '[a2-docker] %s\n' "$*"
}

is_true() {
  [[ "${1:-}" == "true" || "${1:-}" == "1" || "${1:-}" == "yes" || "${1:-}" == "on" ]]
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
  local log_file="${A2_WORKSPACE}/runtime/logs/a2_control_bridge_standby.log"
  local ld_preload="${A2_CONTROL_BRIDGE_LD_PRELOAD:-}"
  local env_args=()

  if [[ -n "$ld_preload" ]]; then
    env_args+=("LD_PRELOAD=${ld_preload}")
  fi

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

  local sdk_iface="${A2_SDK_INTERFACE:-${A2_NETWORK_INTERFACE:-eth0}}"
  local state_topic="${A2_SDK_STATE_TOPIC:-/a2/raw_state}"
  local sport_state_topic="${A2_SDK_SPORT_STATE_TOPIC:-rt/lf/sportmodestate}"
  local timer_hz="${A2_SDK_TIMER_HZ:-50.0}"
  local stale_timeout_sec="${A2_SDK_STALE_TIMEOUT_SEC:-0.5}"
  local log_file="${A2_WORKSPACE}/runtime/logs/a2_sdk_bridge_standby.log"
  local ld_preload="${A2_SDK_BRIDGE_LD_PRELOAD:-${A2_CONTROL_BRIDGE_LD_PRELOAD:-}}"
  local env_args=()

  if [[ -n "$ld_preload" ]]; then
    env_args+=("LD_PRELOAD=${ld_preload}")
  fi

  log "autostarting standby sdk bridge iface=${sdk_iface} state_topic=${state_topic}"
  nohup env "${env_args[@]}" ros2 run a2_sdk_bridge a2_sdk_bridge_node \
    --ros-args \
    -p use_mock:=false \
    -p auto_detect_interface:=false \
    -p allow_loopback:=false \
    -p network_interface:="$sdk_iface" \
    -p state_topic:="$state_topic" \
    -p sport_state_topic:="$sport_state_topic" \
    -p timer_hz:="$timer_hz" \
    -p stale_timeout_sec:="$stale_timeout_sec" \
    > "$log_file" 2>&1 &
  log "standby sdk bridge log=${log_file}"
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
  local enable_motion="${A2_ENABLE_MOTION:-false}"
  local live_motion="${A2_LIVE_MOTION:-false}"
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

start_standby_sdk_bridge
start_standby_control_bridge
start_a2_stack

# Keep the container alive with the Web console backend in the foreground.
exec "${A2_WORKSPACE}/web_console/scripts/run_backend.sh" "$@"
