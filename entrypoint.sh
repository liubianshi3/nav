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

find_latest_3d_map() {
  python3 - "${A2_WORKSPACE}" <<'PY'
from pathlib import Path
import sys

workspace = Path(sys.argv[1])
maps_root = workspace / "runtime" / "maps"
candidates = []
for metadata in maps_root.glob("*/metadata.yaml"):
    map_dir = metadata.parent
    if not any((map_dir / name).exists() for name in ("pointcloud_map_3d.pcd", "native_map.pcd")):
        continue
    text = metadata.read_text(encoding="utf-8", errors="ignore")
    if "pointcloud_map_3d" not in text and "native_pointcloud_map_3d" not in text:
        continue
    candidates.append((metadata.stat().st_mtime, map_dir.name))
if candidates:
    print(sorted(candidates)[-1][1])
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
    if [[ "$enable_motion" == "true" || "$enable_motion" == "1" ]]; then
      args+=(--enable-motion)
    fi
    if [[ "$live_motion" == "true" || "$live_motion" == "1" ]]; then
      args+=(--live-motion)
    fi
  fi

  log "autostarting a2sys stack: ${stack_script} ${args[*]}"
  if "$stack_script" "${args[@]}"; then
    log "a2sys stack autostart command completed"
    return 0
  fi

  local rc=$?
  log "a2sys stack autostart failed rc=${rc}; Web console will still start"
  [[ "$stack_required" == "true" || "$stack_required" == "1" ]] && return "$rc"
  return 0
}

start_a2_stack

# Keep the container alive with the Web console backend in the foreground.
exec "${A2_WORKSPACE}/web_console/scripts/run_backend.sh" "$@"
