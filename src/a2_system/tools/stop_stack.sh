#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/a2_system_ws}"
PID_FILE="${WORKSPACE}/runtime/bringup.pid"
PATTERNS=(
  "bringup.launch.py"
  "a2_sdk_bridge_node"
  "a2_control_bridge_node"
  "task_manager.py"
  "pointcloud_relay"
  "pointcloud_to_laserscan"
  "slam_toolbox"
  "native_map_relay"
  "localization_gate"
  "manual_localization_publisher"
  "amcl"
  "goal_bridge"
  "occupancy_mapper"
  "map_manager_node"
  "map_server"
  "controller_server"
  "smoother_server"
  "planner_server"
  "behavior_server"
  "bt_navigator"
  "waypoint_follower"
  "velocity_smoother"
  "lifecycle_manager"
)

collect_descendants() {
  local root_pid="$1"
  local pending=("$root_pid")
  local seen=()
  while [[ ${#pending[@]} -gt 0 ]]; do
    local current="${pending[-1]}"
    unset 'pending[-1]'
    if [[ " ${seen[*]} " == *" ${current} "* ]]; then
      continue
    fi
    seen+=("${current}")
    local children
    children="$(pgrep -P "${current}" || true)"
    if [[ -n "${children}" ]]; then
      while read -r child; do
        [[ -n "${child}" ]] && pending+=("${child}")
      done <<< "${children}"
    fi
  done
  printf '%s\n' "${seen[@]}"
}

signal_pids() {
  local signal_name="$1"
  shift || true
  local pid
  for pid in "$@"; do
    [[ -z "${pid}" ]] && continue
    kill "-${signal_name}" "${pid}" >/dev/null 2>&1 || true
  done
}

cleanup_patterns() {
  local pattern
  for pattern in "${PATTERNS[@]}"; do
    pkill -f "${pattern}" >/dev/null 2>&1 || true
  done
}

if [[ ! -f "${PID_FILE}" ]]; then
  cleanup_patterns
  echo "No PID file found at ${PID_FILE}, cleaned known runtime patterns"
  exit 0
fi

PID="$(cat "${PID_FILE}")"
TARGET_PIDS=()
if kill -0 "${PID}" >/dev/null 2>&1; then
  while read -r target_pid; do
    [[ -n "${target_pid}" ]] && TARGET_PIDS+=("${target_pid}")
  done < <(collect_descendants "${PID}")
  signal_pids TERM "${TARGET_PIDS[@]}"
  sleep 2
  signal_pids KILL "${TARGET_PIDS[@]}"
  wait "${PID}" 2>/dev/null || true
  echo "Stopped bringup pid=${PID} and descendant processes"
else
  echo "PID ${PID} is not running"
fi

cleanup_patterns
rm -f "${PID_FILE}"
