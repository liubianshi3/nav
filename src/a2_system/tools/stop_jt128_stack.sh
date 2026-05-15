#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/a2_system_ws}"
STATE_FILE="${WORKSPACE}/runtime/jt128_dlio_stack_state.yaml"

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
  sudo kill "-${signal}" "${pids[@]}" >/dev/null 2>&1 || true
}

if [[ -f "$STATE_FILE" ]]; then
  PID="$(awk -F': ' '/^pid:/ {print $2; exit}' "$STATE_FILE" || true)"
  if [[ "${PID:-}" =~ ^[0-9]+$ ]]; then
    kill "$PID" >/dev/null 2>&1 || true
  fi
fi

for pattern in \
  "dlio_mapping.launch.py" \
  "jt128_driver.launch.py" \
  "jt128_hesai_driver" \
  "hesai_ros_driver_node" \
  "dlio_odom_node" \
  "dlio_map_node" \
  "jt128_dlio_odom" \
  "jt128_dlio_map" \
  "jt128_3d_navigation.launch.py" \
  "pointcloud_guard" \
  "pointcloud_map_loader" \
  "pcd_relocalizer_3d" \
  "localization_gate" \
  "goal_bridge" \
  "pose_goal_controller_3d" \
  "safety_supervisor" \
  "real_readiness_monitor" \
  "a2_sdk_bridge_node" \
  "a2_state_publisher_node" \
  "a2_control_bridge_node" \
  "map_manager_node" \
  "jt128_dlio_watchdog.py" \
  "jt128_static_tf_manager"; do
  kill_pattern TERM "$pattern"
done
sleep 1
for pattern in \
  "dlio_mapping.launch.py" \
  "jt128_driver.launch.py" \
  "hesai_ros_driver_node" \
  "dlio_odom_node" \
  "dlio_map_node" \
  "jt128_3d_navigation.launch.py" \
  "pointcloud_guard" \
  "pointcloud_map_loader" \
  "pcd_relocalizer_3d" \
  "goal_bridge" \
  "pose_goal_controller_3d" \
  "a2_control_bridge_node" \
  "map_manager_node" \
  "jt128_dlio_watchdog.py" \
  "jt128_static_tf_manager"; do
  kill_pattern KILL "$pattern"
done

echo "[INFO] stopped JT128/DLIO stack"
