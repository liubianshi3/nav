#!/usr/bin/env bash
set -euo pipefail

SOCKET_PATH="${A2_UNITREE_AGENT_SOCKET:-/run/a2/unitree_agent.sock}"
ROS_BRIDGE_CONTAINER="${A2_ROS_BRIDGE_CONTAINER:-a2-nav}"
UNITREE_AGENT_CONTAINER="${A2_UNITREE_AGENT_CONTAINER:-a2-unitree-agent}"
DESTRUCTIVE="${A2_VERIFY_DESTRUCTIVE:-0}"
if [[ "${1:-}" == "--destructive" ]]; then
  DESTRUCTIVE=1
fi

failures=0
warnings=0

pass() {
  echo "[PASS] $*"
}

warn() {
  warnings=$((warnings + 1))
  echo "[WARN] $*" >&2
}

fail() {
  failures=$((failures + 1))
  echo "[FAIL] $*" >&2
}

find_pids() {
  local pattern="$1"
  pgrep -f "$pattern" 2>/dev/null | while read -r pid; do
    [[ -n "$pid" && "$pid" != "$$" && "$pid" != "$BASHPID" ]] && echo "$pid"
  done
}

read_proc_env() {
  local pid="$1"
  tr '\0' '\n' <"/proc/${pid}/environ"
}

read_proc_maps() {
  local pid="$1"
  cat "/proc/${pid}/maps"
}

docker_container_running() {
  local name="$1"
  command -v docker >/dev/null 2>&1 || return 1
  [[ "$(docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null || true)" == "true" ]]
}

docker_bash() {
  local container="$1"
  local command="$2"
  docker exec "$container" bash -lc "$command"
}

find_container_pids() {
  local container="$1"
  local pattern="$2"
  docker_bash "$container" "pgrep -f '$pattern' 2>/dev/null || true"
}

read_container_env() {
  local container="$1"
  local pid="$2"
  docker_bash "$container" "tr '\\0' '\\n' </proc/${pid}/environ"
}

read_container_maps() {
  local container="$1"
  local pid="$2"
  docker_bash "$container" "cat /proc/${pid}/maps"
}

require_command() {
  local name="$1"
  if ! command -v "$name" >/dev/null 2>&1; then
    fail "required command not found: ${name}"
    return 1
  fi
  return 0
}

ros2_in_available_context() {
  local command="$1"
  if command -v ros2 >/dev/null 2>&1; then
    bash -lc "$command"
    return $?
  fi
  if docker_container_running "$ROS_BRIDGE_CONTAINER"; then
    docker_bash "$ROS_BRIDGE_CONTAINER" "source /opt/ros/humble/setup.bash && source /opt/a2_system_ws/install/setup.bash && $command"
    return $?
  fi
  fail "ros2 is not available on host and ${ROS_BRIDGE_CONTAINER} is not running"
  return 1
}

check_ros_graph() {
  local nodes
  if ! nodes="$(ros2_in_available_context "ros2 node list" 2>&1)"; then
    fail "ros2 node list failed: ${nodes}"
    return 0
  fi

  if grep -q "unitree_agent" <<<"$nodes"; then
    fail "unitree_agent is visible in ros2 node list"
  else
    pass "unitree_agent is not visible in ros2 node list"
  fi

  if grep -q "a2_control_bridge" <<<"$nodes"; then
    pass "a2_control_bridge is visible in ROS graph"
  else
    warn "a2_control_bridge was not found in ros2 node list"
  fi

  if grep -q "a2_sdk_bridge" <<<"$nodes"; then
    pass "a2_sdk_bridge is visible in ROS graph"
  else
    warn "a2_sdk_bridge was not found in ros2 node list"
  fi
}

check_bridge_env_text() {
  local name="$1"
  local pid="$2"
  local env_text="$3"
  if grep -qx "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" <<<"$env_text"; then
    pass "${name} pid=${pid} uses rmw_cyclonedds_cpp"
  else
    fail "${name} pid=${pid} does not use RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
  fi
  if grep -qx "ROS_DOMAIN_ID=0" <<<"$env_text"; then
    pass "${name} pid=${pid} uses ROS_DOMAIN_ID=0"
  else
    fail "${name} pid=${pid} does not use ROS_DOMAIN_ID=0"
  fi
  if grep -q "^LD_PRELOAD=.*libddsc.so.0" <<<"$env_text"; then
    fail "${name} pid=${pid} has LD_PRELOAD=libddsc.so.0"
  else
    pass "${name} pid=${pid} has no libddsc LD_PRELOAD"
  fi
}

check_bridge_maps_text() {
  local name="$1"
  local pid="$2"
  local maps_text="$3"
  if grep -Eq "/opt/unitree_robotics/.*/libddsc(|xx)?\\.so(\\.|$)" <<<"$maps_text"; then
    fail "${name} pid=${pid} loaded Unitree DDS from /opt/unitree_robotics"
  else
    pass "${name} pid=${pid} has not loaded Unitree DDS"
  fi
}

check_bridge_env_and_maps() {
  local name="$1"
  local pattern="$2"
  if docker_container_running "$ROS_BRIDGE_CONTAINER"; then
    check_bridge_env_and_maps_in_container "$name" "$pattern" "$ROS_BRIDGE_CONTAINER"
    return 0
  fi

  local pids
  pids="$(find_pids "$pattern" || true)"
  if [[ -z "$pids" ]]; then
    fail "${name} process not found"
    return 0
  fi

  local pid
  while read -r pid; do
    [[ -n "$pid" ]] || continue
    local env_text
    if ! env_text="$(read_proc_env "$pid" 2>/dev/null)"; then
      fail "cannot read /proc/${pid}/environ for ${name}; run as root on the robot host"
      continue
    fi
    check_bridge_env_text "$name" "$pid" "$env_text"

    local maps_text
    if ! maps_text="$(read_proc_maps "$pid" 2>/dev/null)"; then
      fail "cannot read /proc/${pid}/maps for ${name}; run as root on the robot host"
      continue
    fi
    check_bridge_maps_text "$name" "$pid" "$maps_text"
  done <<<"$pids"
}

check_bridge_env_and_maps_in_container() {
  local name="$1"
  local pattern="$2"
  local container="$3"
  local pids
  pids="$(find_container_pids "$container" "$pattern" || true)"
  if [[ -z "$pids" ]]; then
    fail "${name} process not found in container ${container}"
    return 0
  fi

  local pid
  while read -r pid; do
    [[ -n "$pid" ]] || continue
    local env_text
    if ! env_text="$(read_container_env "$container" "$pid" 2>/dev/null)"; then
      fail "cannot read /proc/${pid}/environ for ${name} in container ${container}"
      continue
    fi
    check_bridge_env_text "${name}@${container}" "$pid" "$env_text"

    local maps_text
    if ! maps_text="$(read_container_maps "$container" "$pid" 2>/dev/null)"; then
      fail "cannot read /proc/${pid}/maps for ${name} in container ${container}"
      continue
    fi
    check_bridge_maps_text "${name}@${container}" "$pid" "$maps_text"
  done <<<"$pids"
}

check_unitree_agent() {
  if docker_container_running "$UNITREE_AGENT_CONTAINER"; then
    check_unitree_agent_in_container "$UNITREE_AGENT_CONTAINER"
    return 0
  fi

  local pids
  pids="$(find_pids '(^|/)unitree_agent( |$)' || true)"
  if [[ -z "$pids" ]]; then
    fail "unitree_agent process not found"
    return 0
  fi

  local pid
  while read -r pid; do
    [[ -n "$pid" ]] || continue
    local env_text
    if env_text="$(read_proc_env "$pid" 2>/dev/null)"; then
      if grep -Eq "^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI)=" <<<"$env_text"; then
        fail "unitree_agent pid=${pid} has ROS environment variables"
      else
        pass "unitree_agent pid=${pid} has no ROS graph environment"
      fi
    else
      warn "cannot read /proc/${pid}/environ for unitree_agent"
    fi

    local maps_text
    if ! maps_text="$(read_proc_maps "$pid" 2>/dev/null)"; then
      fail "cannot read /proc/${pid}/maps for unitree_agent; run as root on the robot host"
      continue
    fi
    if grep -Eq "/opt/unitree_robotics/.*/libddsc(|xx)?\\.so(\\.|$)" <<<"$maps_text"; then
      pass "unitree_agent pid=${pid} loaded Unitree DDS from /opt/unitree_robotics"
    else
      fail "unitree_agent pid=${pid} has not loaded Unitree DDS from /opt/unitree_robotics"
    fi
  done <<<"$pids"
}

check_unitree_agent_in_container() {
  local container="$1"
  local pids
  pids="$(find_container_pids "$container" '(^|/)unitree_agent( |$)' || true)"
  if [[ -z "$pids" ]]; then
    fail "unitree_agent process not found in container ${container}"
    return 0
  fi

  local pid
  while read -r pid; do
    [[ -n "$pid" ]] || continue
    local env_text
    if env_text="$(read_container_env "$container" "$pid" 2>/dev/null)"; then
      if grep -Eq "^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI)=" <<<"$env_text"; then
        fail "unitree_agent@${container} pid=${pid} has ROS environment variables"
      else
        pass "unitree_agent@${container} pid=${pid} has no ROS graph environment"
      fi
    else
      fail "cannot read /proc/${pid}/environ for unitree_agent in container ${container}"
    fi

    local maps_text
    if ! maps_text="$(read_container_maps "$container" "$pid" 2>/dev/null)"; then
      fail "cannot read /proc/${pid}/maps for unitree_agent in container ${container}"
      continue
    fi
    if grep -Eq "/opt/unitree_robotics/.*/libddsc(|xx)?\\.so(\\.|$)" <<<"$maps_text"; then
      pass "unitree_agent@${container} pid=${pid} loaded Unitree DDS from /opt/unitree_robotics"
    else
      fail "unitree_agent@${container} pid=${pid} has not loaded Unitree DDS from /opt/unitree_robotics"
    fi
  done <<<"$pids"
}

check_socket() {
  if [[ -S "$SOCKET_PATH" ]]; then
    pass "${SOCKET_PATH} exists and is a Unix Domain Socket"
  elif [[ -e "$SOCKET_PATH" ]]; then
    fail "${SOCKET_PATH} exists but is not a socket"
  else
    fail "${SOCKET_PATH} does not exist"
  fi
}

check_fastrtps_pollution() {
  pgrep -af "rmw_fastrtps_cpp" 2>/dev/null \
    | grep -v "verify_a2_dds_isolation.sh" \
    | grep -v "pgrep -af" >/tmp/a2_verify_fastrtps_processes.$$ || true
  if [[ -s /tmp/a2_verify_fastrtps_processes.$$ ]]; then
    fail "rmw_fastrtps_cpp appears in running processes: $(cat /tmp/a2_verify_fastrtps_processes.$$)"
  else
    pass "no rmw_fastrtps_cpp process arguments found"
  fi
  rm -f /tmp/a2_verify_fastrtps_processes.$$

  local config_hits
  config_hits="$(
    grep -R "rmw_fastrtps_cpp" \
      docker-compose*.yml docker/entrypoint.sh src/a2_bringup/launch src/a2_system/tools web_console/backend/stack_control.py \
      2>/dev/null || true
  )"
  if [[ -n "$config_hits" ]]; then
    fail "rmw_fastrtps_cpp still appears in runtime configuration: ${config_hits}"
  else
    pass "runtime configuration has no rmw_fastrtps_cpp"
  fi
}

check_destructive_failover() {
  if [[ "$DESTRUCTIVE" != "1" && "$DESTRUCTIVE" != "true" ]]; then
    warn "skipping destructive failover checks; run with --destructive on the robot host to kill unitree_agent and test timeout stop"
    return 0
  fi
  local agent_pid
  if docker_container_running "$UNITREE_AGENT_CONTAINER"; then
    docker kill --signal=TERM "$UNITREE_AGENT_CONTAINER" >/dev/null || {
      fail "failed to kill ${UNITREE_AGENT_CONTAINER}"
      return 0
    }
  else
    agent_pid="$(find_pids '(^|/)unitree_agent( |$)' | head -n 1 || true)"
    if [[ -z "$agent_pid" ]]; then
      fail "cannot run destructive kill test because unitree_agent is not running"
      return 0
    fi
    kill -TERM "$agent_pid" || {
      fail "failed to kill unitree_agent pid=${agent_pid}"
      return 0
    }
  fi
  sleep 2

  local status
  status="$(ros2_in_available_context "timeout 5 ros2 topic echo --once /a2/control/status std_msgs/msg/String" 2>/dev/null || true)"
  if grep -Eq "ipc_unavailable|safe|stop|waiting_agent" <<<"$status"; then
    pass "bridge reported safety state after unitree_agent kill"
  else
    fail "bridge did not report expected safety state after unitree_agent kill; status=${status}"
  fi

  ros2_in_available_context "ros2 topic pub --once /cmd_vel_safe geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'" >/dev/null 2>&1 || true
  sleep 1
  status="$(ros2_in_available_context "timeout 5 ros2 topic echo --once /a2/control/status std_msgs/msg/String" 2>/dev/null || true)"
  if grep -Eq "cmd_timeout|stop|ipc_unavailable|safe" <<<"$status"; then
    pass "cmd_vel_safe timeout/stop path reported a safe state"
  else
    fail "cmd_vel_safe timeout/stop path did not report expected state; status=${status}"
  fi
}

check_ros_graph
check_bridge_env_and_maps "a2_control_bridge_ros" '(^|/)a2_control_bridge_node( |$)'
check_bridge_env_and_maps "a2_sdk_bridge_ros" '(^|/)a2_sdk_bridge_node( |$)'
check_unitree_agent
check_socket
check_fastrtps_pollution
check_destructive_failover

if ((failures > 0)); then
  echo "[SUMMARY] failed=${failures} warnings=${warnings}" >&2
  exit 1
fi

echo "[SUMMARY] ok warnings=${warnings}"
