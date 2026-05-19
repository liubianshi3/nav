#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/ws/device-navigation}"
GRAPH_PID_WS="${A2_GRAPH_PID_WS:-$HOME/graph_pid_ws}"
IFACE="${A2_NETWORK_INTERFACE:-}"
EXTRA_ARGS=()
export A2_WORKSPACE="${WORKSPACE}"

if [[ $# -gt 0 ]]; then
  IFACE="$1"
  shift
fi

if [[ $# -gt 0 ]]; then
  EXTRA_ARGS=("$@")
fi
if [[ "${A2_ENABLE_NAV2:-false}" == "1" || "${A2_ENABLE_NAV2:-false}" == "true" ]]; then
  # Legacy 2D nav2 path; 3D-first projects should set A2_ENABLE_NAV2=false and use start_jt128_3d_stack.sh instead
  EXTRA_ARGS+=("enable_nav2_bringup:=true")
  EXTRA_ARGS+=("enable_control_bridge:=true")
  EXTRA_ARGS+=("real_localization_mode:=${A2_REAL_LOCALIZATION_MODE:-uslam_odom}")
fi
if [[ -n "${A2_MAP_YAML:-}" ]]; then
  EXTRA_ARGS+=("map:=${A2_MAP_YAML}")
fi
LOG_DIR="${WORKSPACE}/runtime/logs"
mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${GRAPH_PID_WS}/install/setup.bash" ]]; then
  source "${GRAPH_PID_WS}/install/setup.bash"
fi
source "${WORKSPACE}/install/setup.bash"
set -u

# Detect Docker: skip hardware-specific setup that requires host system tools
_IS_DOCKER=false
if [[ -f /.dockerenv ]] || grep -q docker /proc/1/cgroup 2>/dev/null; then
  _IS_DOCKER=true
  echo "Running in Docker — skipping network interface preflight, DDS setup, and sudo calls."
fi

PREFLIGHT_SCRIPT="${WORKSPACE}/install/a2_system/share/a2_system/preflight_check.py"
CONFIGURE_NETWORK_SCRIPT="${WORKSPACE}/install/a2_system/share/a2_system/configure_real_network.sh"
SETUP_DDS_SCRIPT="${WORKSPACE}/install/a2_system/share/a2_system/setup_unitree_dds.sh"
PREFLIGHT_ARGS=(--mode real)

if [[ -n "${IFACE}" ]]; then
  PREFLIGHT_ARGS+=(--interface "${IFACE}")
fi

if ! $_IS_DOCKER; then
  python3 "${PREFLIGHT_SCRIPT}" "${PREFLIGHT_ARGS[@]}" || true

  if [[ -z "${IFACE}" ]]; then
    source "${SETUP_DDS_SCRIPT}"
    IFACE="${A2_NETWORK_INTERFACE:-${IFACE}}"
  fi

  if [[ "${A2_AUTO_CONFIGURE_NETWORK:-1}" == "1" ]]; then
    "${CONFIGURE_NETWORK_SCRIPT}" "${IFACE}"
  fi

  source "${SETUP_DDS_SCRIPT}" "${IFACE}"
  IFACE="${A2_NETWORK_INTERFACE:-${IFACE}}"
fi

# Keep the interface readiness result from setup_unitree_dds.sh, but launch the
# ROS workspace with the default RMW to avoid SDK/domain collisions in the
# Unitree bridge processes.
unset RMW_IMPLEMENTATION || true
unset CYCLONEDDS_URI || true

PID_FILE="${WORKSPACE}/runtime/bringup.pid"
LOG_FILE="${LOG_DIR}/bringup_real_$(date +%Y%m%d_%H%M%S).log"

{
  echo "timestamp=$(date --iso-8601=seconds)"
  echo "workspace=${WORKSPACE}"
  echo "network_interface=${IFACE:-<empty>}"
  echo "rmw_implementation=${RMW_IMPLEMENTATION:-<unset>}"
  echo "cyclonedds_bind_status=${A2_CYCLONEDDS_BIND_STATUS:-unknown}"
  echo "cyclonedds_bind_reason=${A2_CYCLONEDDS_BIND_REASON:-unknown}"
  echo "real_diagnostic_only=${A2_REAL_DIAGNOSTIC_ONLY:-unknown}"
  echo "cyclonedds_uri=${CYCLONEDDS_URI:-<unset>}"
  echo "interfaces:"
  ip -br link || true
  echo "addresses:"
  ip -br addr || true
  echo
} > "${LOG_FILE}"

nohup ros2 launch a2_bringup bringup.launch.py runtime_mode:=real network_interface:="${IFACE}" "${EXTRA_ARGS[@]}" >> "${LOG_FILE}" 2>&1 &
echo $! > "${PID_FILE}"
echo "Started real bringup pid=$(cat "${PID_FILE}")"
echo "Log file: ${LOG_FILE}"
echo "network_interface=${IFACE:-<empty>}"
echo "cyclonedds_bind_status=${A2_CYCLONEDDS_BIND_STATUS:-unknown}"
echo "cyclonedds_bind_reason=${A2_CYCLONEDDS_BIND_REASON:-unknown}"
if [[ "${A2_REAL_DIAGNOSTIC_ONLY:-0}" == "1" ]]; then
  echo "Real stack started in diagnostic mode. A ready wired interface is required before A2 data can go online."
fi
