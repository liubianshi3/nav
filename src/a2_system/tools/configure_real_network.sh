#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${A2_WORKSPACE:-$HOME/a2_system_ws}"
DEFAULT_CONFIG_FILE="${SCRIPT_DIR}/config/network.yaml"
FALLBACK_CONFIG_FILE="${WORKSPACE_ROOT}/src/a2_system/config/network.yaml"
DEFAULT_REAL_LIDAR_FILE="${SCRIPT_DIR}/config/real_lidar.yaml"
FALLBACK_REAL_LIDAR_FILE="${WORKSPACE_ROOT}/src/a2_system/config/real_lidar.yaml"
CONFIG_FILE="${A2_NETWORK_CONFIG:-}"
REAL_LIDAR_FILE="${A2_REAL_LIDAR_CONFIG:-}"
IFACE="${1:-${A2_NETWORK_INTERFACE:-}}"

fail() {
  echo "$1" >&2
  exit 1
}

resolve_config_file() {
  if [[ -n "${CONFIG_FILE}" ]]; then
    echo "${CONFIG_FILE}"
    return 0
  fi
  if [[ -f "${DEFAULT_CONFIG_FILE}" ]]; then
    echo "${DEFAULT_CONFIG_FILE}"
    return 0
  fi
  echo "${FALLBACK_CONFIG_FILE}"
}

resolve_real_lidar_file() {
  if [[ -n "${REAL_LIDAR_FILE}" ]]; then
    echo "${REAL_LIDAR_FILE}"
    return 0
  fi
  if [[ -f "${DEFAULT_REAL_LIDAR_FILE}" ]]; then
    echo "${DEFAULT_REAL_LIDAR_FILE}"
    return 0
  fi
  echo "${FALLBACK_REAL_LIDAR_FILE}"
}

ensure_root() {
  if [[ "$(id -u)" == "0" ]]; then
    return 0
  fi
  if command -v sudo >/dev/null 2>&1; then
    exec sudo A2_NETWORK_CONFIG="${CONFIG_FILE}" A2_REAL_LIDAR_CONFIG="${REAL_LIDAR_FILE}" A2_NETWORK_INTERFACE="${IFACE}" bash "$0" "$@"
  fi
  fail "configure_real_network.sh requires root privileges. Re-run with sudo."
}

ensure_addr() {
  local iface="$1"
  local address="$2"
  if ip -4 -o addr show dev "${iface}" | awk '{print $4}' | grep -Fxq "${address}"; then
    echo "address_present=${address}"
    return 0
  fi
  ip addr add "${address}" dev "${iface}"
  echo "address_added=${address}"
}

[[ -n "${IFACE}" ]] || fail "Usage: configure_real_network.sh <iface>"
CONFIG_FILE="$(resolve_config_file)"
REAL_LIDAR_FILE="$(resolve_real_lidar_file)"
[[ -f "${CONFIG_FILE}" ]] || fail "network config not found: ${CONFIG_FILE}"
[[ -f "${REAL_LIDAR_FILE}" ]] || fail "real lidar config not found: ${REAL_LIDAR_FILE}"
command -v ip >/dev/null 2>&1 || fail "`ip` command not found."
command -v python3 >/dev/null 2>&1 || fail "`python3` command not found."

ensure_root "$@"

readarray -t CONFIG_VALUES < <(python3 - "${CONFIG_FILE}" <<'PY'
from pathlib import Path
import sys

try:
    import yaml
except Exception as exc:
    raise SystemExit(f"failed_to_import_yaml:{exc}")

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
network = data.get("network", {}) or {}

values = [
    str(network.get("a2_host_ip", "") or ""),
    str(network.get("a2_control_prefix_len", 24) or 24),
    str(network.get("mid360_host_ip", "") or ""),
    str(network.get("mid360_prefix_len", 24) or 24),
]
for value in values:
    print(value)
PY
)

readarray -t REAL_LIDAR_VALUES < <(python3 - "${REAL_LIDAR_FILE}" <<'PY'
from pathlib import Path
import sys

try:
    import yaml
except Exception as exc:
    raise SystemExit(f"failed_to_import_yaml:{exc}")

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
params = data.get("real_lidar", {}).get("ros__parameters", {}) or {}
print(str(params.get("profile", "") or ""))
print(str(params.get("driver_mode", "") or ""))
PY
)

A2_HOST_IP="${CONFIG_VALUES[0]:-}"
A2_CONTROL_PREFIX_LEN="${CONFIG_VALUES[1]:-24}"
MID360_HOST_IP="${CONFIG_VALUES[2]:-}"
MID360_PREFIX_LEN="${CONFIG_VALUES[3]:-24}"
REAL_LIDAR_PROFILE="${REAL_LIDAR_VALUES[0]:-}"
REAL_LIDAR_DRIVER_MODE="${REAL_LIDAR_VALUES[1]:-}"

[[ -n "${A2_HOST_IP}" ]] || fail "network.a2_host_ip is empty in ${CONFIG_FILE}"
[[ -n "${MID360_HOST_IP}" ]] || fail "network.mid360_host_ip is empty in ${CONFIG_FILE}"

ip link show dev "${IFACE}" >/dev/null 2>&1 || fail "interface does not exist: ${IFACE}"
ip link set "${IFACE}" up

ensure_addr "${IFACE}" "${A2_HOST_IP}/${A2_CONTROL_PREFIX_LEN}"
if [[ "${REAL_LIDAR_PROFILE}" == "unitree_native_fused" || "${REAL_LIDAR_DRIVER_MODE}" == "external_pointcloud" ]]; then
  echo "sensor_subnet_skipped=real_lidar_native_source profile=${REAL_LIDAR_PROFILE} driver_mode=${REAL_LIDAR_DRIVER_MODE}"
elif [[ "${REAL_LIDAR_PROFILE}" == "hesai_jt128_front" || "${REAL_LIDAR_DRIVER_MODE}" == "dedicated_hesai_ros_driver" ]]; then
  echo "sensor_subnet_skipped=dedicated_hesai_jt128 profile=${REAL_LIDAR_PROFILE} driver_mode=${REAL_LIDAR_DRIVER_MODE}"
else
  ensure_addr "${IFACE}" "${MID360_HOST_IP}/${MID360_PREFIX_LEN}"
fi

echo "configured_interface=${IFACE}"
echo "config_file=${CONFIG_FILE}"
echo "real_lidar_file=${REAL_LIDAR_FILE}"
ip -br addr show dev "${IFACE}"
