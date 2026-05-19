#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${A2_WORKSPACE:-$HOME/ws/device-navigation}"
DEFAULT_CONFIG_FILE="${SCRIPT_DIR}/config/network.yaml"
FALLBACK_CONFIG_FILE="${WORKSPACE_ROOT}/src/a2_system/config/network.yaml"
CONFIG_FILE="${A2_NETWORK_CONFIG:-}"
PREFERRED_IFACE="${1:-${A2_NETWORK_INTERFACE:-}}"

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

CONFIG_FILE="$(resolve_config_file)"

fail() {
  echo "$1" >&2
  return 1 2>/dev/null || exit 1
}

is_virtual_like() {
  case "$1" in
    docker*|br-*|veth*|virbr*|vmnet*|wl*|tun*|tap*|tailscale*|Meta*)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

is_wired_like() {
  case "$1" in
    en*|eth*|enx*)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

interface_exists() {
  [[ -n "${1:-}" ]] || return 1
  ip link show dev "$1" >/dev/null 2>&1
}

interface_state_line() {
  ip -o link show dev "$1" 2>/dev/null || true
}

interface_has_ipv4() {
  [[ -n "${1:-}" ]] || return 1
  ip -4 -o addr show dev "$1" scope global 2>/dev/null | grep -q .
}

interface_has_carrier() {
  local iface="$1"
  if [[ -r "/sys/class/net/${iface}/carrier" ]]; then
    [[ "$(cat "/sys/class/net/${iface}/carrier")" == "1" ]]
    return
  fi
  interface_state_line "${iface}" | grep -q 'LOWER_UP'
}

interface_is_ready_for_real() {
  local iface="$1"
  local line
  [[ -n "${iface:-}" ]] || return 1
  interface_exists "${iface}" || return 1
  [[ "${iface}" != "lo" ]] || return 1
  is_virtual_like "${iface}" && return 1
  line="$(interface_state_line "${iface}")"
  [[ "${line}" == *" UP "* || "${line}" == *"<"*UP*">"* ]] || return 1
  interface_has_carrier "${iface}" || return 1
}

describe_interface() {
  local iface="$1"
  if [[ -z "${iface}" ]]; then
    echo "<none>"
    return 0
  fi
  if ! interface_exists "${iface}"; then
    echo "${iface}:missing"
    return 0
  fi

  local link_line
  local addr_line
  link_line="$(ip -br link show dev "${iface}" 2>/dev/null | xargs || true)"
  addr_line="$(ip -br addr show dev "${iface}" 2>/dev/null | xargs || true)"
  echo "${link_line} | ${addr_line}"
}

mapfile_or_empty() {
  local -n out_ref=$1
  shift
  if "$@" >/dev/null 2>&1; then
    mapfile -t out_ref < <("$@")
  else
    out_ref=()
  fi
}

load_configured_interfaces() {
  [[ -f "${CONFIG_FILE}" ]] || return 0
  command -v python3 >/dev/null 2>&1 || return 0
  python3 - "${CONFIG_FILE}" <<'PY'
from pathlib import Path
import sys

try:
    import yaml
except Exception:
    sys.exit(0)

path = Path(sys.argv[1])
if not path.exists():
    sys.exit(0)

data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
network = data.get("network", {}) or {}
iface = network.get("network_interface", "") or ""
if iface:
    print(iface)
for candidate in network.get("interface_candidates", []) or []:
    if candidate:
        print(candidate)
PY
}

discover_interfaces() {
  command -v ip >/dev/null 2>&1 || return 0
  ip -o link show | awk -F': ' '{print $2}' | cut -d'@' -f1
}

pick_best_ready_interface() {
  local iface
  while IFS= read -r iface; do
    [[ -n "${iface}" ]] || continue
    is_wired_like "${iface}" || continue
    is_virtual_like "${iface}" && continue
    [[ "${iface}" == "lo" ]] && continue
    if interface_is_ready_for_real "${iface}"; then
      echo "${iface}"
      return 0
    fi
  done < <(discover_interfaces)
  return 1
}

pick_best_existing_interface() {
  local iface
  while IFS= read -r iface; do
    [[ -n "${iface}" ]] || continue
    is_wired_like "${iface}" || continue
    is_virtual_like "${iface}" && continue
    [[ "${iface}" == "lo" ]] && continue
    if interface_exists "${iface}"; then
      echo "${iface}"
      return 0
    fi
  done < <(discover_interfaces)
  return 1
}

pick_interface() {
  local configured=()
  local candidate

  if [[ -n "${PREFERRED_IFACE}" ]]; then
    echo "${PREFERRED_IFACE}"
    return 0
  fi

  mapfile_or_empty configured load_configured_interfaces
  for candidate in "${configured[@]}"; do
    if interface_is_ready_for_real "${candidate}"; then
      echo "${candidate}"
      return 0
    fi
  done
  for candidate in "${configured[@]}"; do
    if interface_exists "${candidate}"; then
      echo "${candidate}"
      return 0
    fi
  done

  if candidate="$(pick_best_ready_interface)"; then
    echo "${candidate}"
    return 0
  fi
  if candidate="$(pick_best_existing_interface)"; then
    echo "${candidate}"
    return 0
  fi
  return 1
}

IFACE="$(pick_interface || true)"

if [[ -z "${IFACE:-}" ]]; then
  fail "No candidate network interface found. Pass one explicitly: source setup_unitree_dds.sh <iface>"
fi

set +u
source /opt/ros/humble/setup.bash
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export A2_NETWORK_INTERFACE="${IFACE}"
export A2_CYCLONEDDS_BIND_STATUS="diagnostic"
export A2_CYCLONEDDS_BIND_REASON="interface_not_ready"
unset CYCLONEDDS_URI || true

if interface_is_ready_for_real "${IFACE}"; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
  export A2_CYCLONEDDS_BIND_STATUS="bound"
  export A2_CYCLONEDDS_BIND_REASON="ready"
else
  if ! interface_exists "${IFACE}"; then
    export A2_CYCLONEDDS_BIND_REASON="interface_missing"
  elif [[ "${IFACE}" == "lo" ]]; then
    export A2_CYCLONEDDS_BIND_REASON="loopback_not_allowed"
  elif is_virtual_like "${IFACE}"; then
    export A2_CYCLONEDDS_BIND_REASON="virtual_interface"
  elif ! interface_has_carrier "${IFACE}"; then
    export A2_CYCLONEDDS_BIND_REASON="no_carrier"
  else
    export A2_CYCLONEDDS_BIND_REASON="interface_not_ready"
  fi
fi

export A2_REAL_DIAGNOSTIC_ONLY="$([[ "${A2_CYCLONEDDS_BIND_STATUS}" == "bound" ]] && echo 0 || echo 1)"

echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "A2_NETWORK_INTERFACE=${A2_NETWORK_INTERFACE}"
echo "A2_CYCLONEDDS_BIND_STATUS=${A2_CYCLONEDDS_BIND_STATUS}"
echo "A2_CYCLONEDDS_BIND_REASON=${A2_CYCLONEDDS_BIND_REASON}"
echo "A2_REAL_DIAGNOSTIC_ONLY=${A2_REAL_DIAGNOSTIC_ONLY}"
echo "A2_NETWORK_CONFIG=${CONFIG_FILE}"
echo "INTERFACE_DETAIL=$(describe_interface "${IFACE}")"
if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
  echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
else
  echo "CYCLONEDDS_URI=<unset>"
  echo "CycloneDDS binding skipped. ROS 2 will start in diagnostic mode until a ready wired interface is available." >&2
fi
