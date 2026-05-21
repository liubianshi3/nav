#!/usr/bin/env bash
set -euo pipefail

A2_WORKSPACE="${A2_WORKSPACE:-/opt/a2_system_ws}"
SOCKET_PATH="${A2_UNITREE_AGENT_SOCKET:-/run/a2/unitree_agent.sock}"
SDK_IFACE="${A2_SDK_INTERFACE:-eth0}"
AGENT_BIN="${A2_UNITREE_AGENT_BIN:-${A2_WORKSPACE}/install/a2_unitree_agent/lib/a2_unitree_agent/unitree_agent}"

if [[ ! -x "$AGENT_BIN" ]]; then
  echo "unitree_agent executable not found: ${AGENT_BIN}" >&2
  exit 1
fi

mkdir -p "$(dirname "$SOCKET_PATH")"

export LD_LIBRARY_PATH="/opt/unitree_robotics/lib/x86_64:/opt/unitree_robotics/lib:${LD_LIBRARY_PATH:-}"
if [[ -n "${A2_UNITREE_AGENT_LD_PRELOAD:-}" ]]; then
  export LD_PRELOAD="${A2_UNITREE_AGENT_LD_PRELOAD}"
else
  unset LD_PRELOAD || true
fi

unset ROS_DOMAIN_ID || true
unset RMW_IMPLEMENTATION || true
unset CYCLONEDDS_URI || true
unset FASTDDS_BUILTIN_TRANSPORTS || true

exec "$AGENT_BIN" \
  --socket "$SOCKET_PATH" \
  --interface "$SDK_IFACE" \
  --dds-domain-id "${A2_UNITREE_DDS_DOMAIN_ID:-0}" \
  --command-timeout-ms "${A2_UNITREE_AGENT_COMMAND_TIMEOUT_MS:-300}"
