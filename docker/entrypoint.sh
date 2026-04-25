#!/usr/bin/env bash
set -euo pipefail

export A2_WORKSPACE="${A2_WORKSPACE:-/opt/a2_system_ws}"
export CONFIG_PATH="${CONFIG_PATH:-${A2_WORKSPACE}/web_console/backend/config.docker.yaml}"
export LD_LIBRARY_PATH="/opt/unitree_robotics/lib:/opt/unitree_robotics/lib/x86_64:${LD_LIBRARY_PATH:-}"

mkdir -p "${A2_WORKSPACE}/runtime/maps" "${A2_WORKSPACE}/runtime/logs"

exec "${A2_WORKSPACE}/web_console/scripts/run_backend.sh" "$@"
