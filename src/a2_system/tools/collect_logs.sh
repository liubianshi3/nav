#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/ws/device-navigation}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${WORKSPACE}/runtime/log_bundle_${STAMP}"
mkdir -p "${OUT_DIR}"

if [[ -d "${WORKSPACE}/runtime/logs" ]]; then
  cp -r "${WORKSPACE}/runtime/logs" "${OUT_DIR}/runtime_logs"
fi

if [[ -d "$HOME/.ros/log" ]]; then
  cp -r "$HOME/.ros/log" "${OUT_DIR}/ros_logs"
fi

if [[ -f "${WORKSPACE}/src/a2_system/config/network.yaml" ]]; then
  cp "${WORKSPACE}/src/a2_system/config/network.yaml" "${OUT_DIR}/"
fi

tar -C "${WORKSPACE}/runtime" -czf "${WORKSPACE}/runtime/log_bundle_${STAMP}.tar.gz" "log_bundle_${STAMP}"
echo "${WORKSPACE}/runtime/log_bundle_${STAMP}.tar.gz"
