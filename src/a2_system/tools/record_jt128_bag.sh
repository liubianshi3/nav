#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/ws/device-navigation}"
BAG_ROOT="${A2_BAG_ROOT:-${WORKSPACE}/runtime/bags}"
TOPICS_FILE="${A2_JT128_BAG_TOPICS:-${WORKSPACE}/install/a2_system/share/a2_system/config/jt128_rosbag_topics.txt}"
if [[ ! -f "$TOPICS_FILE" ]]; then
  TOPICS_FILE="${WORKSPACE}/src/a2_system/config/jt128_rosbag_topics.txt"
fi

mkdir -p "$BAG_ROOT"
BAG_NAME="${1:-jt128_$(date +%Y%m%d_%H%M%S)}"
BAG_PATH="${BAG_ROOT}/${BAG_NAME}"

source /opt/ros/humble/setup.bash
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
  source "${WORKSPACE}/install/setup.bash"
fi

mapfile -t TOPICS < <(grep -E '^/' "$TOPICS_FILE")
if [[ "${#TOPICS[@]}" -eq 0 ]]; then
  echo "[ERROR] no topics found in $TOPICS_FILE" >&2
  exit 1
fi

echo "[INFO] Recording JT128 bag to $BAG_PATH"
printf '[INFO] topic %s\n' "${TOPICS[@]}"
exec ros2 bag record -o "$BAG_PATH" "${TOPICS[@]}"
