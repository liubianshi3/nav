#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/a2_system_ws}"
OUT_DIR="${WORKSPACE}/runtime/bags/$(date +%Y%m%d_%H%M%S)"
TOPIC_FILE="${WORKSPACE}/src/a2_system/config/rosbag_topics.txt"
mkdir -p "${OUT_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

if [[ ! -f "${TOPIC_FILE}" ]]; then
  echo "Topic file not found: ${TOPIC_FILE}" >&2
  exit 1
fi

mapfile -t TOPICS < <(grep -v '^\s*$' "${TOPIC_FILE}")
ros2 bag record "${TOPICS[@]}" -o "${OUT_DIR}"
