#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "${WORKSPACE:-}" ]; then
  if [ -d "${SCRIPT_DIR}/../../../src/a2_system" ]; then
    WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
  else
    WORKSPACE="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
  fi
fi
REPORT_ROOT="${REPORT_ROOT:-/tmp/a2_mock_scan_mission_reports}"
CONFIG_FILE="${CONFIG_FILE:-${WORKSPACE}/src/a2_system/config/scan_mission.yaml}"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-${WORKSPACE}/src/a2_system/config/scan_waypoints.example.yaml}"

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

rm -rf "${REPORT_ROOT}"
mkdir -p "${REPORT_ROOT}"

run_case() {
  local name="$1"
  local result_mode="$2"
  local localization_ok="$3"
  local expected_outcome="$4"
  local case_report_root="${REPORT_ROOT}/${name}"
  local case_domain_id="$((120 + RANDOM % 80))"
  mkdir -p "${case_report_root}"

  echo "== mock scan mission case: ${name} domain=${case_domain_id} =="
  export ROS_DOMAIN_ID="${case_domain_id}"
  ros2 run a2_system mock_scan_mission_harness.py --ros-args \
    -p result_mode:="${result_mode}" \
    -p publish_localization_ok:="${localization_ok}" &
  local harness_pid=$!
  trap 'kill ${harness_pid} >/dev/null 2>&1 || true' RETURN
  sleep 1.0

  ros2 run a2_system auto_scan_mission.py --ros-args \
    --params-file "${CONFIG_FILE}" \
    -p waypoints_file:="${WAYPOINTS_FILE}" \
    -p reports_root:="${case_report_root}" \
    -p goal_result_timeout_sec:=3.0 \
    -p preflight_timeout_sec:=3.0 \
    -p save_map_on_finish:=false \
    -p save_map_on_failure:=false

  kill "${harness_pid}" >/dev/null 2>&1 || true
  wait "${harness_pid}" 2>/dev/null || true
  trap - RETURN

  python3 - "$case_report_root" "$expected_outcome" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
expected = sys.argv[2]
reports = sorted(root.glob("*.json"))
if not reports:
    raise SystemExit(f"no json report under {root}")
payload = json.loads(reports[-1].read_text(encoding="utf-8"))
actual = payload["summary"]["outcome"]
if actual != expected:
    raise SystemExit(f"expected outcome {expected}, got {actual}")
print(f"PASS {root.name}: outcome={actual}")
PY
}

run_case "succeeded" "succeeded" "true" "succeeded"
run_case "aborted" "aborted" "true" "failed"
run_case "rejected" "reject" "true" "failed"
run_case "localization_lost" "succeeded" "false" "failed"

echo "PASS: mock scan mission integration cases passed."
