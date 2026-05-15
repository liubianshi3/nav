#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${A2_WORKSPACE:-$HOME/a2_system_ws}"
IFACE="${A2_NETWORK_INTERFACE:-eth0}"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--iface eth0] [--force-build-web] [--skip-native-lidar]

Starts the A2 3D-first Web console suite:
  - stops old ROS/Nav2/AMCL/2D projection stacks
  - starts/reuses Unitree front-LiDAR source
  - starts the Web console service
  - leaves mapping/navigation selection to the Web UI

After startup open:
  http://<robot-lan-ip>:8080/

Optional dry-run 3D goal preflight:
  ros2 run a2_system goal_pose_3d_smoke_test.py

Optional real short-goal test, only in a clear area:
  ros2 run a2_system goal_pose_3d_smoke_test.py --execute --i-understand-robot-will-move
EOF
}

ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface)
      IFACE="$2"
      ARGS+=("--iface" "$2")
      shift 2
      ;;
    --force-build-web|--skip-native-lidar)
      ARGS+=("$1")
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

export A2_WORKSPACE="${WORKSPACE}"
export A2_NETWORK_INTERFACE="${IFACE}"
export A2_ENABLE_NAV2=false
export A2_REAL_LOCALIZATION_MODE=uslam_odom
export A2_NATIVE_LIDAR_TOPIC="${A2_NATIVE_LIDAR_TOPIC:-/jt128/front/points}"

SCRIPT="${WORKSPACE}/install/a2_system/share/a2_system/start_web_console_suite.sh"
if [[ ! -x "$SCRIPT" ]]; then
  SCRIPT="${WORKSPACE}/src/a2_system/tools/start_web_console_suite.sh"
fi

exec "$SCRIPT" "${ARGS[@]}"
