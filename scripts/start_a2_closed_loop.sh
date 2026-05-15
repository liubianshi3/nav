#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER_HOST="${REMOTE_USER_HOST:-a2}"
REMOTE_WS="${REMOTE_WS:-/home/unitree/a2_system_ws}"
MODE="standby"
MAP_ID=""
LIDAR_IFACE="${A2_JT128_INTERFACE:-net1}"
SDK_IFACE="${A2_SDK_INTERFACE:-eth0}"
CONTROL_IFACE="${A2_CONTROL_INTERFACE:-$SDK_IFACE}"
ENABLE_MOTION=0
LIVE_MOTION=0
SSH_TTY=0

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--host a2] [--mode standby] [--lidar-iface net1]
  $(basename "$0") --mode mapping
  $(basename "$0") --mode navigation --map-id MAP_ID [--enable-motion] [--live-motion]

Default:
  SSH to the robot and bring the Web console to stopped/standby state.

Examples:
  $(basename "$0")
  $(basename "$0") --host a2 --mode mapping
  $(basename "$0") --mode navigation --map-id perfect4-29
  $(basename "$0") --mode navigation --map-id perfect4-29 --enable-motion --live-motion --tty
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      REMOTE_USER_HOST="$2"
      shift 2
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    --map-id)
      MAP_ID="$2"
      shift 2
      ;;
    --lidar-iface|--iface)
      LIDAR_IFACE="$2"
      shift 2
      ;;
    --sdk-iface)
      SDK_IFACE="$2"
      CONTROL_IFACE="$2"
      shift 2
      ;;
    --control-iface)
      CONTROL_IFACE="$2"
      shift 2
      ;;
    --enable-motion)
      ENABLE_MOTION=1
      shift
      ;;
    --live-motion)
      LIVE_MOTION=1
      ENABLE_MOTION=1
      shift
      ;;
    --tty)
      SSH_TTY=1
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

REMOTE_CMD=(
  "${REMOTE_WS}/src/a2_system/tools/start_jt128_3d_closed_loop.sh"
  "--mode" "$MODE"
  "--lidar-iface" "$LIDAR_IFACE"
)

if [[ "$MODE" == "navigation" ]]; then
  REMOTE_CMD+=("--map-id" "$MAP_ID" "--sdk-iface" "$SDK_IFACE" "--control-iface" "$CONTROL_IFACE")
  if (( ENABLE_MOTION == 1 )); then
    REMOTE_CMD+=("--enable-motion")
  fi
  if (( LIVE_MOTION == 1 )); then
    REMOTE_CMD+=("--live-motion")
  fi
fi

printf -v REMOTE_CMD_STR '%q ' "${REMOTE_CMD[@]}"

SSH_ARGS=()
if (( SSH_TTY == 1 )); then
  SSH_ARGS+=("-tt")
fi

echo "[INFO] Running remote A2 closed-loop bringup on ${REMOTE_USER_HOST}"
ssh "${SSH_ARGS[@]}" "${REMOTE_USER_HOST}" "bash -lc 'cd ${REMOTE_WS} && ${REMOTE_CMD_STR}'"
