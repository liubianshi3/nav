#!/usr/bin/env bash
set -euo pipefail

REMOTE_HOST="${REMOTE_HOST:-a2}"
REMOTE_WS="${REMOTE_WS:-/home/unitree/a2_system_ws}"
REMOTE_USER_HOST="${REMOTE_USER_HOST:-${REMOTE_HOST}}"
DO_DEPLOY="${DO_DEPLOY:-1}"
BUILD_WEB="${BUILD_WEB:-1}"
IFACE="${A2_NETWORK_INTERFACE:-eth0}"
MAP_YAML_REMOTE="${A2_MAP_YAML:-${REMOTE_WS}/runtime/maps/test_map_20260423_1059/map.yaml}"

SET_INITIAL_POSE=0
POSE_X=""
POSE_Y=""
POSE_YAW="0.0"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--host a2] [--iface eth0] [--map-yaml /remote/path/map.yaml]
                     [--initial-pose X Y [YAW]] [--no-deploy] [--no-build-web]

Examples:
  $(basename "$0")
  $(basename "$0") --host a2 --iface eth0
  $(basename "$0") --initial-pose 0.0 0.0 0.0
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      REMOTE_USER_HOST="$2"
      shift 2
      ;;
    --iface)
      IFACE="$2"
      shift 2
      ;;
    --map-yaml)
      MAP_YAML_REMOTE="$2"
      shift 2
      ;;
    --initial-pose)
      SET_INITIAL_POSE=1
      POSE_X="$2"
      POSE_Y="$3"
      if [[ $# -ge 4 ]] && [[ ! "$4" =~ ^-- ]]; then
        POSE_YAW="$4"
        shift 4
      else
        shift 3
      fi
      ;;
    --no-deploy)
      DO_DEPLOY=0
      shift
      ;;
    --no-build-web)
      BUILD_WEB=0
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

log() {
  printf '[INFO] %s\n' "$*"
}

LOCAL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if (( DO_DEPLOY == 1 )); then
  log "Deploying workspace to ${REMOTE_USER_HOST}"
  BUILD_WEB="${BUILD_WEB}" START_SERVICE=0 "${LOCAL_ROOT}/scripts/deploy_to_a2.sh" "${REMOTE_USER_HOST}"
fi

REMOTE_CMD=(
  "${REMOTE_WS}/src/a2_system/tools/start_real1_suite.sh"
  "--iface" "${IFACE}"
  "--map-yaml" "${MAP_YAML_REMOTE}"
)

if (( SET_INITIAL_POSE == 1 )); then
  REMOTE_CMD+=("--initial-pose" "${POSE_X}" "${POSE_Y}" "${POSE_YAW}")
fi

printf -v REMOTE_CMD_STR '%q ' "${REMOTE_CMD[@]}"

log "Running remote one-click startup on ${REMOTE_USER_HOST}"
ssh "${REMOTE_USER_HOST}" "bash -lc 'cd ${REMOTE_WS} && ${REMOTE_CMD_STR}'"
