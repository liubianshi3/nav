#!/usr/bin/env bash
set -euo pipefail

REMOTE_HOST="${REMOTE_HOST:-a2}"
REMOTE_WS="${REMOTE_WS:-/home/unitree/a2_system_ws}"
REMOTE_USER_HOST="${REMOTE_USER_HOST:-${REMOTE_HOST}}"
DO_DEPLOY="${DO_DEPLOY:-1}"
BUILD_WEB="${BUILD_WEB:-1}"
FORCE_BUILD_WEB="${A2_FORCE_BUILD_WEB:-0}"
IFACE="${A2_NETWORK_INTERFACE:-eth0}"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--host a2] [--iface eth0] [--no-deploy] [--no-build-web] [--force-build-web]

Examples:
  $(basename "$0")
  $(basename "$0") --host a2 --iface eth0
  $(basename "$0") --no-deploy
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
    --no-deploy)
      DO_DEPLOY=0
      shift
      ;;
    --no-build-web)
      BUILD_WEB=0
      shift
      ;;
    --force-build-web)
      FORCE_BUILD_WEB=1
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
  "A2_FORCE_BUILD_WEB=${FORCE_BUILD_WEB}"
  "${REMOTE_WS}/src/a2_system/tools/start_web_console_suite.sh"
  "--iface" "${IFACE}"
)

printf -v REMOTE_CMD_STR '%q ' "${REMOTE_CMD[@]}"

log "Running remote web-console suite on ${REMOTE_USER_HOST}"
ssh "${REMOTE_USER_HOST}" "bash -lc 'cd ${REMOTE_WS} && ${REMOTE_CMD_STR}'"
