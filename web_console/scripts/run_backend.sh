#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_PATH="${CONFIG_PATH:-${PROJECT_ROOT}/backend/config.example.yaml}"
HOST="${HOST:-}"
PORT="${PORT:-}"

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi

WORKSPACE_SETUP="${PROJECT_ROOT%/web_console}/install/setup.bash"
if [[ -f "${WORKSPACE_SETUP}" ]]; then
  source "${WORKSPACE_SETUP}"
fi
set -u

if [[ -f "${PROJECT_ROOT}/.venv/bin/activate" ]]; then
  source "${PROJECT_ROOT}/.venv/bin/activate"
fi

cd "${PROJECT_ROOT}"

PYTHON_BIN="${PROJECT_ROOT}/.venv/bin/python"
if [[ ! -x "${PYTHON_BIN}" ]]; then
  PYTHON_BIN="$(command -v python3)"
fi

ARGS=("${PYTHON_BIN}" -m backend.main --config "${CONFIG_PATH}")
if [[ -n "${HOST}" ]]; then
  ARGS+=(--host "${HOST}")
fi
if [[ -n "${PORT}" ]]; then
  ARGS+=(--port "${PORT}")
fi

exec "${ARGS[@]}"
