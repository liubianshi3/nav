#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
PIP_INDEX_URL="${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"

cd "${PROJECT_ROOT}"

if [[ ! -d .venv ]]; then
  "${PYTHON_BIN}" -m venv .venv
fi

if [[ ! -f .venv/bin/activate ]]; then
  rm -rf .venv
  "${PYTHON_BIN}" -m venv .venv
fi

source .venv/bin/activate
pip install -i "${PIP_INDEX_URL}" --upgrade pip
pip install -i "${PIP_INDEX_URL}" -r backend/requirements.txt

echo "Backend virtualenv is ready at ${PROJECT_ROOT}/.venv"
