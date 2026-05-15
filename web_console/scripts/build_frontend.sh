#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FRONTEND_DIR="${PROJECT_ROOT}/frontend"

if ! command -v npm >/dev/null 2>&1; then
  echo "npm not found. Install Node.js 18+ before building the frontend." >&2
  exit 1
fi

cd "${FRONTEND_DIR}"
npm install
npm run build

echo "Frontend assets built into ${PROJECT_ROOT}/backend/static"
