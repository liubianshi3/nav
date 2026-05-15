#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_NAME="${SERVICE_NAME:-a2-web-console.service}"
TARGET_DIR="${TARGET_DIR:-/etc/systemd/system}"

if [[ $EUID -ne 0 ]]; then
  echo "Please run with sudo." >&2
  exit 1
fi

install -m 0644 "${PROJECT_ROOT}/systemd/${SERVICE_NAME}" "${TARGET_DIR}/${SERVICE_NAME}"
systemctl daemon-reload
echo "Installed ${SERVICE_NAME} into ${TARGET_DIR}"
echo "Next:"
echo "  systemctl enable --now ${SERVICE_NAME}"
