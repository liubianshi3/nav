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

set +u
source /opt/ros/humble/setup.bash
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source "${WORKSPACE}/install/setup.bash"
fi
set -u

cd "${WORKSPACE}"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/a2_system/test \
  src/nav2_integration/test \
  src/localization_manager/test \
  src/tf_manager/test \
  web_console/backend/test
