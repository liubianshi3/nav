#!/usr/bin/env bash
set -euo pipefail

REMOTE_HOST="${REMOTE_HOST:-a2}"
REMOTE_WS="${REMOTE_WS:-/home/unitree/a2_system_ws}"
REMOTE_USER_HOST="${1:-${REMOTE_HOST}}"
START_SERVICE="${START_SERVICE:-0}"
BUILD_WEB="${BUILD_WEB:-1}"
PIP_INDEX_URL="${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"

LOCAL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "== A2 deploy =="
echo "local_root : ${LOCAL_ROOT}"
echo "remote     : ${REMOTE_USER_HOST}:${REMOTE_WS}"
echo "build_web  : ${BUILD_WEB}"
echo "service    : ${START_SERVICE}"
echo "pip_index  : ${PIP_INDEX_URL}"

rsync -az --delete \
  --exclude build \
  --exclude install \
  --exclude log \
  --exclude runtime \
  --exclude .git \
  --exclude web_console/frontend/node_modules \
  --exclude web_console/.venv \
  --exclude web_console/backend/.venv \
  --exclude web_console/backend/config.yaml \
  --exclude web_console/backend/static \
  "${LOCAL_ROOT}/" "${REMOTE_USER_HOST}:${REMOTE_WS}/"

ssh "${REMOTE_USER_HOST}" "bash -lc '
set -euo pipefail
cd ${REMOTE_WS}
rm -rf build/autoware_cmake build/autoware_ndt_scan_matcher \
       install/autoware_cmake install/autoware_ndt_scan_matcher
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --packages-select \
  a2_interfaces unitree_api a2_system a2_bringup localization_manager nav2_integration tf_manager \
  a2_state_publisher safety_manager map_manager slam_manager sensor_sync a2_sdk_bridge a2_control_bridge \
  nav_health_monitor exploration_manager a2_ndt_adapter \
  a2_ground_segmentation_cpp nav2_integration_cpp
set +u
source install/setup.bash
set -u
ros2 run a2_system config_schema_check.py
ros2 run a2_system nav_contract_check.py
if [ \"${BUILD_WEB}\" = \"1\" ]; then
  cd ${REMOTE_WS}/web_console
  PIP_INDEX_URL=${PIP_INDEX_URL} ./scripts/bootstrap_backend.sh
  ./scripts/build_frontend.sh
fi
if [ \"${START_SERVICE}\" = \"1\" ]; then
  sudo systemctl daemon-reload
  sudo systemctl restart a2-web-console.service
fi
echo DEPLOY_OK
'"
