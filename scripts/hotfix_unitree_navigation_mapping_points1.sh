#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER_HOST="${1:-a2}"
REMOTE_FILE="${REMOTE_FILE:-/home/unitree/graph_pid_ws/bin/tools/py-planner/navigation_mapping.py}"
REMOTE_SERVICE="${REMOTE_SERVICE:-unitree_slam.service}"
RESTART_SERVICE="${RESTART_SERVICE:-1}"
SUDO_PASSWORD="${SUDO_PASSWORD:-}"

tmpdir="$(mktemp -d)"
cleanup() {
  rm -rf "${tmpdir}"
}
trap cleanup EXIT

local_file="${tmpdir}/navigation_mapping.py"

echo "== Unitree SLAM single-lidar hotfix =="
echo "remote      : ${REMOTE_USER_HOST}"
echo "remote_file : ${REMOTE_FILE}"
echo "service     : ${REMOTE_SERVICE}"
echo "restart     : ${RESTART_SERVICE}"

ssh "${REMOTE_USER_HOST}" "bash -lc 'test -f \"${REMOTE_FILE}\"'"
ssh "${REMOTE_USER_HOST}" "bash -lc 'cp \"${REMOTE_FILE}\" \"${REMOTE_FILE}.bak-\$(date +%F-%H%M%S)\"'"
scp "${REMOTE_USER_HOST}:${REMOTE_FILE}" "${local_file}"

python3 - "${local_file}" <<'PY'
import sys
from pathlib import Path

path = Path(sys.argv[1])
text = path.read_text()
original = text

declare_anchor = "        self.declare_parameter('enable_timestamp_sync', True)\n"
if "self.declare_parameter('pointcloud_topic'," not in text:
    if declare_anchor not in text:
        raise SystemExit("Missing declare_parameter anchor")
    text = text.replace(
        declare_anchor,
        declare_anchor + "        self.declare_parameter('pointcloud_topic', '/unitree/slam_lidar/points1')\n",
        1,
    )

param_anchor = "        self.enable_timestamp_sync = self.get_parameter('enable_timestamp_sync').value\n"
if "self.pointcloud_topic = self.get_parameter('pointcloud_topic').value" not in text:
    if param_anchor not in text:
        raise SystemExit("Missing parameter assignment anchor")
    text = text.replace(
        param_anchor,
        param_anchor + "        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value\n",
        1,
    )

old_subscription = (
    "        self.pointcloud_sub = self.create_subscription(\n"
    "            PointCloud2, '/rslidar_points', self.pointcloud_callback, 1)\n"
)
new_subscription = (
    "        self.pointcloud_sub = self.create_subscription(\n"
    "            PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 1)\n"
)
if old_subscription in text:
    text = text.replace(old_subscription, new_subscription, 1)
elif new_subscription not in text:
    raise SystemExit("Missing subscription anchor")

log_anchor = "        self.get_logger().info('导航地图节点初始化完成')\n"
new_log = "        self.get_logger().info(f'点云输入 topic: {self.pointcloud_topic}')\n"
if new_log not in text:
    if log_anchor not in text:
        raise SystemExit("Missing logger anchor")
    text = text.replace(log_anchor, new_log + log_anchor, 1)

if text == original:
    print("No changes needed; file already patched.")
else:
    path.write_text(text)
    print("Patched navigation_mapping.py")
PY

scp "${local_file}" "${REMOTE_USER_HOST}:${REMOTE_FILE}"

if [[ "${RESTART_SERVICE}" == "1" ]]; then
  if [[ -n "${SUDO_PASSWORD}" ]]; then
    ssh "${REMOTE_USER_HOST}" "bash -lc 'printf \"%s\n\" \"${SUDO_PASSWORD}\" | sudo -S systemctl restart \"${REMOTE_SERVICE}\"'"
  else
    ssh "${REMOTE_USER_HOST}" "bash -lc 'sudo -n systemctl restart \"${REMOTE_SERVICE}\"'"
  fi
  sleep 8
fi

ssh "${REMOTE_USER_HOST}" "bash -lc '
set -euo pipefail
source /opt/ros/humble/setup.bash
ros2 node info /navigation_mapping_node 2>/dev/null | sed -n \"1,80p\"
printf \"\n====\n\"
ros2 topic info /unitree/slam_lidar/points1 2>/dev/null || true
'"
