#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${A2_DOCKER_IMAGE:-a2-system-ws:real}"
CONTAINER_NAME="${A2_DOCKER_TEST_NAME:-a2-system-ws-smoke}"
PORT="${A2_DOCKER_TEST_PORT:-18080}"
HOST_MAP_ROOT="${A2_HOST_MAP_ROOT:-/home/unitree/a2_system_ws/runtime/maps}"
HOST_LOG_ROOT="${A2_HOST_LOG_ROOT:-/home/unitree/a2_system_ws/runtime/logs}"

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker run -d \
  --name "${CONTAINER_NAME}" \
  --net host \
  --privileged \
  -e PORT="${PORT}" \
  -v "${HOST_MAP_ROOT}:/opt/a2_system_ws/runtime/maps" \
  -v "${HOST_LOG_ROOT}:/opt/a2_system_ws/runtime/logs" \
  "${IMAGE_NAME}" >/dev/null

cleanup() {
  docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

for _ in $(seq 1 30); do
  if curl -fsS "http://127.0.0.1:${PORT}/api/health" >/tmp/a2_docker_health.json; then
    cat /tmp/a2_docker_health.json
    echo
    break
  fi
  sleep 1
done

curl -fsS "http://127.0.0.1:${PORT}/api/health" >/dev/null
docker exec "${CONTAINER_NAME}" bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/a2_system_ws/install/setup.bash && ros2 pkg prefix a2_bringup >/dev/null && test -x /opt/a2_system_ws/install/a2_system/share/a2_system/start_real_stack.sh'

echo "Docker smoke test passed on port ${PORT}."
