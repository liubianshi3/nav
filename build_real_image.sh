#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="${A2_DOCKER_IMAGE:-a2-system-ws:real}"
NODE_IMAGE="${A2_NODE_IMAGE:-docker.m.daocloud.io/library/node:20-bookworm-slim}"
ROS_IMAGE="${A2_ROS_IMAGE:-docker.m.daocloud.io/library/ros:humble-ros-base-jammy}"
SDK_CONFIG_PATH="${REPO_ROOT}/docker/unitree_sdk/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"

if [[ ! -f "${SDK_CONFIG_PATH}" ]]; then
  echo "Bundled Unitree SDK not found at ${SDK_CONFIG_PATH}" >&2
  exit 1
fi

cd "${REPO_ROOT}"
docker build \
  --build-arg "NODE_IMAGE=${NODE_IMAGE}" \
  --build-arg "ROS_IMAGE=${ROS_IMAGE}" \
  -f Dockerfile \
  -t "${IMAGE_NAME}" \
  .
