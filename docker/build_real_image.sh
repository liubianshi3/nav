#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="${A2_DOCKER_IMAGE:-a2-system-ws:real}"
UNITREE_SDK_PATH="${UNITREE_SDK_PATH:-/opt/unitree_robotics}"
NODE_IMAGE="${A2_NODE_IMAGE:-docker.m.daocloud.io/library/node:20-bookworm-slim}"
ROS_IMAGE="${A2_ROS_IMAGE:-docker.m.daocloud.io/library/ros:humble-ros-base-jammy}"

if [[ ! -f "${UNITREE_SDK_PATH}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake" ]]; then
  echo "Unitree SDK not found at ${UNITREE_SDK_PATH}" >&2
  echo "This real image must be built on A2 or with UNITREE_SDK_PATH pointing to a valid SDK." >&2
  exit 1
fi

cd "${REPO_ROOT}"
DOCKER_BUILDKIT=1 docker build \
  --build-context "unitree_sdk=${UNITREE_SDK_PATH}" \
  --build-arg "NODE_IMAGE=${NODE_IMAGE}" \
  --build-arg "ROS_IMAGE=${ROS_IMAGE}" \
  -f docker/Dockerfile.real \
  -t "${IMAGE_NAME}" \
  .
