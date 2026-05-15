#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
export UNITREE_SDK2_ROOT="${UNITREE_SDK2_ROOT:-$HOME/unitree_sdk2}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "UNITREE_SDK2_ROOT=${UNITREE_SDK2_ROOT}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
