#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export A2_UNITREE_AGENT_SOCKET="${A2_UNITREE_AGENT_SOCKET:-/run/a2/unitree_agent.sock}"

echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "A2_UNITREE_AGENT_SOCKET=${A2_UNITREE_AGENT_SOCKET}"
