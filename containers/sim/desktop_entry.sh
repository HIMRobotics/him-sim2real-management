#!/bin/bash
# Start the ROS bridge (and optional hold) inside the Compose loopback net.
set -eo pipefail
cd /workspace
set +u
source /opt/ros/foxy/setup.bash
source /opt/him_safety/install/setup.bash
set -u
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-97}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/workspace/containers/sim/fastdds.xml}"
export G1_SIM_NETNS=1
export ROBOT_INTERFACE=lo
export PYTHONPATH="/workspace/packages/policy_safety/src:/workspace/robot_safety:/workspace/containers/sim${PYTHONPATH:+:$PYTHONPATH}"
python3 containers/sim/bridge.py
