#!/usr/bin/env bash
# ROS setup reads unset variables. nounset must stay off around those files.
# Leave ROS_LOCALHOST_ONLY at 0. Foxy's localhost path adds shared memory and
# never sets a unicast peer, so it cannot see the sim bridge on lo.
# containers/sim/fastdds.xml does that unicast discovery instead.
set -eo pipefail
localhost_only="${ROS_LOCALHOST_ONLY:-0}"
domain_id="${ROS_DOMAIN_ID:-97}"
rmw="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
profile="${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
source /opt/ros/foxy/setup.bash
source /opt/walker/install/setup.bash
export ROS_LOCALHOST_ONLY="$localhost_only"
export ROS_DOMAIN_ID="$domain_id"
export RMW_IMPLEMENTATION="$rmw"
if [[ -n "$profile" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$profile"
fi
export WALKER_COMMAND="${WALKER_COMMAND:-/opt/walker/stand.yaml}"
echo "walker node domain=$ROS_DOMAIN_ID localhost=$ROS_LOCALHOST_ONLY rmw=$RMW_IMPLEMENTATION command=$WALKER_COMMAND"
exec python3 /opt/walker/walker_node.py
