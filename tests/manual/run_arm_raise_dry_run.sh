#!/usr/bin/env bash
# Build and launch the diagnostic arm test, or explicitly request one run.
set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_dir"
source /opt/ros/foxy/setup.bash
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/unitree/cyclonedds_ws/install/cyclonedds}"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=97
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$repo_dir:${PYTHONPATH:-}"
export HIM_BALANCE_ONNX="$repo_dir/.cache/groot_balance/GR00T-WholeBodyControl-Balance.onnx"

# Use one overlay for launch and the separate explicit start request.
if [[ "${1:-}" == "--start" || "${1:-}" == "--check-ready" || "${1:-}" == "--joint-deltas" ]]; then
    source /tmp/him-safety-ros-install/setup.bash
    if [[ "$1" == "--joint-deltas" ]]; then
        exec ros2 topic echo /safety/joint_diagnostics std_msgs/msg/String
    fi
    if [[ "$1" == "--check-ready" ]]; then
        exec ros2 service call /safety/motor_gate/ready std_srvs/srv/Trigger '{}'
    fi
    exec ros2 service call /safety/arm_raise/start std_srvs/srv/Trigger '{}'
fi

colcon --log-base /tmp/him-safety-ros-log build --base-paths ros2 \
    --build-base /tmp/him-safety-ros-build \
    --install-base /tmp/him-safety-ros-install \
    --executor sequential --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
source /tmp/him-safety-ros-install/setup.bash
exec ros2 launch "$repo_dir/tests/manual/arm_raise_dry_run.launch.py" "$@"
