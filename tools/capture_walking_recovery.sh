#!/usr/bin/env bash
# Passive recorder. This does not start or restart the robot controller.
set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"
source /opt/ros/foxy/setup.bash
source "${HIM_SAFETY_ROS_INSTALL:-$HOME/.him-safety/ros-install}/setup.bash"
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/unitree/cyclonedds_ws/install/cyclonedds}"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$repo_dir:${PYTHONPATH:-}"

exec /usr/bin/python3 tools/capture_telemetry.py \
    --seconds 60 \
    --label walking-push-automatic-recovery \
    --ros-domain 97 \
    --control-trace .cache/control-trace.jsonl \
    --diagnostic-trace .cache/control-diagnostics.jsonl \
    --require-diagnostics "$@"
