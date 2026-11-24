#!/usr/bin/env bash
# Boot the soft-reset and hard-damp gate only. No arm-raise test is started.
set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
install_base="${HIM_SAFETY_ROS_INSTALL:-$HOME/.him-safety/ros-install}"
profile="${HIM_SAFETY_PROFILE:-$repo_dir/robot_safety/profiles/g1_type5.template.json}"
# Looser manual tests: HIM_SAFETY_PROFILE=$repo_dir/robot_safety/profiles/g1_type5.lenient.json

cd "$repo_dir"
source /opt/ros/foxy/setup.bash
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/unitree/cyclonedds_ws/install/cyclonedds}"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=97
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$repo_dir:${PYTHONPATH:-}"
export PYTHONUNBUFFERED=1
export HIM_BALANCE_ONNX="${HIM_BALANCE_ONNX:-$repo_dir/containers/examples/policies/walker/balance.onnx}"
export G1_STARTUP_POLICY="${G1_STARTUP_POLICY:-groot}"
case "${G1_OUTPUT_ON_COMMAND:-false}" in
  true|1) output_on_command=true ;;
  false|0) output_on_command=false ;;
  *) echo "G1_OUTPUT_ON_COMMAND must be true or false." >&2; exit 2 ;;
esac
recovery_blend_secs="${G1_RECOVERY_BLEND_SECS:-0}"
if ! [[ "$recovery_blend_secs" =~ ^[0-9]+([.][0-9]+)?$ ]] \
    || ! awk -v value="$recovery_blend_secs" 'BEGIN { exit !(value + 0 <= 3) }'; then
  echo "G1_RECOVERY_BLEND_SECS must be seconds from 0 to 3." >&2; exit 2
fi
# Typed values for the ROS parameters: a bool and a double.
export G1_OUTPUT_ON_COMMAND="$output_on_command"
G1_RECOVERY_BLEND_SECS="$(LC_ALL=C awk -v value="$recovery_blend_secs" 'BEGIN { printf "%.3f", value + 0 }')"
export G1_RECOVERY_BLEND_SECS

mkdir -p "$install_base"
colcon --log-base "$HOME/.him-safety/ros-log" build --base-paths ros2 \
    --build-base "$HOME/.him-safety/ros-build" \
    --install-base "$install_base" \
    --executor sequential --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
source "$install_base/setup.bash"

exec ros2 run him_safety_nodes safety_node hardware_gate --ros-args \
    -p enable_hardware_output:=true \
    -p whole_body_output:=true \
    -p launch_gate:=true \
    -p "stance_blend_secs:=${G1_STANCE_BLEND_SECS:-5}" \
    -p "limit_mode:=${G1_LIMIT_MODE:-stop}" \
    -p "output_on_command:=${G1_OUTPUT_ON_COMMAND:-false}" \
    -p "recovery_blend_secs:=${G1_RECOVERY_BLEND_SECS:-0}" \
    -p "hardware_profile_path:=$profile" \
    -p sdk_interface:=eth0 \
    -p sdk_domain:=0
