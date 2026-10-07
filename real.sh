#!/usr/bin/env bash
# Real-robot counterpart of sim.sh. Same container and arguments, on the robot.
# The host eases to the container's first joint command, then waits.
# Left and B together launch the policy. Double Start requests stock damping.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
upload_dir="$repo_dir/containers/upload"
runtime="${CONTAINER_RUNTIME:-docker}"
original_args=("$@")
if [[ "${1:-}" != "--help" && "${1:-}" != "-h" && "${EUID}" -ne 0 && ! -w /var/run/docker.sock ]]; then
  echo "Docker socket is not writable. Re-running with sudo."
  exec sudo --preserve-env=HOME,G1_SIM_PYTHON,G1_JOINT_PROFILE,G1_COMMAND_FILE,G1_IMAGE,G1_LAUNCH_GATE,G1_USE_SAFETY_SERVICE,G1_STANCE_BLEND_SECS,G1_SIM_EXAMPLE,G1_THRASH_MODE,G1_TRIAL_CONFIG,G1_TASK_COMMAND,G1_POLICY_ROS_DOMAIN,G1_REQUIRE_BASE_STATE,G1_BASE_STATE_TOPIC,G1_BASE_VELOCITY_FRAME,G1_NVIDIA,G1_RUN_LOG,HIM_SAFETY_ROS_INSTALL,ROBOT_INTERFACE,ROBOT_DOMAIN,PYTHON,CONTAINER_RUNTIME -- "$0" "${original_args[@]}"
fi
python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
[[ -n "${G1_SIM_PYTHON:-}" || -x "$python" ]] || python="${PYTHON:-python3}"
profile="${G1_JOINT_PROFILE:-$repo_dir/robot_safety/profiles/g1_type5.template.json}"
image="${G1_IMAGE:-}"
example=""
command_file="${G1_COMMAND_FILE:-}"
extra=()

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./real.sh [--image NAME] [--example NAME] [--command FILE]
                 [--profile PATH] [--] [container args...]
       ./real.sh stop

Runs a policy container on this robot, not in MuJoCo. --example builds
stairs, walker, or balance the same way ./sim.sh does. With no --image
and no --example, the one archive in containers/upload is used.

g1-safety-stops.service stays up. The container publishes /safety/precheck.
The checker forwards accepted commands to /safety/postcheck. That service
is the hardware gate: it applies joint limits, eases to the first pose,
waits for left + B, writes the motors, and double Start damps.
This script does not publish rt/lowcmd or rt/arm_sdk.

--command FILE     Velocity yaml, mounted for the container as G1_COMMAND_FILE.
--profile PATH     Joint profile. An example uses the profile from its bundle.
HELP
  exit 0
fi

if [[ "${1:-}" == "stop" ]]; then
  exec "$repo_dir/setup.sh" stop
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) image="${2:?--image needs a name}"; shift 2 ;;
    --example) example="${2:?--example needs a name}"; shift 2 ;;
    --command) command_file="${2:?--command needs a yaml file}"; shift 2 ;;
    --profile) profile="${2:?--profile needs a JSON file}"; shift 2 ;;
    --) shift; extra+=("$@"); break ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ -n "$example" && -n "$image" ]]; then
  echo "Use --example or --image, not both." >&2
  exit 2
fi
if [[ -n "$command_file" && ! -f "$command_file" ]]; then
  echo "Command file is not readable: $command_file" >&2
  exit 2
fi

if [[ -n "$example" ]]; then
  example_name="$("$python" "$repo_dir/containers/examples/prepare.py" "$example" --settings | awk '{print $1}')"
  prepared="$("$python" "$repo_dir/containers/examples/prepare.py" "$example")"
  image="g1-real-example-${example_name}:local"
  "$runtime" build -t "$image" "$prepared/container"
  profile="$prepared/container/project/joint_profile.json"
  export G1_SIM_EXAMPLE="$example_name"
fi

export G1_USE_SAFETY_SERVICE=1
export G1_POLICY_ROS_DOMAIN=97
if [[ -z "${G1_BASE_STATE_TOPIC:-}" ]]; then
  export G1_REQUIRE_BASE_STATE=false
fi
export G1_IMAGE="$image"
if [[ -n "$command_file" ]]; then
  export G1_COMMAND_FILE="$(realpath "$command_file")"
fi

setup_args=(--mode hardware --enable-hardware-output --profile "$profile")
[[ -z "$image" ]] || setup_args+=(--image "$image")
if [[ ${#extra[@]} -gt 0 ]]; then
  setup_args+=(-- "${extra[@]}")
fi
exec "$repo_dir/setup.sh" "${setup_args[@]}"
