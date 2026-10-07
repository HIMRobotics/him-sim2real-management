#!/usr/bin/env bash
# Unified policy submission entry point for simulation and hardware.
# The host eases to the container's first joint command, then waits.
# Left and B together launch the policy. Double Start requests stock damping.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" || $# -eq 0 ]]; then
  cat <<'HELP'
Usage: ./submit.sh --mode sim [sim.sh options]
       ./submit.sh --mode real [--image NAME | --example NAME] [--command FILE]
                   [--profile PATH] [--] [container args...]
       ./submit.sh --mode real stop

Simulation: --example walker --local --seconds 10 --view
Hardware: the standing safety gate waits for left + B. Double Start damps.
Use ./sim.sh --help for all simulation options.
HELP
  exit 0
fi
if [[ "${1:-}" != --mode || $# -lt 2 ]]; then
  echo "Select --mode sim or --mode real." >&2
  exit 2
fi
mode="$2"
shift 2
case "$mode" in
  sim) exec "$repo_dir/sim.sh" "$@" ;;
  real|hardware) ;;
  *) echo "Select --mode sim or --mode real." >&2; exit 2 ;;
esac
original_args=("$@")
runtime="${CONTAINER_RUNTIME:-docker}"
python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
[[ -n "${G1_SIM_PYTHON:-}" || -x "$python" ]] || python="${PYTHON:-python3}"
profile="${G1_JOINT_PROFILE:-$repo_dir/robot_safety/profiles/g1_type5.template.json}"
image="${G1_IMAGE:-}"
profile_set=0
[[ -z "${G1_JOINT_PROFILE:-}" ]] || profile_set=1
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
  stopping=1
  shift
else
  stopping=0
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) image="${2:?--image needs a name}"; shift 2 ;;
    --example) example="${2:?--example needs a name}"; shift 2 ;;
    --command) command_file="${2:?--command needs a yaml file}"; shift 2 ;;
    --profile) profile="${2:?--profile needs a JSON file}"; profile_set=1; shift 2 ;;
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

if [[ "$runtime" == docker && "${EUID}" -ne 0 && ! -w /var/run/docker.sock ]]; then
  echo "Docker socket is not writable. Re-running with sudo."
  exec sudo --preserve-env=HOME,G1_SIM_PYTHON,G1_JOINT_PROFILE,G1_COMMAND_FILE,G1_IMAGE,G1_LAUNCH_GATE,G1_USE_SAFETY_SERVICE,G1_STANCE_BLEND_SECS,G1_SIM_EXAMPLE,G1_THRASH_MODE,G1_TRIAL_CONFIG,G1_TASK_COMMAND,G1_POLICY_ROS_DOMAIN,G1_REQUIRE_BASE_STATE,G1_BASE_STATE_TOPIC,G1_BASE_VELOCITY_FRAME,G1_NVIDIA,G1_RUN_LOG,HIM_SAFETY_ROS_INSTALL,ROBOT_INTERFACE,ROBOT_DOMAIN,PYTHON,CONTAINER_RUNTIME -- "$0" --mode real "${original_args[@]}"
fi

if [[ "$stopping" -eq 1 ]]; then
  exec "$repo_dir/setup.sh" stop
fi

if [[ -n "$example" ]]; then
  example_name="$("$python" "$repo_dir/containers/examples/prepare.py" "$example" --settings | awk '{print $1}')"
  prepared="$("$python" "$repo_dir/containers/examples/prepare.py" "$example")"
  image="g1-real-example-${example_name}:local"
  "$runtime" build -t "$image" "$prepared/container"
  [[ "$profile_set" -eq 1 ]] || profile="$prepared/container/project/joint_profile.json"
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
