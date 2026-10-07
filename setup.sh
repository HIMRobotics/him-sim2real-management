#!/usr/bin/env bash
# Load the container dropped in containers/upload and run it on the robot network.
# The image's own entrypoint decides what to execute. This script does not
# interpret models. Host nodes own checked motor output.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
upload_dir="$repo_dir/containers/upload"
container_name=g1-user-container
runtime="${CONTAINER_RUNTIME:-}"

if [[ -z "$runtime" ]]; then
  if command -v docker >/dev/null 2>&1; then
    runtime=docker
  elif command -v podman >/dev/null 2>&1; then
    runtime=podman
  else
    echo "Install Docker or Podman on this G1 before running setup.sh." >&2
    exit 1
  fi
fi

if [[ "${EUID}" -ne 0 && ! -w /var/run/docker.sock && "$runtime" == docker ]]; then
  echo "Docker socket is not writable. Run with Docker access as the robot user." >&2
  exit 1
fi

stop_container() {
  if "$runtime" ps -a --format '{{.Names}}' | grep -qx "$container_name"; then
    "$runtime" rm -f "$container_name" >/dev/null
  fi
}

if [[ "${1:-}" == "stop" ]]; then
  stop_container
  echo "Stopped $container_name."
  exit 0
fi

detach=0
mode=preflight
enabled=false
profile="${G1_JOINT_PROFILE:-}"
image="${G1_IMAGE:-}"
extra=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --detach) detach=1; shift ;;
    --image) image="${2:?--image needs a name}"; shift 2 ;;
    --mode) mode="${2:?--mode needs preflight, dry-run or hardware}"; shift 2 ;;
    --profile) profile="${2:?--profile needs a JSON file}"; shift 2 ;;
    --enable-hardware-output) enabled=true; shift ;;
    --) shift; extra+=("$@"); break ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
case "$mode" in preflight|dry-run|hardware) ;; *) echo "Invalid mode: $mode" >&2; exit 2 ;; esac
if [[ "$mode" == hardware && "$enabled" != true ]]; then
  echo "Hardware requires --enable-hardware-output." >&2; exit 2
fi
if [[ "$mode" != hardware && "$enabled" == true ]]; then
  echo "Output enable is only valid in hardware mode." >&2; exit 2
fi
if [[ "$detach" == 1 ]]; then
  echo "Use a foreground session so its safety processes remain supervised." >&2; exit 2
fi
[[ -f "$profile" ]] || { echo "Supply --profile with the approved joint profile." >&2; exit 2; }
profile="$(realpath "$profile")"
PYTHONPATH="$repo_dir" /usr/bin/python3 -c 'import sys; from robot_safety.hardware_profile import HardwareProfile; HardwareProfile.load(sys.argv[1])' "$profile"

if [[ -z "$image" ]]; then
  shopt -s nullglob
  archives=("$upload_dir"/*.tar "$upload_dir"/*.tar.gz)
  shopt -u nullglob
  if [[ -f "$upload_dir/Dockerfile" && ${#archives[@]} -gt 0 ]]; then
    echo "containers/upload has both a Dockerfile and an image archive. Leave one." >&2
    exit 1
  fi
  if [[ ${#archives[@]} -gt 1 ]]; then
    echo "containers/upload has more than one image archive. Leave one." >&2
    exit 1
  fi
  if [[ ${#archives[@]} -eq 1 ]]; then
    echo "Loading ${archives[0]}"
    loaded="$("$runtime" load -i "${archives[0]}")"
    echo "$loaded"
    image="$(printf '%s\n' "$loaded" | sed -n 's/^Loaded image: //p' | tail -n 1)"
    if [[ -z "$image" ]]; then
      echo "The archive did not report a loaded image name." >&2
      exit 1
    fi
  elif [[ -f "$upload_dir/Dockerfile" ]]; then
    image="$container_name:upload"
    echo "Building $image from containers/upload/Dockerfile"
    "$runtime" build -t "$image" "$upload_dir"
  else
    echo "Put one docker save archive or a Dockerfile in containers/upload, or pass --image." >&2
    exit 1
  fi
fi

echo "Code: $(git -C "$repo_dir" rev-parse HEAD)"
sha256sum "$profile"
"$runtime" image inspect "$image" --format '{{.Id}}'
if [[ "$mode" == preflight ]]; then
  echo "Preflight complete. No robot processes or container started."
  exit 0
fi
if [[ "$mode" == hardware ]] && systemctl --user is-active --quiet g1-safety-stops.service; then
  echo "With the robot supported, deliberately stop g1-safety-stops before selecting this gate." >&2
  exit 2
fi
thrash="${G1_THRASH_MODE:-monitor}"
if [[ -n "${G1_TRIAL_CONFIG:-}" && "$thrash" != enforce ]]; then
  echo "Scored trials require G1_THRASH_MODE=enforce." >&2; exit 2
fi
set +u
source /opt/ros/foxy/setup.bash
source "${HIM_SAFETY_ROS_INSTALL:-$HOME/.him-safety/ros-install}/setup.bash"
set -u
export PYTHONPATH="$repo_dir:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${G1_POLICY_ROS_DOMAIN:-98}" ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
log_dir="${G1_RUN_LOG:-$repo_dir/.cache/hardware_runs/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$log_dir"
{ git -C "$repo_dir" rev-parse HEAD; sha256sum "$profile"; "$runtime" image inspect "$image" --format '{{.Id}}'; } > "$log_dir/configuration.txt"
cp "$profile" "$log_dir/joint_profile.json"
pids=()
cleanup() {
  trap - EXIT INT TERM
  stop_container
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
launch_node() {
  local role="$1"
  shift
  /usr/bin/python3 "$repo_dir/ros2/him_safety_nodes/scripts/safety_node" "$role" --ros-args "$@" > "$log_dir/$role.log" 2>&1 &
  pids+=("$!")
}
launch_node checker -p "thrash.mode:=$thrash"
launch_node controller_stop -p startup_timeout_secs:=5.0
launch_node feedback -p "sdk_interface:=${ROBOT_INTERFACE:-eth0}" -p "sdk_domain:=${ROBOT_DOMAIN:-0}" \
  -p publish_policy_state:=true -p "require_base_state:=${G1_REQUIRE_BASE_STATE:-true}" \
  -p "base_state_topic:=${G1_BASE_STATE_TOPIC:-}" -p "base_velocity_frame:=${G1_BASE_VELOCITY_FRAME:-}"
dry_run=false
[[ "$mode" != dry-run ]] || dry_run=true
launch_node hardware_gate -p "enable_hardware_output:=$enabled" -p "dry_run:=$dry_run" \
  -p whole_body_output:=true -p "hardware_profile_path:=$profile" \
  -p "sdk_interface:=${ROBOT_INTERFACE:-eth0}" -p "sdk_domain:=${ROBOT_DOMAIN:-0}"
stop_container
echo "Starting $image as $container_name on the robot network (interface ${ROBOT_INTERFACE:-eth0}, DDS domain ${ROBOT_DOMAIN:-0})."
echo "Mode: $mode. Logs: $log_dir. Ctrl+C invokes the selected gate's stop path."

run_args=(
  --name "$container_name"
  --network host
  -e "ROBOT_INTERFACE=${ROBOT_INTERFACE:-eth0}"
  -e "ROBOT_DOMAIN=${ROBOT_DOMAIN:-0}"
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-97}"
  -e "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}"
  -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  -e "G1_EXECUTION_MODE=$mode"
  -e "G1_THRASH_MODE=$thrash"
)
if [[ -n "${G1_TRIAL_CONFIG:-}" ]]; then
  trial_config="$(realpath "$G1_TRIAL_CONFIG")"
  cp "$trial_config" "$log_dir/trial.json"
  run_args+=(-v "$trial_config:/trial.json:ro" -e G1_TRIAL_CONFIG=/trial.json)
fi
if [[ "${G1_NVIDIA:-0}" == 1 ]]; then
  run_args+=(--runtime nvidia)
fi
if [[ "$detach" -eq 1 ]]; then
  run_args+=(-d)
else
  run_args+=(--rm)
fi

"$runtime" run "${run_args[@]}" "$image" "${extra[@]}" 2>&1 | tee "$log_dir/policy.jsonl"
