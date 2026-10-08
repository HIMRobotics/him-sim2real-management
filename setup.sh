#!/usr/bin/env bash
# Load the container dropped in containers/upload and run it on the robot network.
# The image's own entrypoint decides what to execute. This script does not
# interpret models. Host nodes own checked motor output.
set -euo pipefail
trap 'launch_status=$?; echo "setup.sh failed at line $LINENO (exit $launch_status)." >&2' ERR

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
upload_dir="$repo_dir/containers/upload"
container_name=g1-user-container
runtime="${CONTAINER_RUNTIME:-}"
original_args=("$@")

show_help() {
  cat <<'HELP'
Usage: ./setup.sh [--mode preflight|dry-run|hardware] [--enable-hardware-output]
                  [--image NAME | --example NAME] [--command FILE] [--profile FILE]
                  [--] [container args...]
       ./setup.sh stop

Canonical hardware supervisor. Prefer ./submit.sh --mode real for deployment.
Hardware always uses g1-safety-stops.service. Startup runs GR00T Balance
until measured stability, then waits for left + B before policy handoff.
Double Start still requests damping. Preflight never starts robot processes.
HELP
}

stop_container() {
  if "$runtime" ps -a --format '{{.Names}}' | grep -qx "$container_name"; then
    "$runtime" rm -f "$container_name" >/dev/null
  fi
}

stopping=0
if [[ "${1:-}" == stop ]]; then
  stopping=1
  shift
fi

detach=0
mode=preflight
enabled=false
profile="${G1_JOINT_PROFILE:-}"
image="${G1_IMAGE:-}"
example=""
command_file="${G1_COMMAND_FILE:-}"
python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
[[ -n "${G1_SIM_PYTHON:-}" || -x "$python" ]] || python="${PYTHON:-python3}"
extra=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) show_help; exit 0 ;;
    --detach) detach=1; shift ;;
    --example) example="${2:?--example needs a name}"; shift 2 ;;
    --command) command_file="${2:?--command needs a YAML file}"; shift 2 ;;
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
if [[ -n "$example" && -n "$image" ]]; then
  echo "Use --example or --image, not both." >&2
  exit 2
fi
if [[ -n "$command_file" && ! -f "$command_file" ]]; then
  echo "Command file is not readable: $command_file" >&2
  exit 2
fi
if [[ -n "$example" ]]; then
  case "$example" in walker|stairs|balance) ;; *) echo "Unknown hardware example: $example" >&2; exit 2 ;; esac
fi
if [[ -z "$runtime" ]]; then
  if command -v docker >/dev/null 2>&1; then runtime=docker
  elif command -v podman >/dev/null 2>&1; then runtime=podman
  else echo "Install Docker or Podman." >&2; exit 1
  fi
fi
if [[ "$runtime" == docker && "${EUID}" -ne 0 && ! -w /var/run/docker.sock ]]; then
  exec sudo --preserve-env=HOME,G1_SIM_PYTHON,G1_JOINT_PROFILE,G1_COMMAND_FILE,G1_IMAGE,G1_STANCE_BLEND_SECS,G1_SIM_EXAMPLE,G1_THRASH_MODE,G1_TRIAL_CONFIG,G1_TASK_COMMAND,G1_REQUIRE_BASE_STATE,G1_BASE_STATE_TOPIC,G1_BASE_VELOCITY_FRAME,G1_NVIDIA,G1_RUN_LOG,HIM_SAFETY_ROS_INSTALL,ROBOT_INTERFACE,ROBOT_DOMAIN,PYTHON,CONTAINER_RUNTIME -- "$0" "${original_args[@]}"
fi
if [[ "$stopping" -eq 1 ]]; then
  stop_container
  echo "Stopped $container_name."
  exit 0
fi
if [[ -n "$example" ]]; then
  prepared="$("$python" "$repo_dir/containers/examples/prepare.py" "$example")"
  image="g1-example-${example}:local"
  "$runtime" build -t "$image" "$prepared/container"
  [[ -n "$profile" ]] || profile="$prepared/container/project/joint_profile.json"
  export G1_SIM_EXAMPLE="$example"
fi
profile="${profile:-$repo_dir/robot_safety/profiles/g1_type5.template.json}"
if [[ -n "$command_file" ]]; then
  export G1_COMMAND_FILE="$(realpath "$command_file")"
fi
# Every real entry point selects the same long-lived motor writer.
if [[ "$mode" == hardware ]]; then
  export G1_USE_SAFETY_SERVICE=1 G1_POLICY_ROS_DOMAIN=97
else
  export G1_USE_SAFETY_SERVICE=0
fi
if [[ -z "${G1_BASE_STATE_TOPIC:-}" ]]; then
  export G1_REQUIRE_BASE_STATE=false
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

if [[ "$mode" == hardware ]]; then
  "$runtime" image inspect "$image" --format '{{.Id}}' >/dev/null
else
  echo "Code: $(git -C "$repo_dir" rev-parse HEAD)"
  sha256sum "$profile"
  "$runtime" image inspect "$image" --format '{{.Id}}'
fi
if [[ "$mode" == preflight ]]; then
  echo "Preflight complete. No robot processes or container started."
  exit 0
fi
safety_systemctl() {
  local service_user uid
  service_user="${SUDO_USER:-${USER:-}}"
  if [[ "$EUID" -ne 0 || -z "$service_user" || "$service_user" == root ]]; then
    systemctl --user "$@"
    return
  fi
  uid="$(id -u "$service_user")"
  runuser -u "$service_user" -- env "XDG_RUNTIME_DIR=/run/user/$uid" \
    systemctl --user "$@"
}
safety_service_active() {
  safety_systemctl is-active --quiet g1-safety-stops.service
}
wait_for_safety_service() {
  local _attempt
  for _attempt in $(seq 1 180); do
    safety_service_active && return 0
    sleep 1
  done
  echo "g1-safety-stops.service did not become active." >&2
  return 1
}
if [[ "$mode" == hardware && "${G1_USE_SAFETY_SERVICE:-}" == 1 ]]; then
  export ROS_DOMAIN_ID=97
  if ! safety_service_active; then
    echo "Starting g1-safety-stops.service. It stays up and is the motor stop."
    safety_systemctl start g1-safety-stops.service
    wait_for_safety_service
  fi
fi
default_thrash=monitor
[[ "$mode" != hardware ]] || default_thrash=enforce
thrash="${G1_THRASH_MODE:-$default_thrash}"
if [[ -n "${G1_TRIAL_CONFIG:-}" && "$thrash" != enforce ]]; then
  echo "Scored trials require G1_THRASH_MODE=enforce." >&2; exit 2
fi
set +u
source /opt/ros/foxy/setup.bash
operator_home="$HOME"
if [[ -n "${SUDO_USER:-}" ]]; then
  operator_home="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
fi
source "${HIM_SAFETY_ROS_INSTALL:-$operator_home/.him-safety/ros-install}/setup.bash"
set -u
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/unitree/cyclonedds_ws/install/cyclonedds}"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$repo_dir:${PYTHONPATH:-}"
if [[ "${G1_USE_SAFETY_SERVICE:-}" == 1 ]]; then
  export G1_POLICY_ROS_DOMAIN=97
fi
export ROS_DOMAIN_ID="${G1_POLICY_ROS_DOMAIN:-98}" ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
log_dir="${G1_RUN_LOG:-$repo_dir/.cache/hardware_runs/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$log_dir"
{ git -C "$repo_dir" rev-parse HEAD; sha256sum "$profile"; "$runtime" image inspect "$image" --format '{{.Id}}'; } > "$log_dir/configuration.txt"
echo "Thrashing detection: $thrash" >> "$log_dir/configuration.txt"
cp "$profile" "$log_dir/joint_profile.json"
pids=()
stop_session_nodes() {
  local node_user pid args
  node_user="${host_user:-${SUDO_USER:-${USER:-}}}"
  [[ -z "$node_user" || "$node_user" == root ]] && node_user="$(id -un)"
  # ps truncates args to the terminal width unless -ww is set, which hid these.
  while read -r pid args; do
    case "$args" in
      *safety_node\ checker*|*safety_node\ controller_stop*|*safety_node\ feedback*)
        kill -KILL "$pid" 2>/dev/null || true
        ;;
    esac
  done < <(ps -ww -u "$node_user" -o pid=,args=)
}
cleanup() {
  trap - EXIT INT TERM
  stop_container || true
  stop_session_nodes
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
host_user="${SUDO_USER:-${USER:-}}"
[[ "$host_user" == root ]] && host_user=""
as_host_user() {
  if [[ -n "$host_user" && "$(id -un)" != "$host_user" ]]; then
    local host_home uid
    uid="$(id -u "$host_user")"
    host_home="$(getent passwd "$host_user" | cut -d: -f6)"
    runuser --preserve-environment -u "$host_user" -- \
      env "HOME=$host_home" "XDG_RUNTIME_DIR=/run/user/$uid" "$@"
  else
    "$@"
  fi
}
launch_node() {
  local role="$1"
  shift
  as_host_user /usr/bin/python3 "$repo_dir/ros2/him_safety_nodes/scripts/safety_node" "$role" --ros-args "$@" > "$log_dir/$role.log" 2>&1 &
  pids+=("$!")
}
read_gate_status() {
  as_host_user timeout 5 /usr/bin/python3 -m robot_safety.read_hardware_status || true
}

gate_is_latched() {
  [[ -n "${1:-}" ]] || return 1
  python3 -c '
import json, sys
gate = json.loads(sys.argv[1])
raise SystemExit(0 if gate.get("abort_reason") else 1)
' "$1"
}
# If live status is unavailable, show the previous stop from syslog.
# Use grep because sudo's PATH may not contain rg.
syslog_abort_reason() {
  local main_pid child
  main_pid="$(safety_systemctl show -p MainPID --value g1-safety-stops.service 2>/dev/null || true)"
  [[ -n "$main_pid" && "$main_pid" != 0 ]] || return 0
  child="$(ps --ppid "$main_pid" -o pid= 2>/dev/null | awk '{print $1; exit}')"
  [[ -n "$child" ]] || child="$main_pid"
  grep -a "run_safety_stops.sh\\[$child\\]" /var/log/syslog 2>/dev/null \
    | grep -o '"abort_reason": [^,}]*' \
    | tail -1 \
    | python3 -c '
import sys
line = sys.stdin.read().strip()
value = line.split(":", 1)[-1].strip() if line else ""
if len(value) >= 2 and value[0] == value[-1] == "\"":
    value = value[1:-1]
if value and value != "null":
    print(value)
'
}
stop_session_nodes
for _attempt in 1 2 3 4 5; do
  if ! ps -ww -u "${host_user:-${SUDO_USER:-unitree}}" -o args= | grep -E 'safety_node (checker|controller_stop|feedback)' >/dev/null; then
    break
  fi
  stop_session_nodes
  sleep 0.2
done
dry_run=false
[[ "$mode" != dry-run ]] || dry_run=true
if [[ "${G1_USE_SAFETY_SERVICE:-}" == 1 ]]; then
  # Restart a latched watchdog before any session node subscribes. A checker
  # that hears the old abort republishes it, and the fresh gate latches it too.
  wanted_profile="$(realpath "$profile")"
  safety_systemctl set-environment "HIM_SAFETY_PROFILE=$wanted_profile" "G1_STANCE_BLEND_SECS=${G1_STANCE_BLEND_SECS:-5}"
  current_gate="$(read_gate_status)"
  needs_reload=0
  latched_reason=""
  if [[ -n "$current_gate" ]]; then
    latched_reason="$(python3 -c '
import json, sys
print(json.loads(sys.argv[1]).get("abort_reason") or "")
' "$current_gate")"
    needs_reload="$(python3 -c '
import json, sys
gate = json.loads(sys.argv[1])
same = (
    gate.get("startup_balance_required") is True
    and gate.get("whole_body_output") is True
    and gate.get("launch_gate") is True
    and gate.get("hardware_profile_path") == sys.argv[2]
    and not gate.get("abort_reason")
)
print("0" if same else "1")
' "$current_gate" "$wanted_profile")"
  else
    latched_reason="$(syslog_abort_reason || true)"
    needs_reload=1
  fi
  if [[ "$needs_reload" == 0 ]]; then
    service_pid="$(safety_systemctl show -p MainPID --value g1-safety-stops.service || true)"
    if [[ -n "$service_pid" && "$service_pid" != 0 && (
      "$repo_dir/robot_safety/startup_balance.py" -nt "/proc/$service_pid" ||
      "$repo_dir/robot_safety/hardware_adapter.py" -nt "/proc/$service_pid" ||
      "$repo_dir/robot_safety/hardware_node.py" -nt "/proc/$service_pid" ||
      "$repo_dir/robot_safety/ros_nodes.py" -nt "/proc/$service_pid"
    ) ]]; then
      needs_reload=1
    fi
  fi
  if [[ "$needs_reload" == 1 ]]; then
    echo "Reloading safety service. Keep the robot supported: this can request damping." >&2
    echo "Previous stop: $latched_reason" >> "$log_dir/configuration.txt"
    safety_systemctl restart g1-safety-stops.service
    wait_for_safety_service
    current_gate=""
    for _attempt in 1 2 3 4 5; do
      current_gate="$(read_gate_status)"
      [[ -n "$current_gate" ]] && break
    done
  fi
  if [[ -z "$current_gate" ]] || ! python3 -c '
import json, sys
gate = json.loads(sys.argv[1])
raise SystemExit(0 if (
    gate.get("startup_balance_required") is True
    and gate.get("whole_body_output") is True
    and gate.get("launch_gate") is True
    and gate.get("hardware_profile_path") == sys.argv[2]
) else 1)
' "$current_gate" "$wanted_profile"; then
    echo "Safety service startup gate and profile could not be verified. No policy session started." >&2
    exit 2
  fi
  if gate_is_latched "$current_gate"; then
    echo "g1-safety-stops is still latched (${current_gate}). No policy session started." >&2
    exit 2
  fi
fi
launch_node checker -p "thrash.mode:=$thrash"
launch_node controller_stop -p startup_timeout_secs:=5.0
feedback_args=(
  -p "sdk_interface:=${ROBOT_INTERFACE:-eth0}" -p "sdk_domain:=${ROBOT_DOMAIN:-0}"
  -p "hardware_profile_path:=$profile"
  -p publish_policy_state:=true -p "require_base_state:=${G1_REQUIRE_BASE_STATE:-true}"
)
if [[ -n "${G1_BASE_STATE_TOPIC:-}" ]]; then
  feedback_args+=(
    -p "base_state_topic:=${G1_BASE_STATE_TOPIC}"
    -p "base_velocity_frame:=${G1_BASE_VELOCITY_FRAME:?G1_BASE_VELOCITY_FRAME must be world or body}"
  )
fi
launch_node feedback "${feedback_args[@]}"
if [[ "${G1_USE_SAFETY_SERVICE:-}" != 1 ]]; then
  launch_node hardware_gate -p "enable_hardware_output:=$enabled" -p "dry_run:=$dry_run" \
    -p whole_body_output:=true -p "hardware_profile_path:=$profile" \
    -p "launch_gate:=${G1_LAUNCH_GATE:-false}" \
    -p "stance_blend_secs:=${G1_STANCE_BLEND_SECS:-5}" \
    -p "sdk_interface:=${ROBOT_INTERFACE:-eth0}" -p "sdk_domain:=${ROBOT_DOMAIN:-0}"
fi
stop_container
if [[ "$mode" != hardware ]]; then
  echo "Starting $image as $container_name. Mode: $mode. Logs: $log_dir."
fi

run_args=(
  --name "$container_name"
  --network host
  --ipc=host
)
if [[ -n "$host_user" ]]; then
  run_args+=(--user "$(id -u "$host_user"):$(id -g "$host_user")" -e HOME=/tmp)
fi
run_args+=(
  -e "ROBOT_INTERFACE=${ROBOT_INTERFACE:-eth0}"
  -e "ROBOT_DOMAIN=${ROBOT_DOMAIN:-0}"
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-97}"
  -e "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}"
  -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  -e "G1_EXECUTION_MODE=$mode"
  -e "G1_THRASH_MODE=$thrash"
  -e "G1_SIM_EXAMPLE=${G1_SIM_EXAMPLE:-}"
  -e "G1_TASK_COMMAND=${G1_TASK_COMMAND:-}"
)
if [[ -n "${G1_COMMAND_FILE:-}" && -f "${G1_COMMAND_FILE}" ]]; then
  command_file="$(realpath "$G1_COMMAND_FILE")"
  cp "$command_file" "$log_dir/command.yaml"
  run_args+=(-v "$command_file:/command.yaml:ro" -e G1_COMMAND_FILE=/command.yaml)
fi
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

if [[ "$mode" == hardware ]]; then
  "$runtime" run "${run_args[@]}" "$image" "${extra[@]}" 2>&1 | tee "$log_dir/policy.jsonl" | python3 -u -m robot_safety.console_output
else
  "$runtime" run "${run_args[@]}" "$image" "${extra[@]}" 2>&1 | tee "$log_dir/policy.jsonl"
fi
