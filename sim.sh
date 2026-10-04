#!/usr/bin/env bash
# Run containers/upload against a MuJoCo G1. The container shares a loopback-only
# network namespace with the sim bridge. It cannot see eth0, so it cannot command
# the real robot. g1-safety-stops is left running.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
upload_dir="$repo_dir/containers/upload"
out_dir="$repo_dir/containers/sim/output"
ros_dir="$repo_dir/containers/sim/.ros"
net_name=g1-sim-net
user_name=g1-sim-user
runtime="${CONTAINER_RUNTIME:-docker}"
original_args=("$@")

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./sim.sh [--image NAME] [--scene PATH] [--seconds N]
                [--safety-checks | --no-safety-checks]
                [--pose NAME | --counterfactuals] [--self-test] [--circle]
                [--command FILE] [--view]
                [--battery] [--push none|small|medium|heavy]
                [--example stairs|walker|balance] [--local] [--output DIR]
       ./sim.sh replay [TRACE]
       ./sim.sh stop

On macOS or Windows use ./desktop_sim.sh (same flags). This script's
ROS path needs Linux network namespaces.

--scene PATH        Load a host-side MuJoCo XML scene and its relative assets.
                    Default: the bundled G1 on a flat floor.
--safety-checks     Enable safety checks (default).
--no-safety-checks  Disable safety limits, freshness/timeouts, and fall checks.
                    Message format and finite-value validation still apply.
--example NAME     Run a bundled ONNX controller in its matching environment.
--local            Run an example directly in MuJoCo without Docker/ROS/sudo.
--output DIR       Store this run separately for comparison and replay.
--circle           Walk with the cached G1 walker. Command file defaults to
                   examples/commands/walker/circle.yaml. No submission.
--command FILE     Walker command file. examples/commands/walker/stand.yaml stays put.
                   examples/commands/walker/circle.yaml is the loop. Same runner as --circle.
--battery          Flat-ground battery: every pose, then 10/50/150 N pelvis
                   pushes. Writes containers/sim/output/battery/report.json.
                   With --circle, the cached walker gaits instead of a hold.
--push NAME        One battery case: none, small, medium, or heavy.
                   With --view, open the page and press Start to watch it.
--view             Open http://127.0.0.1:8765 first. Press Start to run the
                   policy and watch the render live. Ctrl+C closes the page.
HELP
  exit 0
fi

if [[ "${1:-}" == "replay" ]]; then
  trace="${2:-$out_dir/trace.npz}"
  python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
  if [[ ! -f "$trace" ]]; then
    echo "Trace not found: $trace" >&2
    exit 2
  fi
  "$python" "$repo_dir/containers/sim/view.py" "$trace" "$out_dir/view"
  port="$("$python" - <<'PY'
import socket
for port in range(8765, 8815):
    sock = socket.socket()
    try:
        sock.bind(("127.0.0.1", port))
    except OSError:
        continue
    else:
        print(port)
        break
    finally:
        sock.close()
else:
    raise SystemExit("no free viewer port")
PY
)"
  echo "Open http://127.0.0.1:$port"
  echo "Left drag orbits, right drag pans, scroll zooms. Space pauses. Ctrl+C stops the viewer."
  exec "$python" -m http.server --bind 127.0.0.1 "$port" --directory "$out_dir/view"
fi

owner="${SUDO_USER:-$USER}"
seconds="${G1_SIM_SECONDS:-10}"
seconds_set=0
self_test=0
circle=0
command_file=""
view=0
battery=0
push=""
counterfactuals=0
pose="standing"
pose_set=0
image="${G1_IMAGE:-}"
scene=""
safety_checks=1
example=""
local_mode=0
stopping=0
# Keep this list in step with containers/sim/plant.py POSES.
poses=(standing seated supine prone left_side right_side)

kill_tree() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  local child
  for child in $(ps -o pid= --ppid "$pid" 2>/dev/null); do
    kill_tree "$child"
  done
  kill "$pid" >/dev/null 2>&1 || true
}

stop_sim() {
  if [[ -f "$out_dir/bridge.pid" ]]; then
    kill_tree "$(cat "$out_dir/bridge.pid")"
    rm -f "$out_dir/bridge.pid"
  fi
  if [[ -f "$out_dir/hold.pid" ]]; then
    kill_tree "$(cat "$out_dir/hold.pid")"
    rm -f "$out_dir/hold.pid"
  fi
  "$runtime" rm -f "$user_name" "$net_name" >/dev/null 2>&1 || true
}

live_pid=""
stop_view() {
  if [[ -n "${live_pid:-}" ]]; then
    kill_tree "$live_pid"
    wait "$live_pid" 2>/dev/null || true
    live_pid=""
  fi
}

start_live_view() {
  mkdir -p "$out_dir"
  chown "$owner" "$out_dir" 2>/dev/null || true
  rm -f "$out_dir/view/start" "$out_dir/live.jsonl"
  local python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
  local scene_args=()
  [[ -n "$scene" ]] && scene_args=(--scene "$scene")
  as_owner "$python" "$repo_dir/containers/sim/view.py" --idle "$out_dir/view" --pose "$pose" "${scene_args[@]}"
  rm -f "$out_dir/live-view.log"
  # Open the log as the user. A root shell would otherwise leave a file
  # the battery viewer cannot replace.
  as_owner env "LOG=$out_dir/live-view.log" bash -c \
    'exec "$1" "${@:2}" >"$LOG" 2>&1' _ \
    "$python" "$repo_dir/containers/sim/live.py" \
    --directory "$out_dir/view" \
    --live "$out_dir/live.jsonl" \
    --start "$out_dir/view/start" \
    --report "$out_dir/report.json" \
    --pose "$pose" \
    "${scene_args[@]}" &
  live_pid=$!
  echo "Open http://127.0.0.1:8765"
  echo "Press Start to run the policy. The camera stays put so the path is visible."
}

wait_for_start() {
  echo "Waiting for Start in the browser."
  while [[ ! -f "$out_dir/view/start" ]]; do
    if ! kill -0 "$live_pid" 2>/dev/null; then
      echo "Viewer exited before Start." >&2
      cat "$out_dir/live-view.log" >&2 || true
      exit 1
    fi
    sleep 0.2
  done
}

if [[ "${1:-}" == "stop" ]]; then
  stopping=1
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --self-test) self_test=1; shift ;;
    --circle) circle=1; shift ;;
    --command) command_file="${2:?--command needs a yaml file}"; circle=1; shift 2 ;;
    --battery) battery=1; shift ;;
    --push) push="${2:?--push needs none, small, medium, or heavy}"; shift 2 ;;
    --view) view=1; shift ;;
    --counterfactuals) counterfactuals=1; shift ;;
    --pose) pose="${2:?--pose needs a name}"; pose_set=1; shift 2 ;;
    --seconds) seconds="${2:?--seconds needs a duration}"; seconds_set=1; shift 2 ;;
    --image) image="${2:?--image needs a name}"; shift 2 ;;
    --scene) scene="${2:?--scene needs an XML path}"; shift 2 ;;
    --safety-checks) safety_checks=1; shift ;;
    --no-safety-checks) safety_checks=0; shift ;;
    --example) example="${2:?--example needs stairs, walker, or balance}"; shift 2 ;;
    --local) local_mode=1; shift ;;
    --output) out_dir="${2:?--output needs a directory}"; shift 2 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ -n "$example" && "$example" != stairs && "$example" != walker && "$example" != balance ]]; then
  echo "--example must be stairs, walker, or balance" >&2
  exit 2
fi
if [[ -n "$example" && ( -n "$image" || "$self_test" -eq 1 || "$circle" -eq 1 ) ]]; then
  echo "Use --example by itself, without --image, --self-test, or --circle." >&2
  exit 2
fi
if [[ "$circle" -eq 1 && "$self_test" -eq 1 ]]; then
  echo "Use --circle by itself, without --self-test." >&2
  exit 2
fi
if [[ -n "$push" && "$battery" -eq 0 ]]; then
  echo "--push is a battery case. Add --battery." >&2
  exit 2
fi
if [[ "$battery" -eq 1 && "$view" -eq 1 && -z "$push" ]]; then
  echo "Pass --push none, small, medium, or heavy to watch one battery case." >&2
  exit 2
fi
if [[ "$battery" -eq 1 && ( "$self_test" -eq 1 || "$counterfactuals" -eq 1 || -n "$example" || -n "$image" ) ]]; then
  echo "--battery runs on the flat-ground plant. Drop --self-test, --counterfactuals, --example, and --image." >&2
  exit 2
fi
if [[ "$battery" -eq 1 && "$circle" -eq 1 && "$pose" != "standing" ]]; then
  echo "The circle battery starts from the walker stance. Drop --pose." >&2
  exit 2
fi
if [[ "$local_mode" -eq 1 && -z "$example" ]]; then
  echo "--local requires --example stairs, walker, or balance." >&2
  exit 2
fi
mkdir -p "$out_dir"
out_dir="$(cd -- "$out_dir" && pwd -P)"

if [[ "$stopping" -eq 1 ]]; then
  if [[ "${EUID}" -ne 0 ]]; then
    exec sudo --preserve-env=CONTAINER_RUNTIME -- "$0" "${original_args[@]}"
  fi
  stop_sim
  echo "Stopped the MuJoCo container sim."
  exit 0
fi

known_pose=0
for name in "${poses[@]}"; do
  [[ "$pose" == "$name" ]] && known_pose=1
done
if [[ "$known_pose" -ne 1 ]]; then
  echo "--pose must be one of: ${poses[*]}" >&2
  exit 2
fi
if [[ "$counterfactuals" -eq 1 && "$pose" != "standing" ]]; then
  echo "--counterfactuals runs every pose. Drop --pose." >&2
  exit 2
fi

if [[ "$seconds_set" -eq 0 && -n "${G1_SIM_SECONDS:-}" ]]; then
  seconds_set=1
fi
if [[ "$counterfactuals" -eq 1 && "$seconds_set" -eq 0 && "$self_test" -eq 0 ]]; then
  seconds=5
fi
if [[ "$self_test" -eq 1 && "$seconds_set" -eq 0 ]]; then
  seconds=1
fi
if [[ "$circle" -eq 1 && -z "$command_file" ]]; then
  command_file="$repo_dir/examples/commands/walker/circle.yaml"
fi
if [[ -n "$command_file" && ! -f "$command_file" ]]; then
  echo "Walker command file is not readable: $command_file" >&2
  exit 2
fi
if [[ "$battery" -eq 1 && "$seconds_set" -eq 0 ]]; then
  seconds=2
  [[ "$circle" -eq 1 ]] && seconds=10
fi
if ! [[ "$seconds" =~ ^[0-9]+([.][0-9]+)?$ ]] || ! awk "BEGIN { exit !($seconds > 0) }"; then
  echo "--seconds must be positive" >&2
  exit 2
fi

if [[ -n "$scene" ]]; then
  if [[ ! -f "$scene" || ! -r "$scene" ]]; then
    echo "MuJoCo scene is not a readable file: $scene" >&2
    exit 2
  fi
  scene="$(cd -- "$(dirname -- "$scene")" && pwd -P)/$(basename -- "$scene")"
fi

if [[ "$battery" -eq 1 ]]; then
  python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
  battery_args=(--output "$out_dir/battery" --seconds "$seconds")
  [[ "$safety_checks" -eq 0 ]] && battery_args+=(--no-safety-checks)
  if [[ -n "$push" || "$pose_set" -eq 1 ]]; then
    battery_args+=(--pose "$pose")
  fi
  [[ -n "$push" ]] && battery_args+=(--push "$push")
  [[ "$circle" -eq 1 ]] && battery_args+=(--circle)
  if [[ "$view" -eq 1 ]]; then
    mkdir -p "$out_dir/battery"
    rm -f "$out_dir/view/start" "$out_dir/live.jsonl" "$out_dir/battery/report.json" "$out_dir/live-view.log"
    "$python" "$repo_dir/containers/sim/view.py" --idle "$out_dir/view" --pose "$pose"
    "$python" "$repo_dir/containers/sim/live.py" \
      --directory "$out_dir/view" \
      --live "$out_dir/live.jsonl" \
      --start "$out_dir/view/start" \
      --report "$out_dir/battery/report.json" \
      --pose "$pose" \
      >"$out_dir/live-view.log" 2>&1 &
    live_pid=$!
    trap 'kill_tree "$live_pid"' EXIT INT TERM
    echo "Open http://127.0.0.1:8765"
    echo "Press Start to run ${pose} / ${push}."
    wait_for_start
    G1_SIM_LIVE="$out_dir/live.jsonl" G1_SIM_COMMAND="$command_file" "$python" "$repo_dir/containers/sim/battery.py" "${battery_args[@]}"
    status=$?
    echo "Report: $out_dir/battery/report.json"
    echo "Viewer is still at http://127.0.0.1:8765"
    echo "Left drag orbits, right drag pans, scroll zooms. Space pauses. Ctrl+C closes the page."
    wait "$live_pid" || true
    exit "$status"
  fi
  G1_SIM_COMMAND="$command_file" exec "$python" "$repo_dir/containers/sim/battery.py" "${battery_args[@]}"
fi

if [[ "$local_mode" -eq 1 ]]; then
  python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
  local_args=(--example "$example" --seconds "$seconds")
  [[ "$safety_checks" -eq 0 ]] && local_args+=(--no-safety-checks)
  [[ -n "$scene" ]] && local_args+=(--scene "$scene")
  if [[ "$counterfactuals" -eq 1 ]]; then
    for pose in "${poses[@]}"; do
      "$python" "$repo_dir/containers/sim/local.py" "${local_args[@]}" --pose "$pose" --output "$out_dir/poses/$pose" || true
    done
    exec "$python" "$repo_dir/containers/sim/summary.py" "$out_dir/poses" "$safety_checks" "$scene" "${poses[@]}"
  fi
  status=0
  "$python" "$repo_dir/containers/sim/local.py" "${local_args[@]}" --pose "$pose" --output "$out_dir" || status=$?
  echo "Replay: ./sim.sh replay $out_dir/trace.npz"
  if [[ "$view" -eq 1 && -f "$out_dir/trace.npz" ]]; then
    "$python" "$repo_dir/containers/sim/view.py" "$out_dir/trace.npz" "$out_dir/view"
    echo "Open http://127.0.0.1:8765"
    "$python" -m http.server --bind 127.0.0.1 8765 --directory "$out_dir/view"
  fi
  exit "$status"
fi

if [[ "$(uname -s)" != Linux ]]; then
  echo "The Linux network-namespace sim does not run on $(uname -s)." >&2
  echo "Use ./desktop_sim.sh with the same flags. --local, --self-test, --circle, --battery, and replay need no Docker." >&2
  exit 2
fi
if [[ "${EUID}" -ne 0 ]]; then
  echo "Sim needs root to enter the isolated network. Re-running with sudo."
  exec sudo --preserve-env=G1_SIM_SECONDS,G1_SIM_PYTHON,CONTAINER_RUNTIME,G1_IMAGE,HIM_BALANCE_ONNX,DISPLAY,XAUTHORITY -- "$0" "${original_args[@]}"
fi

as_owner() {
  if [[ "${EUID}" -eq 0 && "$owner" != root ]]; then
    runuser -u "$owner" -- "$@"
  else
    "$@"
  fi
}

in_netns() {
  local pid="$1"
  shift
  # -n puts the bridge on the sim container's loopback. Discovery is unicast
  # to 127.0.0.1 there (containers/sim/fastdds.xml), not host shared memory.
  nsenter -t "$pid" -n -- runuser -u "$owner" -- "$@"
}

load_image() {
  if [[ -n "$image" ]]; then
    return
  fi
  shopt -s nullglob
  archives=("$upload_dir"/*.tar "$upload_dir"/*.tar.gz)
  shopt -u nullglob
  if [[ ${#archives[@]} -ne 1 ]]; then
    echo "Put one image archive in containers/upload, pass --image, or use --self-test." >&2
    exit 1
  fi
  echo "Loading ${archives[0]}"
  loaded="$("$runtime" load -i "${archives[0]}")"
  echo "$loaded"
  image="$(printf '%s\n' "$loaded" | sed -n 's/^Loaded image: //p' | tail -n 1)"
  if [[ -z "$image" ]]; then
    echo "The archive did not report a loaded image name." >&2
    exit 1
  fi
}

running="$("$runtime" ps --format '{{.Names}}')"
if grep -qx g1-user-container <<<"$running"; then
  echo "The robot container is running. ./setup.sh stop it before starting the sim." >&2
  exit 1
fi

if [[ ! -f "$ros_dir/install/setup.bash" ]]; then
  echo "Building him_safety_interfaces."
  as_owner bash -c "source /opt/ros/foxy/setup.bash && cd '$repo_dir' && colcon build --packages-select him_safety_interfaces --base-paths ros2 --build-base '$ros_dir/build' --install-base '$ros_dir/install'"
fi

if [[ -n "$example" ]]; then
  python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
  prepared="$(as_owner env "HIM_BALANCE_ONNX=${HIM_BALANCE_ONNX:-$repo_dir/examples/policies/balance/GR00T-WholeBodyControl-Balance.onnx}" "$python" "$repo_dir/containers/examples/prepare.py" "$example")"
  [[ -n "$scene" ]] || scene="$prepared/environment/scene.xml"
  image="g1-sim-example-$example:local"
  "$runtime" build -t "$image" "$prepared/container"
elif [[ "$self_test" -eq 0 && "$circle" -eq 0 ]]; then
  load_image
fi

base_out="$out_dir"
trap 'stop_sim; stop_view' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ "$view" -eq 1 && "$counterfactuals" -eq 0 ]]; then
  start_live_view
fi
run_session() {
  stop_sim
  mkdir -p "$out_dir"
  rm -f "$out_dir/ready" "$out_dir/report.json" "$out_dir/trace.npz" "$out_dir/live.jsonl"
  chown "$owner" "$out_dir" 2>/dev/null || true

  echo "Starting $pose (isolated network, loopback only, no eth0)."
  echo "Scene: ${scene:-bundled G1 on a flat floor}. Safety checks: $safety_checks."
  "$runtime" run -d --name "$net_name" --network none --ipc shareable ros:foxy-ros-base sleep infinity >/dev/null || return 2
  net_pid="$("$runtime" inspect -f '{{.State.Pid}}' "$net_name")" || return 2
  if nsenter -t "$net_pid" -n -- ip link show eth0 >/dev/null 2>&1; then
    echo "Isolated network still has eth0. Not starting the sim." >&2
    stop_sim
    return 1
  fi

  sim_env=(
    "G1_SIM_NETNS=1"
    "G1_SIM_POSE=$pose"
    "G1_SIM_SCENE=$scene"
    "G1_SIM_SAFETY_CHECKS=$safety_checks"
    "G1_SIM_EXAMPLE=$example"
    "G1_SIM_SECONDS=$seconds"
    "G1_SIM_REPORT=$out_dir/report.json"
    "G1_SIM_READY=$out_dir/ready"
    "G1_SIM_PYTHON=${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
    "ROBOT_INTERFACE=lo"
    "ROBOT_DOMAIN=0"
    "ROS_DOMAIN_ID=97"
    "ROS_LOCALHOST_ONLY=0"
    "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    "FASTRTPS_DEFAULT_PROFILES_FILE=$repo_dir/containers/sim/fastdds.xml"
    "G1_SIM_STARTUP=30"
    "G1_SIM_TRACE=$out_dir/trace.npz"
  )
  if [[ "$view" -eq 1 ]]; then
    sim_env+=("G1_SIM_LIVE=$out_dir/live.jsonl")
  fi
  if [[ "$circle" -eq 1 ]]; then
    sim_env+=("G1_SIM_CIRCLE=1" "G1_SIM_EXAMPLE=circle" "G1_SIM_COMMAND=$command_file")
  fi

  # Foxy's setup hook forces ROS_LOCALHOST_ONLY=0 when it is already set.
  # Keep it 0 on purpose: the value 1 enables a discovery path that cannot
  # meet the submission. The XML profile is the discovery config.
  bridge_cmd="cd '$repo_dir' && source /opt/ros/foxy/setup.bash && source '$ros_dir/install/setup.bash' && export ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=97 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE='$repo_dir/containers/sim/fastdds.xml' && exec python3 containers/sim/bridge.py"
  in_netns "$net_pid" env "${sim_env[@]}" bash -c "$bridge_cmd" >"$out_dir/bridge.log" 2>&1 &
  echo $! >"$out_dir/bridge.pid"

  for _ in $(seq 1 100); do
    [[ -f "$out_dir/ready" ]] && break
    if ! kill -0 "$(cat "$out_dir/bridge.pid")" 2>/dev/null; then
      echo "Sim bridge exited before it was ready." >&2
      cat "$out_dir/bridge.log" >&2 || true
      stop_sim
      return 1
    fi
    sleep 0.1
  done
  if [[ ! -f "$out_dir/ready" ]]; then
    echo "Sim bridge did not become ready." >&2
    cat "$out_dir/bridge.log" >&2 || true
    stop_sim
    return 1
  fi

  if [[ "$circle" -eq 1 ]]; then
    : # The bridge drives the walker. No submission container.
  elif [[ "$self_test" -eq 1 ]]; then
    hold_cmd="cd '$repo_dir' && source /opt/ros/foxy/setup.bash && source '$ros_dir/install/setup.bash' && export ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=97 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE='$repo_dir/containers/sim/fastdds.xml' && exec python3 containers/sim/hold.py"
    in_netns "$net_pid" env "${sim_env[@]}" bash -c "$hold_cmd" >"$out_dir/hold.log" 2>&1 &
    echo $! >"$out_dir/hold.pid"
  else
    "$runtime" run -d --name "$user_name" --network "container:$net_name" --ipc "container:$net_name" \
      -v "$repo_dir/containers/sim/fastdds.xml:/fastdds.xml:ro" \
      -e ROBOT_INTERFACE=lo \
      -e G1_SIM_NETNS=1 \
      -e "G1_SIM_EXAMPLE=$example" \
      -e ROBOT_DOMAIN=0 \
      -e ROS_DOMAIN_ID=97 \
      -e ROS_LOCALHOST_ONLY=0 \
      -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml \
      "$image" >/dev/null || { stop_sim; return 2; }
  fi

  set +e
  wait "$(cat "$out_dir/bridge.pid")"
  status=$?
  set -e
  if [[ -f "$out_dir/hold.pid" ]]; then
    kill_tree "$(cat "$out_dir/hold.pid")"
  fi
  "$runtime" logs "$user_name" >"$out_dir/policy.log" 2>&1 || true
  "$runtime" rm -f "$user_name" "$net_name" >/dev/null 2>&1 || true
  rm -f "$out_dir/bridge.pid" "$out_dir/hold.pid"

  if [[ -f "$out_dir/report.json" ]]; then
    echo "Report: $out_dir/report.json"
    cat "$out_dir/report.json"
    if [[ -f "$out_dir/trace.npz" ]]; then
      echo "Replay: ./sim.sh replay $out_dir/trace.npz"
    fi
  else
    echo "Sim bridge produced no report." >&2
    cat "$out_dir/bridge.log" >&2 || true
    status=2
  fi
  return "$status"
}

if [[ "$counterfactuals" -eq 0 ]]; then
  if [[ "$view" -eq 1 ]]; then
    wait_for_start
  fi
  status=0
  run_session || status=$?
  stop_sim
  if [[ "$view" -eq 1 && -n "$live_pid" ]]; then
    echo "Viewer is still at http://127.0.0.1:8765"
    echo "Left drag orbits, right drag pans, scroll zooms. Space pauses. Ctrl+C closes the page."
    wait "$live_pid" || true
    live_pid=""
  fi
  trap - EXIT INT TERM
  exit "$status"
fi

stop_sim
for pose in "${poses[@]}"; do
  out_dir="$base_out/poses/$pose"
  echo "Counterfactual: $pose"
  run_session || true
done
out_dir="$base_out"
python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
set +e
as_owner "$python" "$repo_dir/containers/sim/summary.py" "$base_out/poses" "$safety_checks" "$scene" "${poses[@]}"
status=$?
set -e
echo "Summary: $base_out/poses/summary.json"
exit "$status"
