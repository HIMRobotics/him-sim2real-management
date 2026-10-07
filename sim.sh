#!/usr/bin/env bash
# Shared simulation CLI. Only the ROS execution backend varies by platform.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
upload_dir="$repo_dir/containers/upload"
out_dir="$repo_dir/containers/sim/output"
runtime="${CONTAINER_RUNTIME:-docker}"
backend="${G1_SIM_BACKEND:-namespace}"
owner="${SUDO_USER:-${USER:-root}}"
python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
[[ -n "${G1_SIM_PYTHON:-}" || -x "$python" ]] || python="${PYTHON:-python3}"
original_args=("$@")
if [[ "$backend" != namespace && "$backend" != compose ]]; then
  echo "G1_SIM_BACKEND must be namespace or compose" >&2
  exit 2
fi

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./sim.sh [--image NAME] [--scene PATH] [--seconds N]
                [--safety-checks | --no-safety-checks]
                [--pose NAME | --counterfactuals] [--self-test] [--circle]
                [--command FILE] [--view]
                [--battery] [--push none|small|medium|heavy]
                [--example NAME] [--local] [--output DIR]
       ./sim.sh replay [TRACE]
       ./sim.sh stop

On macOS or Windows use ./desktop_sim.sh (same flags). This script's
ROS path needs Linux network namespaces. Desktop uses Docker Compose.
Desktop --self-test and --circle run locally without ROS or Docker.

--scene PATH        Load a host-side MuJoCo XML scene and its relative assets.
                    Default: the bundled G1 on a flat floor.
--safety-checks     Enable safety checks (default).
--no-safety-checks  Disable safety limits, freshness/timeouts, and fall checks.
                    Message format and finite-value validation still apply.
--example NAME     Policy package: stairs, walker, or balance, from
                   examples/policies. Builds image g1-sim-example-<name>:local.
                   The report policy field is that name. This is not a velocity file.
                   walker/thrash: belly-up walker, only thrash stops enabled.
                   walker/thrash-monitor: same experiment, readings without thrash stops.
--image NAME       Run this container instead of the archive in containers/upload.
--local            Run an example directly in MuJoCo without Docker/ROS/sudo.
--output DIR       Store this run separately for comparison and replay.
--circle           Cached walker in examples/policies/walker. No container.
                   Command file examples/commands/walker/circle.yaml.
                   The report policy field is circle.
--command FILE     Walker velocity file.
                   Alone, the cached walker plays it on the plant. No container.
                   The report policy field is the file name without .yaml.
                   With --example walker, that container plays the same file.
                   stand.yaml stays put. circle.yaml is the loop.
                   forward_stop.yaml walks at 0.5 m/s for 2 s, then steps down
                   0.1 m/s each second, then selects NVIDIA Balance.
                   Use --seconds 7.
--battery          Flat-ground battery: every pose, then 10/50/150 N pelvis
                   pushes. Writes containers/sim/output/battery/report.json.
                   With --circle, the cached walker gaits instead of a hold.
--push NAME        One battery case: none, small, medium, or heavy.
                   With --view, open the page and press Start to watch it.
--view             Open the live viewer and press Start to run.
                   Uses the first free port from G1_SIM_VIEW_PORT (8765).
                   Ctrl+C closes the viewer.
HELP
  exit 0
fi

as_owner() {
  if [[ "${EUID}" -eq 0 && "$owner" != root ]]; then
    runuser -u "$owner" -- "$@"
  else
    "$@"
  fi
}

kill_tree() {
  local pid="${1:-}" child
  [[ -n "$pid" ]] || return 0
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do
    kill_tree "$child"
  done
  kill "$pid" >/dev/null 2>&1 || true
}

free_port() {
  local start="${G1_SIM_VIEW_PORT:-8765}"
  "$python" - "$start" <<'PY'
import socket
import sys

start = int(sys.argv[1])
for port in range(start, start + 50):
    sock = socket.socket()
    try:
        sock.bind(("127.0.0.1", port))
    except OSError:
        continue
    else:
        print(port)
        raise SystemExit(0)
    finally:
        sock.close()
raise SystemExit(f"no free viewer port in {start}-{start + 49}")
PY
}

live_pid=""
view_port=""
stop_view() {
  if [[ -n "${live_pid:-}" ]]; then
    kill_tree "$live_pid"
    wait "$live_pid" 2>/dev/null || true
    live_pid=""
  fi
}

start_live_view() {
  view_port="$(free_port)"
  mkdir -p "$out_dir"
  chown "$owner" "$out_dir" 2>/dev/null || true
  rm -f "$out_dir/view/start" "$out_dir/live.jsonl" "${viewer_report:-$out_dir/report.json}"
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
    --report "${viewer_report:-$out_dir/report.json}" \
    --port "$view_port" \
    --pose "$pose" \
    "${scene_args[@]}" &
  live_pid=$!
  echo "Open http://127.0.0.1:$view_port"
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

keep_viewer() {
  echo "Viewer is still at http://127.0.0.1:$view_port"
  echo "Left drag orbits, right drag pans, scroll zooms. Space pauses. Ctrl+C closes the page."
  wait "$live_pid" || true
  live_pid=""
}

if [[ "${1:-}" == "replay" ]]; then
  trace="${2:-$out_dir/trace.npz}"
  if [[ ! -f "$trace" ]]; then
    echo "Trace not found: $trace" >&2
    exit 2
  fi
  view_dir="$(cd -- "$(dirname -- "$trace")" && pwd -P)/view"
  "$python" "$repo_dir/containers/sim/view.py" "$trace" "$view_dir"
  port="$(free_port)"
  echo "Open http://127.0.0.1:$port"
  echo "Left drag orbits, right drag pans, scroll zooms. Space pauses. Ctrl+C stops the viewer."
  exec "$python" -m http.server --bind 127.0.0.1 "$port" --directory "$view_dir"
fi

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
safety_set=0
example=""
example_arg=""
local_mode=0
stopping=0
# Keep this list in step with containers/sim/plant.py POSES.
poses=(standing seated supine prone left_side right_side)

if [[ "${1:-}" == "stop" ]]; then
  stopping=1
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --self-test) self_test=1; shift ;;
    --circle) circle=1; shift ;;
    --command) command_file="${2:?--command needs a yaml file}"; shift 2 ;;
    --battery) battery=1; shift ;;
    --push) push="${2:?--push needs none, small, medium, or heavy}"; shift 2 ;;
    --view) view=1; shift ;;
    --counterfactuals) counterfactuals=1; shift ;;
    --pose) pose="${2:?--pose needs a name}"; pose_set=1; shift 2 ;;
    --seconds) seconds="${2:?--seconds needs a duration}"; seconds_set=1; shift 2 ;;
    --image) image="${2:?--image needs a name}"; shift 2 ;;
    --scene) scene="${2:?--scene needs an XML path}"; shift 2 ;;
    --safety-checks) safety_checks=1; safety_set=1; shift ;;
    --no-safety-checks) safety_checks=0; safety_set=1; shift ;;
    --example) example_arg="${2:?--example needs a name}"; example="$example_arg"; shift 2 ;;
    --local) local_mode=1; shift ;;
    --output) out_dir="${2:?--output needs a directory}"; shift 2 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ -n "$example" ]]; then
  settings="$("$python" "$repo_dir/containers/examples/prepare.py" "$example" --settings)"
  read -r example default_pose default_safety detector_mode <<<"$settings"
  [[ "$pose_set" -eq 1 ]] || pose="$default_pose"
  [[ "$safety_set" -eq 1 ]] || safety_checks="$default_safety"
  if [[ "$detector_mode" != default ]]; then
    export G1_SIM_THRASH_MODE="$detector_mode"
  fi
fi

if [[ -n "$command_file" && -z "$example" ]]; then
  circle=1
fi
if [[ -n "$command_file" && -n "$example_arg" && "$example_arg" != walker ]]; then
  echo "--command is a walker velocity file. Use --example walker, or pass --command alone for the direct plant path." >&2
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

# Signal handlers exit so cleanup cannot resume a stopped simulation.
trap 'stop_view' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "$stopping" -eq 1 ]]; then
  source "$repo_dir/containers/sim/$backend.sh"
  exit 0
fi

if [[ "$battery" -eq 1 ]]; then
  battery_args=(--output "$out_dir/battery" --seconds "$seconds")
  [[ "$safety_checks" -eq 0 ]] && battery_args+=(--no-safety-checks)
  if [[ -n "$push" || "$pose_set" -eq 1 ]]; then
    battery_args+=(--pose "$pose")
  fi
  [[ -n "$push" ]] && battery_args+=(--push "$push")
  [[ "$circle" -eq 1 ]] && battery_args+=(--circle)
  if [[ "$view" -eq 1 ]]; then
    mkdir -p "$out_dir/battery"
    viewer_report="$out_dir/battery/report.json"
    scene=""
    start_live_view
    wait_for_start
    status=0
    G1_SIM_LIVE="$out_dir/live.jsonl" G1_SIM_COMMAND="$command_file" "$python" "$repo_dir/containers/sim/battery.py" "${battery_args[@]}" || status=$?
    echo "Report: $viewer_report"
    keep_viewer
    exit "$status"
  fi
  G1_SIM_COMMAND="$command_file" exec "$python" "$repo_dir/containers/sim/battery.py" "${battery_args[@]}"
fi

if [[ "$local_mode" -eq 1 || ( "$backend" == compose && ( "$self_test" -eq 1 || "$circle" -eq 1 ) ) ]]; then
  local_args=(--seconds "$seconds")
  [[ -z "$example" ]] || local_args+=(--example "$example")
  [[ "$self_test" -eq 0 ]] || local_args+=(--self-test)
  [[ "$circle" -eq 0 ]] || local_args+=(--circle)
  [[ "$safety_checks" -eq 1 ]] || local_args+=(--no-safety-checks)
  [[ -z "$scene" ]] || local_args+=(--scene "$scene")
  export G1_SIM_COMMAND="$command_file"
  if [[ -n "$command_file" && -n "$example" ]]; then
    export G1_COMMAND_FILE="$command_file"
  fi
  if [[ "$counterfactuals" -eq 1 ]]; then
    for pose in "${poses[@]}"; do
      "$python" "$repo_dir/containers/sim/local.py" "${local_args[@]}" --pose "$pose" --output "$out_dir/poses/$pose" || true
    done
    exec "$python" "$repo_dir/containers/sim/summary.py" "$out_dir/poses" "$safety_checks" "$scene" "${poses[@]}"
  fi
  if [[ "$view" -eq 1 ]]; then
    if [[ -n "$example" && -z "$scene" ]]; then
      prepared="$("$python" "$repo_dir/containers/examples/prepare.py" "$example" --assets-only)"
      scene="$prepared/environment/scene.xml"
      local_args+=(--scene "$scene")
    fi
    start_live_view
    wait_for_start
    export G1_SIM_LIVE="$out_dir/live.jsonl"
  fi
  status=0
  "$python" "$repo_dir/containers/sim/local.py" "${local_args[@]}" --pose "$pose" --output "$out_dir" || status=$?
  echo "Report: $out_dir/report.json"
  [[ ! -f "$out_dir/trace.npz" ]] || echo "Replay: ./sim.sh replay $out_dir/trace.npz"
  [[ "$view" -eq 0 ]] || keep_viewer
  exit "$status"
fi

source "$repo_dir/containers/sim/$backend.sh"
trap 'stop_sim; stop_view' EXIT
if [[ -n "$example" ]]; then
  prepared="$(as_owner "$python" "$repo_dir/containers/examples/prepare.py" "$example")"
  [[ -n "$scene" ]] || scene="$prepared/environment/scene.xml"
  export G1_JOINT_PROFILE="$prepared/container/project/joint_profile.json"
  image="g1-sim-example-$example:local"
  "$runtime" build -t "$image" "$prepared/container"
elif [[ -z "$image" && "$self_test" -eq 0 && "$circle" -eq 0 ]]; then
  shopt -s nullglob
  archives=("$upload_dir"/*.tar "$upload_dir"/*.tar.gz)
  shopt -u nullglob
  if [[ ${#archives[@]} -ne 1 ]]; then
    echo "Put one image archive in containers/upload, pass --image, or use --self-test / --local." >&2
    exit 1
  fi
  loaded="$("$runtime" load -i "${archives[0]}")"
  echo "$loaded"
  image="$(printf '%s\n' "$loaded" | sed -n 's/^Loaded image: //p' | tail -n 1)"
  if [[ -z "$image" ]]; then
    echo "The archive did not report a loaded image name." >&2
    exit 1
  fi
fi
if [[ "$backend" == compose ]]; then
  export G1_IMAGE="$image" G1_SIM_OUTPUT="$out_dir"
  "${compose[@]}" build sim
fi

base_out="$out_dir"
if [[ "$counterfactuals" -eq 1 ]]; then
  for pose in "${poses[@]}"; do
    out_dir="$base_out/poses/$pose"
    run_session || true
  done
  out_dir="$base_out"
  as_owner "$python" "$repo_dir/containers/sim/summary.py" "$base_out/poses" "$safety_checks" "$scene" "${poses[@]}"
else
  if [[ "$view" -eq 1 ]]; then
    start_live_view
    wait_for_start
  fi
  status=0
  run_session || status=$?
  stop_sim
  [[ "$view" -eq 0 ]] || keep_viewer
  exit "$status"
fi
