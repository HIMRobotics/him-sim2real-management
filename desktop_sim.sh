#!/usr/bin/env bash
# Mac/Windows MuJoCo entry with the same flags as ./sim.sh.
# Host-only modes (--local, --self-test, --circle, --battery, replay) need no
# Docker. The ROS submission path uses containers/sim/compose.yaml instead of
# Linux network namespaces and sudo.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
upload_dir="$repo_dir/containers/upload"
out_dir="$repo_dir/containers/sim/output"
compose_file="$repo_dir/containers/sim/compose.yaml"
runtime="${CONTAINER_RUNTIME:-docker}"
compose=("$runtime" compose -f "$compose_file" --project-name g1-desktop-sim)

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./desktop_sim.sh [--image NAME] [--scene PATH] [--seconds N]
                [--safety-checks | --no-safety-checks]
                [--pose NAME | --counterfactuals] [--self-test] [--circle]
                [--command FILE] [--view]
                [--battery] [--push none|small|medium|heavy]
                [--example stairs|walker|balance] [--local] [--output DIR]
       ./desktop_sim.sh replay [TRACE]
       ./desktop_sim.sh stop

Mac/Windows entry for the G1 MuJoCo sim. Flags match ./sim.sh.

Host (no Docker, no sudo):
  --local --example NAME   ONNX example against the shared Runner
  --self-test              Hold the stand pose (default 1 s)
  --circle / --command     Cached walker on the flat floor
  --battery                Pose and push battery
  replay                   Saved recording only. Not the live container.

ROS submission path (Docker Compose, loopback-only net):
  --example NAME --view    Build that example image and stream it live
  --image NAME --view      Run an existing image live
  ./desktop_sim.sh --view  Load the one archive in containers/upload

--local            Run an example directly in MuJoCo without Docker/ROS/sudo.
--output DIR       Store this run separately.
--view             Open the site first. Press Start to run the container and
                   stream poses live. This is not a replay of a saved trace.
HELP
  exit 0
fi

resolve_dir() {
  mkdir -p "$1"
  (cd "$1" && pwd -P)
}

resolve_file() {
  local dir
  dir="$(cd -- "$(dirname -- "$1")" && pwd -P)"
  printf '%s/%s\n' "$dir" "$(basename -- "$1")"
}

sim_python() {
  if [[ -n "${G1_SIM_PYTHON:-}" ]]; then
    echo "$G1_SIM_PYTHON"
    return
  fi
  if [[ -x "$repo_dir/.venv-policy-safety/bin/python" ]]; then
    echo "$repo_dir/.venv-policy-safety/bin/python"
    return
  fi
  echo "${PYTHON:-python3}"
}

free_port() {
  local start="${G1_SIM_VIEW_PORT:-8765}"
  python3 - "$start" <<'PY'
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

serve_view() {
  local directory="$1"
  local port
  port="$(free_port)"
  echo "Open http://127.0.0.1:$port"
  echo "Left drag orbits, right drag pans, scroll zooms. Space pauses. Ctrl+C stops the viewer."
  exec "$python" -m http.server --bind 127.0.0.1 "$port" --directory "$directory"
}

live_pid=""
view_port=""

start_live_view() {
  view_port="$(free_port)"
  mkdir -p "$out_dir/view"
  rm -f "$out_dir/view/start" "$out_dir/live.jsonl" "$out_dir/live-view.log"
  local live_scene="${host_scene:-${scene:-}}"
  if [[ -n "$live_scene" ]]; then
    "$python" "$repo_dir/containers/sim/view.py" --idle "$out_dir/view" --pose "$pose" --scene "$live_scene"
  else
    "$python" "$repo_dir/containers/sim/view.py" --idle "$out_dir/view" --pose "$pose"
  fi
  if [[ -n "$live_scene" ]]; then
    "$python" "$repo_dir/containers/sim/live.py" \
      --directory "$out_dir/view" \
      --live "$out_dir/live.jsonl" \
      --start "$out_dir/view/start" \
      --report "$out_dir/report.json" \
      --pose "$pose" \
      --port "$view_port" \
      --scene "$live_scene" \
      >"$out_dir/live-view.log" 2>&1 &
  else
    "$python" "$repo_dir/containers/sim/live.py" \
      --directory "$out_dir/view" \
      --live "$out_dir/live.jsonl" \
      --start "$out_dir/view/start" \
      --report "$out_dir/report.json" \
      --pose "$pose" \
      --port "$view_port" \
      >"$out_dir/live-view.log" 2>&1 &
  fi
  live_pid=$!
  echo "Open http://127.0.0.1:$view_port"
  echo "Press Start to run the policy. The page streams live poses, not a saved trace."
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

stop_view() {
  if [[ -n "${live_pid:-}" ]]; then
    kill_tree "$live_pid"
    wait "$live_pid" 2>/dev/null || true
    live_pid=""
  fi
}

keep_viewer() {
  echo "Viewer is still at http://127.0.0.1:$view_port"
  echo "Left drag orbits, right drag pans, scroll zooms. Space pauses. Ctrl+C closes the page."
  wait "$live_pid" || true
  live_pid=""
}

if [[ "${1:-}" == "replay" ]]; then
  out_dir="$(resolve_dir "$out_dir")"
  trace="${2:-$out_dir/trace.npz}"
  python="$(sim_python)"
  if [[ ! -f "$trace" ]]; then
    echo "Trace not found: $trace" >&2
    exit 2
  fi
  trace="$(resolve_file "$trace")"
  view_dir="$(dirname -- "$trace")/view"
  "$python" "$repo_dir/containers/sim/view.py" "$trace" "$view_dir"
  echo "$view_dir/index.html"
  serve_view "$view_dir"
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
example=""
local_mode=0
stopping=0
poses=(standing seated supine prone left_side right_side)

kill_tree() {
  local pid="${1:-}" child
  [[ -n "$pid" ]] || return 0
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do
    kill_tree "$child"
  done
  kill "$pid" >/dev/null 2>&1 || true
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

out_dir="$(resolve_dir "$out_dir")"

if [[ "$stopping" -eq 1 ]]; then
  "${compose[@]}" down --remove-orphans >/dev/null 2>&1 || true
  echo "Stopped the desktop MuJoCo sim."
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
  command_file="$repo_dir/examples/circle.yaml"
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
  scene="$(resolve_file "$scene")"
fi

python="$(sim_python)"
if ! command -v "$python" >/dev/null 2>&1 && [[ ! -x "$python" ]]; then
  echo "Missing sim Python at $python. Create .venv-policy-safety (see packages/policy_safety/README.md) or set G1_SIM_PYTHON." >&2
  exit 2
fi

run_local_python() {
  local extra=("$@")
  if [[ "$counterfactuals" -eq 1 ]]; then
    for pose in "${poses[@]}"; do
      "$python" "$repo_dir/containers/sim/local.py" "${extra[@]}" --pose "$pose" --output "$out_dir/poses/$pose" || true
    done
    exec "$python" "$repo_dir/containers/sim/summary.py" "$out_dir/poses" "$safety_checks" "$scene" "${poses[@]}"
  fi
  if [[ "$view" -eq 1 ]]; then
    start_live_view
    trap 'stop_view' EXIT INT TERM
    wait_for_start
  fi
  local status=0
  G1_SIM_LIVE="$out_dir/live.jsonl" "$python" "$repo_dir/containers/sim/local.py" "${extra[@]}" --pose "$pose" --output "$out_dir" || status=$?
  echo "Report: $out_dir/report.json"
  if [[ "$view" -eq 1 && -n "$live_pid" ]]; then
    keep_viewer
    trap - EXIT INT TERM
  fi
  return "$status"
}

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
    rm -f "$out_dir/view/start" "$out_dir/live.jsonl" "$out_dir/battery/report.json" "$out_dir/live-view.log"
    view_port="$(free_port)"
    "$python" "$repo_dir/containers/sim/view.py" --idle "$out_dir/view" --pose "$pose"
    "$python" "$repo_dir/containers/sim/live.py" \
      --directory "$out_dir/view" \
      --live "$out_dir/live.jsonl" \
      --start "$out_dir/view/start" \
      --report "$out_dir/battery/report.json" \
      --pose "$pose" \
      --port "$view_port" \
      >"$out_dir/live-view.log" 2>&1 &
    live_pid=$!
    trap 'kill_tree "$live_pid"' EXIT INT TERM
    echo "Open http://127.0.0.1:$view_port"
    echo "Press Start to run ${pose} / ${push}."
    while [[ ! -f "$out_dir/view/start" ]]; do
      if ! kill -0 "$live_pid" 2>/dev/null; then
        echo "Viewer exited before Start." >&2
        cat "$out_dir/live-view.log" >&2 || true
        exit 1
      fi
      sleep 0.2
    done
    G1_SIM_LIVE="$out_dir/live.jsonl" G1_SIM_COMMAND="$command_file" "$python" "$repo_dir/containers/sim/battery.py" "${battery_args[@]}"
    status=$?
    echo "Report: $out_dir/battery/report.json"
    echo "Viewer is still at http://127.0.0.1:$view_port"
    wait "$live_pid" || true
    exit "$status"
  fi
  G1_SIM_COMMAND="$command_file" exec "$python" "$repo_dir/containers/sim/battery.py" "${battery_args[@]}"
fi

if [[ "$local_mode" -eq 1 ]]; then
  local_args=(--example "$example" --seconds "$seconds")
  [[ "$safety_checks" -eq 0 ]] && local_args+=(--no-safety-checks)
  [[ -n "$scene" ]] && local_args+=(--scene "$scene")
  run_local_python "${local_args[@]}"
  exit $?
fi

if [[ "$self_test" -eq 1 ]]; then
  local_args=(--self-test --seconds "$seconds")
  [[ "$safety_checks" -eq 0 ]] && local_args+=(--no-safety-checks)
  [[ -n "$scene" ]] && local_args+=(--scene "$scene")
  run_local_python "${local_args[@]}"
  exit $?
fi

if [[ "$circle" -eq 1 ]]; then
  local_args=(--circle --seconds "$seconds")
  [[ "$safety_checks" -eq 0 ]] && local_args+=(--no-safety-checks)
  [[ -n "$scene" ]] && local_args+=(--scene "$scene")
  G1_SIM_COMMAND="$command_file" run_local_python "${local_args[@]}"
  exit $?
fi

need_compose=0
if [[ -n "$example" || -n "$image" ]]; then
  need_compose=1
fi
if [[ "$need_compose" -eq 0 ]]; then
  shopt -s nullglob
  archives=("$upload_dir"/*.tar "$upload_dir"/*.tar.gz)
  shopt -u nullglob
  if [[ ${#archives[@]} -eq 1 ]]; then
    need_compose=1
  fi
fi

if [[ "$need_compose" -eq 0 ]]; then
  echo "No submission to run. Use one of:" >&2
  echo "  ./desktop_sim.sh --example walker --view" >&2
  echo "  ./desktop_sim.sh --image NAME --view" >&2
  echo "  put one archive in containers/upload and run ./desktop_sim.sh --view" >&2
  exit 2
fi

if ! command -v "$runtime" >/dev/null 2>&1; then
  echo "Docker is required to run a submission container on this machine." >&2
  echo "Start Docker Desktop, then: ./desktop_sim.sh --example walker --view" >&2
  exit 2
fi
if ! "$runtime" compose version >/dev/null 2>&1; then
  echo "Docker Compose is required for the ROS submission path." >&2
  exit 2
fi

load_image() {
  if [[ -n "$image" ]]; then
    return
  fi
  shopt -s nullglob
  archives=("$upload_dir"/*.tar "$upload_dir"/*.tar.gz)
  shopt -u nullglob
  if [[ ${#archives[@]} -ne 1 ]]; then
    echo "Put one image archive in containers/upload, pass --image, or use --self-test / --local." >&2
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

if [[ -n "$example" ]]; then
  prepared="$("$python" "$repo_dir/containers/examples/prepare.py" "$example")"
  [[ -n "$scene" ]] || scene="$prepared/environment/scene.xml"
  image="g1-sim-example-$example:local"
  echo "Building submission image $image"
  "$runtime" build -t "$image" "$prepared/container"
else
  load_image
fi

host_scene="$scene"
if [[ -n "$scene" ]]; then
  case "$scene" in
    "$repo_dir"/*) scene="/workspace/${scene#"$repo_dir"/}" ;;
    *)
      echo "Compose --scene must be inside this repository so it can be mounted." >&2
      exit 2
      ;;
  esac
fi

export G1_IMAGE="$image"
export G1_SIM_SECONDS="$seconds"
export G1_SIM_POSE="$pose"
export G1_SIM_SCENE="$scene"
export G1_SIM_SAFETY_CHECKS="$safety_checks"
export G1_SIM_EXAMPLE="$example"
export G1_SIM_OUTPUT="$out_dir"
if [[ "$view" -eq 1 ]]; then
  export G1_SIM_LIVE=/output/live.jsonl
else
  export G1_SIM_LIVE=""
fi

echo "Starting Compose sim (loopback-only net, no eth0)."
echo "Scene: ${host_scene:-bundled G1 on a flat floor}. Safety checks: $safety_checks."
echo "Submission image: $image"
"${compose[@]}" down --remove-orphans >/dev/null 2>&1 || true
"${compose[@]}" build sim
rm -f "$out_dir/ready" "$out_dir/report.json" "$out_dir/trace.npz" "$out_dir/live.jsonl"
mkdir -p "$out_dir"
if [[ "$view" -eq 1 ]]; then
  start_live_view
  trap 'stop_view; "${compose[@]}" down --remove-orphans >/dev/null 2>&1 || true' EXIT INT TERM
  wait_for_start
fi
"${compose[@]}" up -d net sim
for _ in $(seq 1 300); do
  [[ -f "$out_dir/ready" ]] && break
  if [[ "$view" -eq 1 ]] && ! kill -0 "$live_pid" 2>/dev/null; then
    echo "Viewer exited before the sim was ready." >&2
    cat "$out_dir/live-view.log" >&2 || true
    "${compose[@]}" logs sim >&2 || true
    exit 1
  fi
  sleep 0.1
done
if [[ ! -f "$out_dir/ready" ]]; then
  echo "Sim service did not become ready." >&2
  "${compose[@]}" logs sim >&2 || true
  "${compose[@]}" down --remove-orphans >/dev/null 2>&1 || true
  exit 1
fi
"${compose[@]}" --profile submission up -d submission
set +e
"${compose[@]}" wait sim
status=$?
set -e
"${compose[@]}" logs submission >"$out_dir/policy.log" 2>&1 || true
"${compose[@]}" down --remove-orphans >/dev/null 2>&1 || true
if [[ -f "$out_dir/report.json" ]]; then
  echo "Report: $out_dir/report.json"
  cat "$out_dir/report.json"
else
  echo "Sim service produced no report." >&2
  status=2
fi
if [[ "$view" -eq 1 && -n "$live_pid" ]]; then
  keep_viewer
  trap - EXIT INT TERM
fi
exit "$status"
