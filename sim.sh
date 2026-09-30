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

if [[ "${1:-}" == "replay" ]]; then
  trace="${2:-$out_dir/trace.npz}"
  python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
  "$python" "$repo_dir/containers/sim/view.py" "$trace" "$out_dir/view"
  echo "Open http://127.0.0.1:8765"
  echo "Left drag orbits, right drag pans, scroll zooms. Space pauses. Ctrl+C stops the viewer."
  exec python3 -m http.server --bind 127.0.0.1 8765 --directory "$out_dir/view"
fi

if [[ "${EUID}" -ne 0 ]]; then
  echo "Sim needs root to enter the isolated network. Re-running with sudo."
  exec sudo --preserve-env=G1_SIM_SECONDS,G1_SIM_PYTHON,CONTAINER_RUNTIME,G1_IMAGE,DISPLAY,XAUTHORITY -- "$0" "$@"
fi

owner="${SUDO_USER:-$USER}"
seconds="${G1_SIM_SECONDS:-10}"
seconds_set=0
self_test=0
view=0
counterfactuals=0
pose="standing"
image="${G1_IMAGE:-}"
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

if [[ "${1:-}" == "stop" ]]; then
  stop_sim
  echo "Stopped the MuJoCo container sim."
  exit 0
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --self-test) self_test=1; shift ;;
    --view) view=1; shift ;;
    --counterfactuals) counterfactuals=1; shift ;;
    --pose) pose="${2:?--pose needs a name}"; shift 2 ;;
    --seconds) seconds="${2:?--seconds needs a duration}"; seconds_set=1; shift 2 ;;
    --image) image="${2:?--image needs a name}"; shift 2 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

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
if ! [[ "$seconds" =~ ^[0-9]+([.][0-9]+)?$ ]] || ! awk "BEGIN { exit !($seconds > 0) }"; then
  echo "--seconds must be positive" >&2
  exit 2
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

if "$runtime" ps --format '{{.Names}}' | grep -qx g1-user-container; then
  echo "The robot container is running. ./setup.sh stop it before starting the sim." >&2
  exit 1
fi

if [[ ! -f "$ros_dir/install/setup.bash" ]]; then
  echo "Building him_safety_interfaces."
  as_owner bash -c "source /opt/ros/foxy/setup.bash && cd '$repo_dir' && colcon build --packages-select him_safety_interfaces --base-paths ros2 --build-base '$ros_dir/build' --install-base '$ros_dir/install'"
fi

if [[ "$self_test" -eq 0 ]]; then
  load_image
fi

base_out="$out_dir"
run_session() {
  stop_sim
  mkdir -p "$out_dir"
  rm -f "$out_dir/ready" "$out_dir/report.json"
  chown "$owner" "$out_dir" 2>/dev/null || true

  echo "Starting $pose (isolated network, loopback only, no eth0)."
  "$runtime" run -d --name "$net_name" --network none ros:foxy-ros-base sleep infinity >/dev/null
  net_pid="$("$runtime" inspect -f '{{.State.Pid}}' "$net_name")"
  if nsenter -t "$net_pid" -n -- ip link show eth0 >/dev/null 2>&1; then
    echo "Isolated network still has eth0. Not starting the sim." >&2
    stop_sim
    return 1
  fi

  sim_env=(
    "G1_SIM_NETNS=1"
    "G1_SIM_POSE=$pose"
    "G1_SIM_SECONDS=$seconds"
    "G1_SIM_REPORT=$out_dir/report.json"
    "G1_SIM_READY=$out_dir/ready"
    "G1_SIM_PYTHON=${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
    "ROBOT_INTERFACE=lo"
    "ROBOT_DOMAIN=0"
    "ROS_DOMAIN_ID=97"
    "ROS_LOCALHOST_ONLY=1"
    "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    "FASTRTPS_DEFAULT_PROFILES_FILE=$repo_dir/containers/sim/fastdds.xml"
    "G1_SIM_TRACE=$out_dir/trace.npz"
  )

  bridge_cmd="cd '$repo_dir' && source /opt/ros/foxy/setup.bash && source '$ros_dir/install/setup.bash' && exec python3 containers/sim/bridge.py"
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

  if [[ "$self_test" -eq 1 ]]; then
    hold_cmd="cd '$repo_dir' && source /opt/ros/foxy/setup.bash && source '$ros_dir/install/setup.bash' && exec python3 containers/sim/hold.py"
    in_netns "$net_pid" env "${sim_env[@]}" bash -c "$hold_cmd" >"$out_dir/hold.log" 2>&1 &
    echo $! >"$out_dir/hold.pid"
  else
    "$runtime" run -d --name "$user_name" --network "container:$net_name" \
      -v "$repo_dir/containers/sim/fastdds.xml:/fastdds.xml:ro" \
      -e ROBOT_INTERFACE=lo \
      -e ROBOT_DOMAIN=0 \
      -e ROS_DOMAIN_ID=97 \
      -e ROS_LOCALHOST_ONLY=1 \
      -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml \
      "$image" >/dev/null
  fi

  set +e
  wait "$(cat "$out_dir/bridge.pid")"
  status=$?
  set -e
  if [[ -f "$out_dir/hold.pid" ]]; then
    kill_tree "$(cat "$out_dir/hold.pid")"
  fi
  "$runtime" rm -f "$user_name" "$net_name" >/dev/null 2>&1 || true
  rm -f "$out_dir/bridge.pid" "$out_dir/hold.pid"

  if [[ -f "$out_dir/report.json" ]]; then
    echo "Report: $out_dir/report.json"
    cat "$out_dir/report.json"
    if [[ -f "$out_dir/trace.npz" ]]; then
      if [[ "$out_dir" == "$base_out" ]]; then
        echo "Replay: ./sim.sh replay"
      else
        echo "Replay: ./sim.sh replay $out_dir/trace.npz"
      fi
    fi
  else
    echo "Sim bridge produced no report." >&2
    cat "$out_dir/bridge.log" >&2 || true
    status=2
  fi
  return "$status"
}

if [[ "$counterfactuals" -eq 0 ]]; then
  status=0
  run_session || status=$?
  if [[ "$view" -eq 1 && -f "$out_dir/trace.npz" ]]; then
    python="${G1_SIM_PYTHON:-$repo_dir/.venv-policy-safety/bin/python}"
    as_owner "$python" "$repo_dir/containers/sim/view.py" "$out_dir/trace.npz" "$out_dir/view"
    echo "Open http://127.0.0.1:8765"
    echo "Left drag orbits, right drag pans, scroll zooms. Space pauses. Ctrl+C stops the viewer."
    as_owner python3 -m http.server --bind 127.0.0.1 8765 --directory "$out_dir/view"
  fi
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
as_owner "$python" - "$base_out/poses" "$base_out/poses/summary.json" "${poses[@]}" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
summary_path = Path(sys.argv[2])
rows = []
for name in sys.argv[3:]:
    path = root / name / "report.json"
    if path.is_file():
        report = json.loads(path.read_text())
        rows.append(
            {
                "pose": name,
                "status": report.get("status"),
                "reason": report.get("reason"),
                "fall_check": report.get("fall_check"),
                "commands": report.get("commands"),
                "report": str(path),
            }
        )
    else:
        rows.append({"pose": name, "status": "error", "reason": "no_report", "report": None})
codes = {"within_limits": 0, "violation": 1}
worst = 0
for row in rows:
    worst = max(worst, codes.get(row["status"], 2))
summary = {
    "version": 1,
    "status": {0: "within_limits", 1: "violation", 2: "error"}[worst],
    "poses": rows,
    "hardware_authorization": False,
}
summary_path.write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))
sys.exit(worst)
PY
status=$?
set -e
echo "Summary: $base_out/poses/summary.json"
exit "$status"
