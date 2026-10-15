# Sourced by sim.sh for the Docker Compose backend.
compose=("$runtime" compose -f "$repo_dir/containers/sim/compose.yaml" --project-name g1-desktop-sim)

stop_sim() {
  "${compose[@]}" down --remove-orphans >/dev/null 2>&1 || true
}

if [[ "$stopping" -eq 1 ]]; then
  stop_sim
  echo "Stopped the desktop MuJoCo sim."
  exit 0
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

run_session() {
  export G1_IMAGE="$image" G1_SIM_SECONDS="$seconds" G1_SIM_POSE="$pose"
  export G1_SIM_SCENE="" G1_SIM_SAFETY_CHECKS="$safety_checks"
  export G1_SIM_EXAMPLE="$example" G1_SIM_OUTPUT="$out_dir" G1_SIM_LIVE=""
  export G1_COMMAND_FILE="${command_file:-}"
  if [[ -n "$scene" ]]; then
    case "$scene" in
      "$repo_dir"/*) export G1_SIM_SCENE="/workspace/${scene#"$repo_dir"/}" ;;
      *) echo "Compose --scene must be inside this repository so it can be mounted." >&2; return 2 ;;
    esac
  fi
  if [[ -n "${G1_JOINT_PROFILE:-}" && "$G1_JOINT_PROFILE" != /workspace/* ]]; then
    case "$G1_JOINT_PROFILE" in
      "$repo_dir"/*) export G1_JOINT_PROFILE="/workspace/${G1_JOINT_PROFILE#"$repo_dir"/}" ;;
      *) echo "Compose G1_JOINT_PROFILE must be inside this repository so it can be mounted." >&2; return 2 ;;
    esac
  fi
  [[ "$view" -eq 0 ]] || export G1_SIM_LIVE=/output/live.jsonl
  stop_sim
  mkdir -p "$out_dir"
  rm -f "$out_dir/ready" "$out_dir/report.json" "$out_dir/trace.npz" "$out_dir/live.jsonl"
  echo "Starting $pose (Compose, loopback-only net, no eth0)."
  "${compose[@]}" up -d net sim || return 2
  for _ in $(seq 1 600); do
    [[ -f "$out_dir/ready" || -f "$out_dir/report.json" ]] && break
    if [[ -n "$live_pid" ]] && ! kill -0 "$live_pid" 2>/dev/null; then
      echo "Viewer exited before the sim was ready." >&2
      cat "$out_dir/live-view.log" >&2 || true
      "${compose[@]}" logs sim >&2 || true
      return 1
    fi
    sleep 0.1
  done
  if [[ ! -f "$out_dir/ready" ]]; then
    echo "Sim service did not become ready." >&2
    "${compose[@]}" logs sim >&2 || true
    "${compose[@]}" down --remove-orphans >/dev/null 2>&1 || true
    return 1
  fi
  "${compose[@]}" --profile submission up -d submission || return 2
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
  return "$status"
}
