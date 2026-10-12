# Sourced by sim.sh for the Linux host-ROS backend.
ros_dir="$repo_dir/containers/sim/.ros"
net_name=g1-sim-net
user_name=g1-sim-user

if [[ "$(uname -s)" != Linux ]]; then
  echo "The Linux network-namespace sim does not run on $(uname -s)." >&2
  echo "Use ./desktop_sim.sh with the same flags. --local, --self-test, --circle, --battery, and replay need no Docker." >&2
  exit 2
fi
if [[ "${EUID}" -ne 0 ]]; then
  echo "Sim needs root to enter the isolated network. Re-running with sudo."
  exec sudo --preserve-env=G1_SIM_BACKEND,G1_SIM_VIEW_PORT,G1_SIM_SECONDS,G1_SIM_PYTHON,G1_SIM_THRASH_MODE,G1_JOINT_PROFILE,G1_TRIAL_CONFIG,G1_TASK_COMMAND,CONTAINER_RUNTIME,G1_IMAGE,HIM_BALANCE_ONNX,DISPLAY,XAUTHORITY -- "$0" "${original_args[@]}"
fi

stop_sim() {
  local name
  for name in bridge hold; do
    if [[ -f "$out_dir/$name.pid" ]]; then
      kill_tree "$(cat "$out_dir/$name.pid")"
      rm -f "$out_dir/$name.pid"
    fi
  done
  "$runtime" rm -f "$user_name" "$net_name" >/dev/null 2>&1 || true
}

if [[ "$stopping" -eq 1 ]]; then
  stop_sim
  echo "Stopped the MuJoCo container sim."
  exit 0
fi

in_netns() {
  local pid="$1"
  shift
  # -n puts the bridge on the sim container's loopback. Discovery is unicast
  # to 127.0.0.1 there (containers/sim/fastdds.xml), not host shared memory.
  nsenter -t "$pid" -n -- runuser -u "$owner" -- "$@"
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
    "G1_SIM_THRASH_MODE=${G1_SIM_THRASH_MODE:-}"
    "G1_JOINT_PROFILE=${G1_JOINT_PROFILE:-}"
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
    command_name="$(basename "${command_file%.yaml}")"
    [[ -n "$command_name" ]] || command_name=circle
    sim_env+=("G1_SIM_CIRCLE=1" "G1_SIM_EXAMPLE=$command_name" "G1_SIM_COMMAND=$command_file")
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
    trial_args=()
    if [[ -n "${G1_TRIAL_CONFIG:-}" ]]; then
      trial_args=(-v "$(realpath "$G1_TRIAL_CONFIG"):/trial.json:ro" -e G1_TRIAL_CONFIG=/trial.json)
    fi
    if [[ -n "${command_file:-}" ]]; then
      trial_args+=(-v "$(realpath "$command_file"):/command.yaml:ro" -e G1_COMMAND_FILE=/command.yaml)
    fi
    "$runtime" run -d --name "$user_name" --network "container:$net_name" --ipc "container:$net_name" \
      "${trial_args[@]}" -e "G1_THRASH_MODE=${G1_SIM_THRASH_MODE:-monitor}" \
      -e "G1_TASK_COMMAND=${G1_TASK_COMMAND:-}" \
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
  "$runtime" logs "$user_name" >"$out_dir/policy.log" 2>&1 || true
  stop_sim

  if [[ "$status" -eq 2 && "$self_test" -eq 0 && "$circle" -eq 0 ]]; then
    echo "Policy container log: $out_dir/policy.log" >&2
    tail -n 60 "$out_dir/policy.log" >&2
  fi

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
