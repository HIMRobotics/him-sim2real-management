#!/usr/bin/env bash
# Load the container dropped in containers/upload and run it on the robot network.
# The image's own entrypoint decides what to execute. This script does not
# interpret ONNX files or Unitree's image. Host network lets the container
# publish motor commands on eth0. g1-safety-stops is left running.
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
  echo "Docker socket is not writable by ${USER}. Re-running with sudo."
  exec sudo -- "$0" "$@"
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
image="${G1_IMAGE:-}"
extra=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --detach) detach=1; shift ;;
    --image) image="${2:?--image needs a name}"; shift 2 ;;
    --) shift; extra+=("$@"); break ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

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

stop_container
echo "Starting $image as $container_name on the robot network (interface ${ROBOT_INTERFACE:-eth0}, DDS domain ${ROBOT_DOMAIN:-0})."
echo "Publish joint commands on /safety/precheck. g1-safety-stops is not stopped."

run_args=(
  --name "$container_name"
  --network host
  -e "ROBOT_INTERFACE=${ROBOT_INTERFACE:-eth0}"
  -e "ROBOT_DOMAIN=${ROBOT_DOMAIN:-0}"
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-97}"
  -e "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}"
  -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
)
if [[ "${G1_NVIDIA:-0}" == 1 ]]; then
  run_args+=(--runtime nvidia)
fi
if [[ "$detach" -eq 1 ]]; then
  run_args+=(-d)
else
  run_args+=(--rm -it)
fi

"$runtime" run "${run_args[@]}" "$image" "${extra[@]}"
