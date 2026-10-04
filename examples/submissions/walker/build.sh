#!/usr/bin/env bash
# Build the walker test submission in .cache/walker_submission.
# Does not start the container and does not call setup.sh.
set -euo pipefail

if [[ "${EUID}" -ne 0 && ! -w /var/run/docker.sock ]]; then
  echo "Docker socket is not writable. Re-running with sudo."
  exec sudo --preserve-env=UNITREE_SDK2PY -- "$0" "$@"
fi

example="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$example/../../.." && pwd)"
bundle="$repo/examples/policies/walker"
commands="$repo/examples/commands/walker"
output="$repo/.cache/walker_submission"
stage="$output/build"
model="$bundle/walker.onnx"
sdk="${UNITREE_SDK2PY:-/home/unitree/Workspace/unitree_sdk2_python/unitree_sdk2py}"
dds_py="${CYCLONEDDS_PY:-/home/unitree/.local/lib/python3.8/site-packages/cyclonedds}"
dds_lib="${CYCLONEDDS_LIB:-/home/unitree/cyclonedds_ws/install/cyclonedds/lib}"

if [[ ! -f "$model" || ! -f "$bundle/walker.onnx.data" || ! -f "$bundle/walker.yaml" ]]; then
  echo "Walker weights missing in examples/policies/walker/" >&2
  exit 1
fi
if [[ ! -d "$sdk" ]]; then
  echo "unitree_sdk2py missing: $sdk" >&2
  echo "Set UNITREE_SDK2PY to the unitree_sdk2py package directory." >&2
  exit 1
fi
if [[ ! -d "$dds_py" || ! -f "$dds_lib/libddsc.so.0.10.2" ]]; then
  echo "CycloneDDS 0.10.2 for Python 3.8 arm64 is not on PyPI." >&2
  echo "Expected the host build at:" >&2
  echo "  $dds_py" >&2
  echo "  $dds_lib/libddsc.so.0.10.2" >&2
  exit 1
fi

rm -rf "$stage"
mkdir -p "$stage/interfaces"
cp "$example/Dockerfile" "$example/entrypoint.sh" "$example/walker_node.py" "$example/policy.py" \
  "$bundle/walker.yaml" "$commands/stand.yaml" "$commands/circle.yaml" "$stage/"
cp "$model" "$bundle/walker.onnx.data" "$stage/"
cp -a "$sdk" "$stage/unitree_sdk2py"
cp -a "$dds_py" "$stage/cyclonedds"
mkdir -p "$stage/ddsc"
cp -a "$dds_lib"/libddsc.so* "$dds_lib"/libcycloneddsidl.so* "$stage/ddsc/"
printf '%s\n' "in_wheel = False" "library_path = '/opt/walker/ddsc/libddsc.so'" \
  > "$stage/cyclonedds/__library__.py"
cp -a "$repo/ros2/him_safety_interfaces" "$stage/interfaces/him_safety_interfaces"
find "$stage" -type d -name __pycache__ -prune -exec rm -rf {} +

echo "Building submission:local"
docker build --platform linux/arm64 -t submission:local "$stage"
docker save submission:local -o "$output/submission.tar"
if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != root ]]; then
  chown "${SUDO_USER}:${SUDO_USER}" "$output/submission.tar"
fi
echo "Wrote $output/submission.tar"
echo "Next: cp .cache/walker_submission/submission.tar containers/upload/submission.tar && ./sim.sh"
