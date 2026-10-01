#!/usr/bin/env bash
# Build examples/submission.tar from the G1 walker.
# Does not start the container and does not call setup.sh.
set -euo pipefail

if [[ "${EUID}" -ne 0 && ! -w /var/run/docker.sock ]]; then
  echo "Docker socket is not writable. Re-running with sudo."
  exec sudo --preserve-env=UNITREE_SDK2PY -- "$0" "$@"
fi

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
example="$repo/examples"
stage="$example/.build"
model="$example/walker.onnx"
sdk="${UNITREE_SDK2PY:-/home/unitree/Workspace/unitree_sdk2_python/unitree_sdk2py}"
dds_py="${CYCLONEDDS_PY:-/home/unitree/.local/lib/python3.8/site-packages/cyclonedds}"
dds_lib="${CYCLONEDDS_LIB:-/home/unitree/cyclonedds_ws/install/cyclonedds/lib}"

if [[ ! -f "$model" || ! -f "$example/walker.onnx.data" || ! -f "$example/walker.yaml" ]]; then
  echo "Walker weights missing in examples/: walker.onnx, walker.onnx.data, walker.yaml" >&2
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
cp "$example/Dockerfile" "$example/entrypoint.sh" "$example/balance_node.py" "$example/policy.py" \
  "$example/walker.yaml" "$example/stand.yaml" "$example/circle.yaml" "$stage/"
cp "$model" "$example/walker.onnx.data" "$stage/"
cp -a "$sdk" "$stage/unitree_sdk2py"
cp -a "$dds_py" "$stage/cyclonedds"
mkdir -p "$stage/ddsc"
cp -a "$dds_lib"/libddsc.so* "$dds_lib"/libcycloneddsidl.so* "$stage/ddsc/"
printf '%s\n' "in_wheel = False" "library_path = '/opt/balance/ddsc/libddsc.so'" \
  > "$stage/cyclonedds/__library__.py"
cp -a "$repo/ros2/him_safety_interfaces" "$stage/interfaces/him_safety_interfaces"
find "$stage" -type d -name __pycache__ -prune -exec rm -rf {} +

echo "Building submission:local"
docker build --platform linux/arm64 -t submission:local "$stage"
docker save submission:local -o "$example/submission.tar"
if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != root ]]; then
  chown "${SUDO_USER}:${SUDO_USER}" "$example/submission.tar"
fi
echo "Wrote $example/submission.tar"
echo "Next: cp examples/submission.tar containers/upload/submission.tar && ./sim.sh"
