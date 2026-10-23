#!/usr/bin/env bash
# Build this directory into a linux/arm64 image and save it as submission.tar.
# Usage: ./build.sh [TAG]          (default tag: submission:local)
# Set HIM_REPO if this directory was copied outside the repository.
set -euo pipefail

here="$(cd -- "$(dirname -- "$0")" && pwd -P)"
repo="${HIM_REPO:-$(cd -- "$here/../.." && pwd -P)}"
tag="${1:-submission:local}"
interfaces="$repo/ros2/him_safety_interfaces"
if [[ ! -f "$interfaces/package.xml" ]]; then
  echo "Missing $interfaces. Set HIM_REPO to the him-sim2real-management checkout." >&2
  exit 2
fi

stage="$here/.build"
rm -rf "$stage"
mkdir -p "$stage/interfaces"
cp -R "$interfaces" "$stage/interfaces/"
cp -R "$here/assets" "$stage/assets"
cp "$here/Dockerfile" "$here/requirements.txt" "$here/policy.py" "$here/node.py" \
  "$here/entrypoint.sh" "$stage/"

docker build --platform linux/arm64 -t "$tag" "$stage"
docker save "$tag" -o "$here/submission.tar"
echo "Saved $tag to $here/submission.tar"
echo "Next: cp $here/submission.tar $repo/containers/upload/ && $repo/desktop_sim.sh --view"
