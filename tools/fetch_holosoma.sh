#!/usr/bin/env bash
# Fetch the pinned Holosoma checkout that Holosoma startup/recovery reads its FastSAC weights from.
#   tools/fetch_holosoma.sh [DEST]     default: $HIM_HOLOSOMA_DIR or /home/unitree/holosoma
# robot_safety/holosoma_policy.py loads
#   DEST/src/holosoma_inference/holosoma_inference/models/loco/g1_29dof/fastsac_g1_29dof.onnx
# (HIM_HOLOSOMA_ONNX overrides that path).
set -euo pipefail

url="https://github.com/amazon-far/holosoma.git"
commit="d1e8499c10670f44c49d161b9876a4c17ac305f7"
onnx="src/holosoma_inference/holosoma_inference/models/loco/g1_29dof/fastsac_g1_29dof.onnx"
onnx_sha256="8346fd90778439395922a8c7256f24125ae84b8dea949128bac9e23c02bc7717"
dest="${1:-${HIM_HOLOSOMA_DIR:-/home/unitree/holosoma}}"

if [[ -d "$dest/.git" ]]; then
  git -C "$dest" fetch --depth 1 origin "$commit"
else
  mkdir -p "$dest"
  git -C "$dest" init -q
  git -C "$dest" remote add origin "$url"
  git -C "$dest" fetch --depth 1 origin "$commit"
fi
git -C "$dest" -c advice.detachedHead=false checkout -q "$commit"

echo "$onnx_sha256  $dest/$onnx" | sha256sum -c --quiet -
echo "Holosoma $commit at $dest"
echo "FastSAC weights: $dest/$onnx"
