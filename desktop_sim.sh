#!/usr/bin/env bash
# For ROS submissions, runs ROS and MuJoCo in Docker Compose containers instead of on the Linux host as sim.sh does.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
G1_SIM_BACKEND=compose exec "$repo_dir/sim.sh" "$@"
