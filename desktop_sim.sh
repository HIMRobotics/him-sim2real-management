#!/usr/bin/env bash
# Desktop entry: Compose for ROS submissions, host Python for direct modes.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
G1_SIM_BACKEND=compose exec "$repo_dir/sim.sh" "$@"
