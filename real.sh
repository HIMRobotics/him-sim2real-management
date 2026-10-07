#!/usr/bin/env bash
# Compatibility entry point for hardware policy submission.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$repo_dir/submit.sh" --mode real "$@"
