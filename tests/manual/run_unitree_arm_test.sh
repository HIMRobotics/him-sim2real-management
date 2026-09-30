#!/usr/bin/env bash
# Run the SDK arm sequence with diagnostic output or the existing systemd gate.
set -eo pipefail
manual_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--start" || "${1:-}" == "--check-ready" || "${1:-}" == "--joint-deltas" ]]; then
    exec "$manual_dir/run_arm_raise_dry_run.sh" "$@"
fi
test_policy=unitree_arm7
if [[ "${1:-}" == "--small-step" ]]; then
    test_policy=small_elbow
    shift
fi
if [[ "${1:-}" == "--hardware" ]]; then
    shift
    exec "$manual_dir/run_arm_raise_dry_run.sh" "test_policy:=$test_policy" \
        enable_hardware_output:=true "$@"
fi
exec "$manual_dir/run_arm_raise_dry_run.sh" "test_policy:=$test_policy" "$@"
