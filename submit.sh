#!/usr/bin/env bash
# Unified policy submission entry point for simulation and hardware.
# The host runs GR00T Balance until measured stability, then waits.
# Left and B together launch the policy. Double Start requests stock damping.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" || $# -eq 0 ]]; then
  cat <<'HELP'
Usage: ./submit.sh --mode sim [sim.sh options]
       ./submit.sh --mode real [--image NAME | --example NAME] [--command FILE]
                   [--profile PATH] [--] [container args...]
       ./submit.sh --mode real stop

Simulation: --example walker --local --seconds 10 --view
Hardware: GR00T Balance stabilizes first, then left + B authorizes handoff.
Double Start damps.
Use ./sim.sh --help for all simulation options.
HELP
  exit 0
fi
if [[ "${1:-}" != --mode || $# -lt 2 ]]; then
  echo "Select --mode sim or --mode real." >&2
  exit 2
fi
mode="$2"
shift 2
case "$mode" in
  sim) exec "$repo_dir/sim.sh" "$@" ;;
  real|hardware) ;;
  *) echo "Select --mode sim or --mode real." >&2; exit 2 ;;
esac
if [[ "${1:-}" == stop ]]; then
  shift
  exec "$repo_dir/setup.sh" stop "$@"
fi
exec "$repo_dir/setup.sh" --mode hardware --enable-hardware-output "$@"
