#!/usr/bin/env bash
# Unified policy submission entry point for simulation and hardware.
# Hardware startup holds a stand until measured stability, then waits.
# Left and B together launch the policy. Double Start requests stock damping.
# Startup defaults to GR00T Balance. Double F3 runs Holosoma standing control.
# --startup holosoma explicitly selects Holosoma startup.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" || $# -eq 0 ]]; then
  cat <<'HELP'
Usage: ./submit.sh --mode sim [sim.sh options]
       ./submit.sh --mode real [--startup groot|holosoma]
                   [--image NAME | --example NAME] [--command FILE]
                   [--profile PATH] [--] [container args...]
       ./submit.sh --mode real stop

Simulation: --example walker --local --seconds 10 --view
Hardware startup defaults to GR00T Balance from the ready stand.
Left + B launches the container policy after measured stability.
--startup holosoma selects Holosoma startup and also accepts damping mode.
Double Start damps. Double F3 runs Holosoma standing control.
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
startup="${G1_STARTUP_POLICY:-groot}"
example=""
kept=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --startup)
      startup="${2:?--startup needs groot or holosoma}"
      shift 2
      ;;
    --example)
      example="${2:?--example needs a name}"
      kept+=(--example "$example")
      shift 2
      ;;
    *)
      kept+=("$1")
      shift
      ;;
  esac
done
case "$startup" in
  groot|holosoma) ;;
  *) echo "--startup must be groot or holosoma." >&2; exit 2 ;;
esac
export G1_STARTUP_POLICY="$startup"
if [[ "${kept[0]:-}" == stop ]]; then
  exec "$repo_dir/setup.sh" stop "${kept[@]:1}"
fi
exec "$repo_dir/setup.sh" --mode hardware --enable-hardware-output "${kept[@]}"
