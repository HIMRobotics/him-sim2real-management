#!/usr/bin/env bash
# Unified policy submission entry point for simulation and hardware.
# Hardware startup holds a stand until measured stability, then waits.
# Left and B together launch the policy. Double Start requests stock damping.
# Startup is Holosoma: developer mode, then the same standing policy as the
# original A button. Left + B launches the container policy.
# --startup sonic or --startup groot selects the previous startup controllers.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" || $# -eq 0 ]]; then
  cat <<'HELP'
Usage: ./submit.sh --mode sim [sim.sh options]
       ./submit.sh --mode real [--startup sonic|groot|holosoma]
                   [--image NAME | --example NAME] [--command FILE]
                   [--profile PATH] [--] [container args...]
       ./submit.sh --mode real stop

Simulation: --example walker --local --seconds 10 --view
Hardware startup is Holosoma. Developer mode, then the same standing policy
as the original A button, through the safety gate. Damp or the ready stand is accepted.
Left + B launches the container policy. --startup sonic or --startup groot
selects the previous startup controllers.
Double Start damps. Double F3 stays on GR00T Balance.
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
startup="${G1_STARTUP_POLICY:-holosoma}"
example=""
kept=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --startup)
      startup="${2:?--startup needs sonic, groot, or holosoma}"
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
if [[ "$example" == holosoma ]]; then
  startup=holosoma
fi
case "$startup" in
  sonic|groot|holosoma) ;;
  *) echo "--startup must be sonic, groot, or holosoma." >&2; exit 2 ;;
esac
export G1_STARTUP_POLICY="$startup"
export HIM_SONIC_DIR="${HIM_SONIC_DIR:-$repo_dir/.cache/sonic}"
if [[ "$startup" == sonic && "${kept[0]:-}" != stop ]]; then
  if [[ ! -f "$HIM_SONIC_DIR/model_encoder.onnx" || ! -f "$HIM_SONIC_DIR/model_decoder.onnx" ]]; then
    echo "SONIC startup weights are not in $HIM_SONIC_DIR." >&2
    echo "Download model_encoder.onnx and model_decoder.onnx from https://huggingface.co/nvidia/GEAR-SONIC" >&2
    echo "Undo with: ./submit.sh --mode real --startup groot ..." >&2
    exit 2
  fi
fi
if [[ "${kept[0]:-}" == stop ]]; then
  exec "$repo_dir/setup.sh" stop "${kept[@]:1}"
fi
exec "$repo_dir/setup.sh" --mode hardware --enable-hardware-output "${kept[@]}"
