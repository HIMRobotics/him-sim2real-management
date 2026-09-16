#!/usr/bin/env bash
set -eu
KIT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
exec "${PYTHON:-python3}" "$KIT_DIR/app.py" "$@"
