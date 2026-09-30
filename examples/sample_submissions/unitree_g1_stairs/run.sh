#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

if [[ ! -d .venv ]]; then
  python3 -m venv .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install -q --upgrade pip
python -m pip install -q -r requirements.txt

echo "==> Policy contract tests"
python -m pytest -q tests/test_policy_contract.py

echo "==> Causality tests"
python -m pytest -q tests/test_causality.py -s

echo "==> Generate result.mp4 from successful episode"
python run.py --seed 0 --horizon 20 --video "$ROOT/result.mp4"

echo "==> Inspect result.mp4"
python - <<'PY'
import json
from pathlib import Path
import imageio.v2 as imageio
p = Path("result.mp4")
r = imageio.get_reader(p)
meta = r.get_meta_data()
n = r.count_frames()
# sample a few frames
idxs = sorted(set([0, n//4, n//2, (3*n)//4, max(n-1,0)]))
for i in idxs:
    frame = r.get_data(i)
    print(f"frame[{i}] shape={frame.shape} mean_rgb={frame.mean(axis=(0,1)).round(1)}")
r.close()
fps = float(meta.get("fps", 25))
print(json.dumps({
    "path": str(p.resolve()),
    "size_bytes": p.stat().st_size,
    "frames": n,
    "fps": fps,
    "duration_s": n / fps if fps else None,
    "size": meta.get("size"),
}, indent=2))
PY

echo "DONE"
