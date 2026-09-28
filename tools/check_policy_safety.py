#!/usr/bin/env python3
"""Run the portable offline checker from this repository."""

import sys
from pathlib import Path

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "packages/policy_safety/src")
)

from policy_safety.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
