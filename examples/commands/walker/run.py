"""Run NVIDIA's walker in MuJoCo with a configurable velocity command."""

import json
import os
import subprocess
import sys
from pathlib import Path

forward_velocity = 0.3  # m/s
sideways_velocity = 0.0
yaw_rate = 0.0  # rad/s

if __name__ == "__main__":
    root = Path(__file__).resolve().parents[3]
    env = dict(
        os.environ,
        G1_TASK_COMMAND=json.dumps([forward_velocity, sideways_velocity, yaw_rate]),
    )
    raise SystemExit(
        subprocess.call(
            [
                str(root / "submit.sh"),
                "--mode",
                "sim",
                "--example",
                "walker",
                "--local",
                *sys.argv[1:],
            ],
            env=env,
        )
    )
