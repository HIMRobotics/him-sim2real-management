"""Stdio front end so the ROS bridge can run in a different Python."""

import json
import os
import sys

from runner import Runner


def main():
    runner = Runner(os.environ.get("G1_SIM_POSE", "standing"))
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            request = json.loads(line)
            response = runner.handle(request)
        except Exception as exc:  # noqa: BLE001
            response = {"ok": False, "reason": "plant_error", "detail": str(exc)}
        sys.stdout.write(json.dumps(response) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
