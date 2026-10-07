"""Stdio front end so the ROS bridge can run in a different Python."""

import json
import os
import sys

from runner import Runner


def main():
    safety_checks = os.environ.get("G1_SIM_SAFETY_CHECKS", "1")
    if safety_checks not in ("0", "1"):
        raise ValueError("G1_SIM_SAFETY_CHECKS must be 0 or 1")
    runner = Runner(
        os.environ.get("G1_SIM_POSE", "standing"),
        scene=os.environ.get("G1_SIM_SCENE") or None,
        safety_checks=safety_checks == "1",
    )
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
