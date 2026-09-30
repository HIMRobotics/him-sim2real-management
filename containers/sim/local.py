"""Run the example ONNX controllers directly against the shared MuJoCo runner."""

import argparse
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [
    str(ROOT),
    str(ROOT / "robot_safety"),
    str(ROOT / "packages/policy_safety/src"),
    str(ROOT / "containers/examples"),
]

from controllers import ExamplePolicy
from plant import CONTROL_DT, POSES
from prepare import prepare
from runner import Runner


def run(name, output, seconds=10.0, safety_checks=True, scene=None, pose="standing"):
    """Run real inference and physics with simulated control timestamps."""
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("seconds must be finite and positive")
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    for filename in ("report.json", "trace.npz"):
        (output / filename).unlink(missing_ok=True)
    project = prepare(name)
    scene = scene or project / "environment/scene.xml"
    policy = ExamplePolicy(name, project / "container/project")
    runner = Runner(pose, scene=scene, safety_checks=safety_checks)
    for step in range(math.ceil(seconds / CONTROL_DT)):
        time = step * CONTROL_DT
        if runner.reason is None:
            state = runner.feedback(time)
            runner.command(policy.command(state), time)
        runner.step(time)
        if runner.reason == "nonfinite_simulation":
            break
    previous_trace = os.environ.get("G1_SIM_TRACE")
    os.environ["G1_SIM_TRACE"] = str(output / "trace.npz")
    try:
        report = runner.report()
    finally:
        if previous_trace is None:
            os.environ.pop("G1_SIM_TRACE", None)
        else:
            os.environ["G1_SIM_TRACE"] = previous_trace
    report.update(
        execution="local",
        policy=name,
        requested_seconds=seconds,
        policy_calls=policy.sequence,
    )
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--example", required=True, choices=("stairs", "walker", "balance")
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--pose", choices=POSES, default="standing")
    parser.add_argument("--no-safety-checks", action="store_true")
    args = parser.parse_args()
    try:
        report = run(
            args.example,
            args.output,
            args.seconds,
            not args.no_safety_checks,
            args.scene,
            args.pose,
        )
    except Exception as exc:  # noqa: BLE001
        report = {
            "version": 1,
            "status": "error",
            "reason": "execution_error",
            "detail": str(exc),
            "execution": "local",
            "policy": args.example,
            "safety_checks": not args.no_safety_checks,
            "hardware_authorization": False,
        }
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return {"within_limits": 0, "completed": 0, "violation": 1}.get(report["status"], 2)


if __name__ == "__main__":
    sys.exit(main())
