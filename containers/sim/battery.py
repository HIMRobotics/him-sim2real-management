"""Headless flat-ground battery: each pose, then the pelvis pushes.

Uses the container plant and the same joint checks as ./sim.sh. No viewer
and no submission container. Push strengths and timing come from the offline
suite: 10 N, 50 N, and 150 N for 100 ms, starting at 1 s.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [
    str(ROOT / "packages/policy_safety/src"),
    str(ROOT / "robot_safety"),
    str(Path(__file__).resolve().parent),
]

from policy_safety.config import DEFAULT_SUITE
from plant import CONTROL_DT, POSES
from runner import Runner


def pushes():
    """Forward pelvis shoves at each suite strength."""
    start = float(DEFAULT_SUITE["push_start_seconds"])
    duration = float(DEFAULT_SUITE["push_seconds"])
    cases = [{"name": "none", "push": None}]
    for level in DEFAULT_SUITE["push_levels"]:
        cases.append(
            {
                "name": level["name"],
                "push": {
                    "body": "pelvis",
                    "force": [float(level["force_n"]), 0.0, 0.0],
                    "start": start,
                    "duration": duration,
                },
            }
        )
    return cases


def hold(runner, state, sequence, q):
    return {
        "session_id": runner.session_id,
        "sequence": sequence,
        "observation_id": state["observation_id"],
        "q": [float(value) for value in q],
        "dq": [0.0] * 29,
        "kp": [40.0] * 29,
        "kd": [2.0] * 29,
        "tau": [0.0] * 29,
    }


def run_case(pose, push, seconds, safety_checks=True, circle=False):
    """Hold the settled pose, or walk the circle gait, and apply one push."""
    runner = Runner("standing" if circle else pose, safety_checks=safety_checks)
    runner.plant.set_push(push)
    q = None if circle else runner.plant.q().copy()
    steps = int(np.ceil(seconds / CONTROL_DT))
    for step in range(steps):
        time = step * CONTROL_DT
        if circle:
            runner.circle(time)
            continue
        state = runner.feedback(time)
        runner.command(hold(runner, state, step, q), time)
        runner.step(time)
    report = runner.report()
    report["push"] = None if push is None else {
        "body": push["body"],
        "force_n": float(push["force"][0]),
        "start": push["start"],
        "duration": push["duration"],
    }
    return report


def run_battery(
    output, seconds=2.0, safety_checks=True, poses=POSES, push=None, circle=False
):
    """Flat-floor cases. One pose and one push name runs that case only."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    selected = pushes()
    if push is not None:
        selected = [case for case in selected if case["name"] == push]
        if not selected:
            names = ", ".join(case["name"] for case in pushes())
            raise SystemExit(f"--push must be one of: {names}")
    if circle:
        poses = ("standing",)
    if any(name not in POSES for name in poses):
        raise SystemExit(f"--pose must be one of: {', '.join(POSES)}")
    minimum = (
        float(DEFAULT_SUITE["push_start_seconds"])
        + float(DEFAULT_SUITE["push_seconds"])
        + 0.5
    )
    seconds = max(float(seconds), minimum)
    cases = []
    for pose in poses:
        for case in selected:
            print(f"{pose} {case['name']}", flush=True)
            report = run_case(
                pose, case["push"], seconds, safety_checks, circle=circle
            )
            cases.append(
                {
                    "pose": pose,
                    "push": case["name"],
                    "status": report["status"],
                    "reason": report["reason"],
                    "detail": report["detail"],
                    "fell": report["motion"]["fell"],
                    "upright_at_end": report["motion"]["upright_at_end"],
                    "final_position": report["motion"]["final_position"],
                    "seconds": report["seconds"],
                    "motion": report["motion"],
                }
            )
    summary = {
        "scene": "flat",
        "policy": "circle" if circle else "battery",
        "safety_checks": bool(safety_checks),
        "seconds": seconds,
        "cases": cases,
    }
    if len(cases) == 1:
        summary.update(
            status=cases[0]["status"],
            reason=cases[0]["reason"],
            detail=cases[0]["detail"],
            motion=cases[0]["motion"],
            pose=cases[0]["pose"],
        )
    path = output / "report.json"
    path.write_text(json.dumps(summary, indent=2) + "\n")
    print(path)
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--pose", action="append")
    parser.add_argument("--push", default=None)
    parser.add_argument("--circle", action="store_true")
    parser.add_argument("--no-safety-checks", action="store_true")
    args = parser.parse_args()
    run_battery(
        args.output,
        args.seconds,
        safety_checks=not args.no_safety_checks,
        poses=args.pose or POSES,
        push=args.push,
        circle=args.circle,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
