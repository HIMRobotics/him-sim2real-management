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

from command import load_schedule
from controllers import ExamplePolicy
from plant import CONTROL_DT, POSES
from prepare import prepare
from runner import LAUNCH_SECONDS, Runner
from scenarios import CASES, SafetyScenario


def _prepare_output(output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    for filename in ("report.json", "trace.npz"):
        (output / filename).unlink(missing_ok=True)
    return output


def _finish(runner, output, extra):
    previous_trace = os.environ.get("G1_SIM_TRACE")
    os.environ["G1_SIM_TRACE"] = str(output / "trace.npz")
    try:
        report = runner.report()
    finally:
        if previous_trace is None:
            os.environ.pop("G1_SIM_TRACE", None)
        else:
            os.environ["G1_SIM_TRACE"] = previous_trace
    report.update(execution="local", **extra)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def _rollout(runner, seconds, command):
    for step in range(math.ceil((LAUNCH_SECONDS + seconds) / CONTROL_DT)):
        time = step * CONTROL_DT
        if runner.reason is None:
            command(runner, time, step)
        else:
            runner.step(time)
        if runner.reason == "nonfinite_simulation":
            break


def run(name, output, seconds=10.0, safety_checks=True, scene=None, pose="standing",
        *, scenario=None, tilt_degrees=20):
    """Run real inference and physics with simulated control timestamps."""
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("seconds must be finite and positive")
    if scenario and not safety_checks:
        raise ValueError("safety scenarios require safety checks")
    scenario = SafetyScenario(scenario, tilt_degrees) if scenario else None
    output = _prepare_output(output)
    project = prepare(name, output / "prepared", container=False)
    scene = scene or project / "environment/scene.xml"
    policy = ExamplePolicy(name, project / "container/project")
    # Score and apply this example's gains. The generic template is stiffer
    # and reports a torque the policy never commanded.
    previous_profile = os.environ.get("G1_JOINT_PROFILE")
    os.environ["G1_JOINT_PROFILE"] = str(project / "container/project/joint_profile.json")
    try:
        runner = Runner(pose, scene=scene, safety_checks=safety_checks, record_trace=True)
        task_command = json.loads(os.environ.get("G1_TASK_COMMAND") or "null")
        command_file = os.environ.get("G1_COMMAND_FILE") or ""
        schedule = load_schedule(command_file) if command_file else None

        command_started = None

        def command(current, time, _step):
            nonlocal command_started
            if scenario:
                scenario.apply(current)
            state = current.feedback(time)
            if state["control_ready"] and command_started is None:
                command_started = state["policy_handoff_at_secs"]
            if schedule is not None:
                elapsed = 0.0 if command_started is None else time - command_started
                state["task_command"] = schedule.at(elapsed).tolist()
            elif task_command is not None:
                state["task_command"] = task_command
            current.command(policy.command(state), time)
            current.step(time)

        _rollout(runner, seconds, command)
    finally:
        if previous_profile is None:
            os.environ.pop("G1_JOINT_PROFILE", None)
        else:
            os.environ["G1_JOINT_PROFILE"] = previous_profile
    return _finish(
        runner,
        output,
        {
            "scenario": scenario.report(runner) if scenario else None,
            "policy": name,
            "requested_seconds": seconds,
            "policy_calls": policy.sequence,
            "task_command": schedule.at(0.0).tolist() if schedule is not None else task_command,
            "command_file": command_file or None,
        },
    )


def hold(output, seconds=1.0, safety_checks=True, scene=None, pose="standing"):
    """Hold the measured pose. Desktop equivalent of ./sim.sh --self-test."""
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("seconds must be finite and positive")
    output = _prepare_output(output)
    runner = Runner(pose, scene=scene, safety_checks=safety_checks, record_trace=True)

    def command(current, time, step):
        state = current.feedback(time)
        current.command(
            {
                "session_id": current.session_id,
                "sequence": step,
                "observation_id": state["observation_id"],
                "q": list(state["q"]),
                "dq": [0.0] * 29,
                "kp": [40.0] * 29,
                "kd": [2.0] * 29,
                "tau": [0.0] * 29,
            },
            time,
        )
        current.step(time)

    _rollout(runner, seconds, command)
    return _finish(
        runner,
        output,
        {"policy": None, "requested_seconds": seconds, "mode": "self_test"},
    )


def circle(output, seconds=10.0, safety_checks=True, scene=None, pose="standing"):
    """Walk the cached G1 walker with simulated timestamps."""
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("seconds must be finite and positive")
    output = _prepare_output(output)
    runner = Runner(pose, scene=scene, safety_checks=safety_checks, record_trace=True,
                    control_path="direct")

    def command(current, time, _step):
        current.circle(time)

    _rollout(runner, seconds, command)
    return _finish(
        runner,
        output,
        {
            "policy": Path(os.environ.get("G1_SIM_COMMAND") or "circle").stem or "circle",
            "requested_seconds": seconds,
            "mode": "circle",
        },
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--example", choices=("walker", "balance", "holosoma"))
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--circle", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--pose", choices=POSES, default="standing")
    parser.add_argument("--no-safety-checks", action="store_true")
    parser.add_argument("--scenario", choices=CASES)
    parser.add_argument("--tilt-degrees", type=float, default=20)
    args = parser.parse_args()
    if args.scenario and not args.example:
        parser.error("--scenario requires --example")
    policy = args.example or ("circle" if args.circle else None)
    try:
        if args.self_test:
            report = hold(
                args.output,
                args.seconds,
                not args.no_safety_checks,
                args.scene,
                args.pose,
            )
        elif args.circle:
            report = circle(
                args.output,
                args.seconds,
                not args.no_safety_checks,
                args.scene,
                args.pose,
            )
        else:
            report = run(
                args.example,
                args.output,
                args.seconds,
                not args.no_safety_checks,
                args.scene,
                args.pose,
                scenario=args.scenario, tilt_degrees=args.tilt_degrees,
            )
    except Exception as exc:  # noqa: BLE001
        report = {
            "version": 1,
            "status": "error",
            "reason": "execution_error",
            "detail": str(exc),
            "execution": "local",
            "policy": policy,
            "safety_checks": not args.no_safety_checks,
            "hardware_authorization": False,
        }
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if report.get("scenario"):
        return 0 if report["scenario"]["passed"] else 1
    return {"within_limits": 0, "completed": 0, "violation": 1}.get(report["status"], 2)


if __name__ == "__main__":
    sys.exit(main())
