"""Portable CLI and bounded evaluation worker."""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

from .adapter import ADAPTERS
from .config import SUMMARY_COLUMNS, atomic_report, positive
from .joint_rules import DEFAULT_JOINT_PROFILE


def default_video_dir():
    """Repo tmp/vids when this file lives in the checkout, else ./tmp/vids."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "packages" / "policy_safety").is_dir():
            return parent / "tmp" / "vids"
    return Path.cwd() / "tmp" / "vids"


def failure_report(output, reason):
    path = output / "report.json"
    try:
        report = json.loads(path.read_text())
    except (OSError, ValueError):
        report = {"version": 1, "scenarios": [], "hardware_authorization": False}
    report.update(status="incomplete", exit_code=2, worker_error=reason)
    if "coverage" in report:
        report["coverage"]["unreported"] = (
            report["coverage"]["planned"] - report["coverage"]["reported"]
        )
    atomic_report(path, report)
    return report


def _render_progress(message):
    """Redraw one terminal line, or emit a plain line for redirected output."""
    if sys.stdout.isatty():
        width = max(1, shutil.get_terminal_size().columns - 1)
        print("\r\033[K" + message[:width], end="", flush=True)
    else:
        print(message, flush=True)


def print_scenario_summary(summary):
    """Show problems and overlapping issue counts."""
    if not summary:
        return
    headers = SUMMARY_COLUMNS

    def line(label, values):
        cells = [
            f"{value:>{max(8, len(header))}}" for value, header in zip(values, headers)
        ]
        print(f"{label:<18} " + cells[0] + " | " + " ".join(cells[1:]))

    line("scenario", headers)
    for row in summary:
        line(
            row["scenario"],
            [f"{row.get(key, 0)}/{row['total']}" for key in SUMMARY_COLUMNS],
        )


def print_progress(output, elapsed):
    """Show completed coverage and the active scenario without blocking the worker."""
    try:
        report = json.loads((output / "report.json").read_text())
    except (OSError, ValueError):
        _render_progress(f"[{elapsed:.0f}s] Initializing model and simulator...")
        return
    coverage = report.get("coverage", {})
    active = report.get("active_scenario")
    detail = ""
    if active:
        detail = f" | running {active['kind']}/{active['name']} seed={active['seed']}"
        if active.get("checkpoint_step") is not None:
            detail += f" checkpoint={active['checkpoint_step']}"
        if active.get("push"):
            push = active["push"]
            detail += f" push={push.get('label', push['body'])} {push['force']} N"
    done, total = coverage.get("reported", 0), coverage.get("planned", 0)
    filled = min(10, int(10 * done / total)) if total else 0
    bar = "[" + "#" * filled + "-" * (10 - filled) + "]"
    calibration = (
        ""
        if report.get("profile", {}).get("calibrated")
        and not report.get("joint_movement_checks", {}).get("uncovered_joints")
        else " UNCALIBRATED"
    )
    _render_progress(
        f"[{elapsed:.0f}s] {bar} {done}/{total or '?'}"
        f" violations={coverage.get('violation', 0)} errors={coverage.get('error', 0)}"
        f" incomplete={coverage.get('incomplete', 0)}{calibration}{detail}"
    )


def run_worker(request, timeout):
    """Run one evaluation subprocess so the CLI can enforce a timeout."""
    output = Path(request["output"])
    request_path = output / "request.json"
    atomic_report(request_path, request)
    env = os.environ.copy()
    # The source-tree wrapper must also work without an editable installation.
    source_root = str(Path(__file__).resolve().parent.parent)
    env["PYTHONPATH"] = source_root + os.pathsep + env.get("PYTHONPATH", "")
    import joint_checks

    env["PYTHONPATH"] += os.pathsep + str(Path(joint_checks.__file__).parent)
    started = time.monotonic()
    print(f"Starting evaluation | timeout={timeout:g}s | output={output}", flush=True)
    print(
        "Progress counts tested scenarios, not safety approval. Joint-rule coverage and global threshold gaps are recorded in report.json.",
        flush=True,
    )
    with (output / "worker.log").open("w") as log:
        process = subprocess.Popen(
            [sys.executable, "-m", "policy_safety.worker", str(request_path)],
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
        )
        try:
            print_progress(output, time.monotonic() - started)
            while True:
                remaining = timeout - (time.monotonic() - started)
                if remaining <= 0:
                    raise subprocess.TimeoutExpired(process.args, timeout)
                try:
                    interval = 1.0 if sys.stdout.isatty() else 5.0
                    return_code = process.wait(timeout=min(interval, remaining))
                    print_progress(output, time.monotonic() - started)
                    break
                except subprocess.TimeoutExpired:
                    print_progress(output, time.monotonic() - started)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            return failure_report(output, "wall_clock_timeout")
        except BaseException:
            process.kill()
            process.wait()
            failure_report(output, "evaluation_interrupted")
            raise
        finally:
            if sys.stdout.isatty():
                print(flush=True)
    try:
        report = json.loads((output / "report.json").read_text())
    except (OSError, ValueError):
        return failure_report(output, f"worker_exit_{return_code}_without_report")
    if report.get("status") == "running" or return_code != report.get("exit_code"):
        return failure_report(output, f"unexpected_worker_exit_{return_code}")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prepare = commands.add_parser(
        "prepare-g1", help="copy the known example into a portable test bundle"
    )
    prepare.add_argument("--submission", required=True, type=Path)
    prepare.add_argument("--output", required=True, type=Path)
    evaluate = commands.add_parser(
        "evaluate", help="screen an explicitly adapted neural policy offline"
    )
    render = commands.add_parser("render", help="write videos for selected scenarios")
    for command in (evaluate, render):
        command.add_argument("--model", required=True, type=Path)
        command.add_argument("--adapter", required=True, choices=ADAPTERS)
        command.add_argument("--robot-config", required=True, type=Path)
        command.add_argument(
            "--joint-profile",
            type=Path,
            default=DEFAULT_JOINT_PROFILE,
            help="per-joint movement rules, defaults to the standard 29-joint template",
        )
        command.add_argument(
            "--profile",
            type=Path,
            default=Path(__file__).parent / "profiles/provisional.json",
        )
    evaluate.add_argument("--suite", type=Path)
    evaluate.add_argument("--output", required=True, type=Path)
    evaluate.add_argument(
        "--timeout",
        type=float,
        default=1800.0,
        help="maximum worker wall time in seconds",
    )
    render.add_argument(
        "--output",
        type=Path,
        default=default_video_dir(),
        help="directory for mp4 clips, reused if it already exists",
    )
    render.add_argument(
        "--scenarios",
        default="stand,forward,seated,forward-pelvis-push",
        help="comma-separated commands, poses, forward-pelvis-push-small/medium/heavy, or stand-95/105",
    )
    render.add_argument("--fps", type=float, default=20.0)
    args = parser.parse_args(argv)
    try:
        if args.command == "prepare-g1":
            from .bundle import prepare_g1

            print(prepare_g1(args.submission, args.output))
            return 0
        rendering = args.command == "render"
        positive(
            args.fps if rendering else args.timeout, "fps" if rendering else "timeout"
        )
        output = args.output.resolve()
        output.mkdir(parents=True, exist_ok=rendering)
        request = {
            name: str(getattr(args, name).resolve())
            for name in ("model", "robot_config", "profile", "joint_profile")
        }
        request.update(adapter=args.adapter, output=str(output))
        if rendering:
            os.environ.setdefault("MUJOCO_GL", "egl")
            from .evaluation import summarize_scenarios
            from .render import render_selection

            names = [name.strip() for name in args.scenarios.split(",") if name.strip()]
            if not names:
                raise ValueError("select at least one scenario")
            _written, results = render_selection(request, names, args.fps)
            print_scenario_summary(summarize_scenarios(results))
            print(f"Videos: {output}")
            return 0
        request["suite"] = str(args.suite.resolve()) if args.suite else None
        report = run_worker(request, args.timeout)
        coverage = report.get("coverage", {})
        movement = report.get("joint_movement_checks")
        if movement:
            print(
                f"Joint rules: {movement['profile_file']} | covered={len(movement['covered_joints'])} | uncovered={len(movement['uncovered_joints'])}"
            )
        print(
            f"{report['status']}: exit {report['exit_code']}, "
            f"coverage {coverage}"
        )
        print_scenario_summary(report.get("scenario_summary", []))
        failures = [
            s
            for s in report.get("scenarios", [])
            if s["status"] in ("violation", "error")
        ]
        for scenario in failures[:5]:
            detail = (scenario.get("first_violation") or {}).get(
                "reason", scenario.get("error", "")
            )
            violation = scenario.get("first_violation") or {}
            if "joint" in violation:
                detail += f" joint={violation['joint']} value={violation['value']:.6g} limit={violation['limit']} {violation['units']}"
            print(
                f"  {scenario['kind']}/{scenario['name']} seed={scenario['seed']}: {detail}"
            )
        if report.get("worker_error"):
            print(f"  Worker: {report['worker_error']}")
        print(f"Report: {output / 'report.json'}")
        return report["exit_code"]
    except (OSError, ValueError, KeyError) as exc:
        print(f"policy-safety: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
