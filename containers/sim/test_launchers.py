"""Launcher routing, output isolation, and failure cleanup."""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "containers/sim"), str(ROOT / "robot_safety"),
                str(ROOT / "packages/policy_safety/src")]
from runner import LAUNCH_SECONDS  # noqa: E402


@pytest.mark.parametrize("entry", ["sim.sh", "desktop_sim.sh"])
@pytest.mark.parametrize("example,mode,pose", [
    ("walker", "off", "standing"),
    ("walker/thrash", "enforce", "supine"),
    ("walker/thrash-monitor", "monitor", "supine"),
])
def test_local_example_without_docker_or_live_writes(tmp_path, entry, example, mode, pose):
    result = subprocess.run(
        [str(ROOT / entry), "--local", "--example", example, "--seconds", "0.1",
         "--output", str(tmp_path)] + (["--no-safety-checks"] if example == "walker" else []),
        env=dict(os.environ, G1_SIM_PYTHON=sys.executable, CONTAINER_RUNTIME="missing-docker",
                 G1_SIM_THRASH_MODE=""),
        capture_output=True, text=True, timeout=60, check=False,
    )
    # Every run starts with the launch period. A supine robot thrashes in it.
    assert result.returncode == (1 if mode == "enforce" else 0), result.stderr
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["execution"] == "local"
    assert report["policy"] == "walker"
    assert report["control_path"] == "submission"
    assert report["pose"] == pose
    assert not report["safety_checks"]
    assert not report["checks"]["base_angular_speed"]
    assert report["thrash"]["mode"] == mode
    assert report["checks"]["thrash_monitoring"] == (mode != "off")
    assert (tmp_path / "trace.npz").is_file()
    assert not (tmp_path / "live.jsonl").exists()


@pytest.mark.parametrize("example,counterfactuals", [
    (None, False), (None, True), ("walker/thrash", False), ("walker/thrash-monitor", False),
])
def test_compose_failure_cleanup_and_pose_outputs(tmp_path, example, counterfactuals):
    runtime = tmp_path / "docker"
    log = tmp_path / "calls.jsonl"
    runtime.write_text(f"#!{sys.executable}\n" + '''
import json, os, sys
from pathlib import Path
args = sys.argv[1:]
with open(os.environ["CALL_LOG"], "a") as log:
    log.write(json.dumps(args) + "\\n")
if args[-3:] == ["-d", "net", "sim"]:
    output = Path(os.environ["G1_SIM_OUTPUT"])
    (output / "ready").touch()
    (output / "report.json").write_text(json.dumps({
        "pose": os.environ["G1_SIM_POSE"], "status": "violation",
        "safety_checks": os.environ["G1_SIM_SAFETY_CHECKS"] == "1", "reason": "test rejection",
        "policy": os.environ["G1_SIM_EXAMPLE"],
        "thrash_mode": os.environ.get("G1_SIM_THRASH_MODE"),
    }))
if args[-2:] == ["wait", "sim"]:
    sys.exit(1)
''')
    runtime.chmod(0o755)
    args = [str(ROOT / "desktop_sim.sh"), "--output", str(tmp_path)]
    args += ["--example", example] if example else ["--image", "test:local"]
    if counterfactuals:
        args.append("--counterfactuals")
    result = subprocess.run(
        args, env=dict(os.environ, G1_SIM_PYTHON=sys.executable,
                       CONTAINER_RUNTIME=str(runtime), CALL_LOG=str(log)),
        capture_output=True, text=True, timeout=60, check=False,
    )
    assert result.returncode == 1, result.stderr
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert calls[-1][-2:] == ["down", "--remove-orphans"]
    assert sum(call[-2:] == ["build", "sim"] for call in calls) == 1
    reports = list(tmp_path.glob("poses/*/report.json")) if counterfactuals else [tmp_path / "report.json"]
    assert len(reports) == (6 if counterfactuals else 1)
    assert len({json.loads(path.read_text())["pose"] for path in reports}) == len(reports)
    if example:
        report = json.loads(reports[0].read_text())
        assert report["policy"] == "walker"
        assert report["pose"] == "supine"
        assert not report["safety_checks"]
        assert report["thrash_mode"] == ("monitor" if example.endswith("-monitor") else "enforce")


@pytest.mark.parametrize("overrides", [
    ["--pose", "standing", "--safety-checks"],
    ["--safety-checks", "--pose", "standing"],
])
def test_scenario_explicit_overrides(tmp_path, overrides):
    result = subprocess.run(
        [str(ROOT / "sim.sh"), *overrides, "--example", "walker/thrash", "--local",
         "--seconds", "0.02", "--output", str(tmp_path)],
        env=dict(os.environ, G1_SIM_PYTHON=sys.executable, G1_SIM_THRASH_MODE="off"),
        capture_output=True, text=True, timeout=60, check=False,
    )
    assert result.returncode == 0, result.stderr
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["pose"] == "standing"
    assert report["safety_checks"]
    assert report["thrash"]["mode"] == "enforce"


@pytest.mark.parametrize("args", [
    ["--example", "walker/unknown"],
    ["--thrash-mode", "enforce"],
])
def test_invalid_selection_fails_before_starting(tmp_path, args):
    result = subprocess.run(
        [str(ROOT / "sim.sh"), *args, "--output", str(tmp_path)],
        env=dict(os.environ, G1_SIM_PYTHON=sys.executable, CONTAINER_RUNTIME="missing-docker"),
        capture_output=True, text=True, timeout=30, check=False,
    )
    assert result.returncode != 0
    assert not (tmp_path / "report.json").exists()
    assert "Docker" not in result.stderr


@pytest.mark.parametrize("scenario,enforce", [("thrash", True), ("thrash-monitor", False)])
def test_walker_thrash_scenario_rollout(tmp_path, scenario, enforce):
    result = subprocess.run(
        [str(ROOT / "sim.sh"), "--example", "walker/" + scenario, "--local",
         "--seconds", "3", "--output", str(tmp_path)],
        env=dict(os.environ, G1_SIM_PYTHON=sys.executable),
        capture_output=True, text=True, timeout=60, check=False,
    )
    assert result.returncode == (1 if enforce else 0), result.stderr
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["control_path"] == "submission"
    assert report["thrash"]["event"]["reason"].startswith("thrashing_")
    if enforce:
        assert report["reason"] == report["thrash"]["event"]["reason"]
        assert report["stop_time_seconds"] < 3
    else:
        assert report["reason"] is None
        assert report["commands"] == round((LAUNCH_SECONDS + 3) / 0.02)
