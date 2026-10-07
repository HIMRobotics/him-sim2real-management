"""Launcher routing, output isolation, and failure cleanup."""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("entry", ["sim.sh", "desktop_sim.sh"])
@pytest.mark.parametrize("mode", [None, "monitor", "enforce"])
def test_local_example_without_docker_or_live_writes(tmp_path, entry, mode):
    result = subprocess.run(
        [str(ROOT / entry), "--local", "--example", "walker", "--seconds", "0.1",
         "--no-safety-checks", "--output", str(tmp_path)]
        + (["--thrash-mode", mode] if mode else []),
        env=dict(os.environ, G1_SIM_PYTHON=sys.executable, CONTAINER_RUNTIME="missing-docker"),
        capture_output=True, text=True, timeout=60, check=False,
    )
    assert result.returncode == 0, result.stderr
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["execution"] == "local"
    assert report["thrash"]["mode"] == (mode or "off")
    assert report["checks"]["thrash_monitoring"] == (mode is not None)
    assert (tmp_path / "trace.npz").is_file()
    assert not (tmp_path / "live.jsonl").exists()


@pytest.mark.parametrize("counterfactuals", [False, True])
def test_compose_failure_cleanup_and_pose_outputs(tmp_path, counterfactuals):
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
        "safety_checks": True, "reason": "test rejection",
    }))
if args[-2:] == ["wait", "sim"]:
    sys.exit(1)
''')
    runtime.chmod(0o755)
    args = [str(ROOT / "desktop_sim.sh"), "--image", "test:local", "--output", str(tmp_path)]
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
