"""Submission routing with inert launchers, without hardware access."""

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def launchers(tmp_path):
    for name in ("submit.sh", "real.sh"):
        shutil.copy2(ROOT / name, tmp_path / name)
    for name in ("sim.sh", "setup.sh"):
        path = tmp_path / name
        path.write_text(
            "#!/bin/bash\npython3 - \"$@\" <<'INNER'\n"
            "import json, os, sys\n"
            'print(json.dumps({"args": sys.argv[1:], "gate": os.getenv("G1_USE_SAFETY_SERVICE"), '
            '"command": os.getenv("G1_COMMAND_FILE")}))\nINNER\n'
        )
        path.chmod(0o755)
    return tmp_path


def call(root, script, args):
    return subprocess.run(
        [str(root / script), *args],
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "CONTAINER_RUNTIME": "unused-runtime",
            "G1_IMAGE": "",
            "G1_COMMAND_FILE": "",
        },
        timeout=10,
        check=False,
    )


def test_simulation_routes_options_without_hardware_gate(launchers):
    args = [
        "--example",
        "walker",
        "--local",
        "--seconds",
        "10",
        "--output",
        "space in path",
    ]
    result = call(launchers, "submit.sh", ["--mode", "sim", *args])
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["args"] == args


@pytest.mark.parametrize(
    "script,args", [("submit.sh", ["--mode", "real"]), ("real.sh", [])]
)
def test_real_alias_preserves_gate_and_container_arguments(launchers, script, args):
    command = launchers / "command with spaces.yaml"
    command.write_text("command: [0,0,0]\n")
    result = call(
        launchers,
        script,
        args
        + [
            "--image",
            "test-policy",
            "--command",
            str(command),
            "--profile",
            "explicit.json",
            "--",
            "--policy-flag",
        ],
    )
    assert result.returncode == 0, result.stderr
    data = json.loads(result.stdout)
    assert data["args"] == [
        "--mode",
        "hardware",
        "--enable-hardware-output",
        "--profile",
        "explicit.json",
        "--image",
        "test-policy",
        "--",
        "--policy-flag",
    ]
    assert data["gate"] == "1"
    assert data["command"] == str(command)


@pytest.mark.parametrize(
    "args",
    [["--mode", "invalid"], ["--mode", "real", "--example", "walker", "--image", "x"]],
)
def test_invalid_submission_is_rejected(launchers, args):
    result = call(launchers, "submit.sh", args)
    assert result.returncode == 2
    assert not result.stdout


def test_stop_routes_to_existing_supervisor(launchers):
    result = call(launchers, "real.sh", ["stop"])
    assert result.returncode == 0
    assert json.loads(result.stdout)["args"] == ["stop"]
