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
    shutil.copy2(ROOT / "submit.sh", tmp_path / "submit.sh")
    for name in ("sim.sh", "setup.sh"):
        path = tmp_path / name
        path.write_text(
            "#!/bin/bash\npython3 - \"$@\" <<'INNER'\n"
            "import json, os, sys\n"
            'print(json.dumps({"args": sys.argv[1:], "gate": os.getenv("G1_USE_SAFETY_SERVICE"), '
            '"command": os.getenv("G1_COMMAND_FILE"), '
            '"startup": os.getenv("G1_STARTUP_POLICY")}))\nINNER\n'
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
            "G1_STARTUP_POLICY": "",
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


@pytest.mark.parametrize('startup', ['groot', 'holosoma'])
def test_startup_selection_reaches_supervisor(launchers, startup):
    result = call(launchers, "submit.sh", ["--mode", "real", "--startup", startup])
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["startup"] == startup


@pytest.mark.parametrize('example', ['walker', 'holosoma'])
def test_every_example_defaults_to_groot_startup(launchers, example):
    result = call(launchers, "submit.sh", ["--mode", "real", "--example", example])
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["startup"] == 'groot'


def test_holosoma_example_preserves_explicit_groot_startup(launchers):
    result = call(launchers, "submit.sh", ["--mode", "real", "--startup", "groot",
                                         "--example", "holosoma"])
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["startup"] == 'groot'


def test_real_mode_preserves_gate_and_container_arguments(launchers):
    command = launchers / "command with spaces.yaml"
    command.write_text("command: [0,0,0]\n")
    result = call(
        launchers,
        "submit.sh",
        [
            "--mode",
            "real",
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
        "--image",
        "test-policy",
        "--command",
        str(command),
        "--profile",
        "explicit.json",
        "--",
        "--policy-flag",
    ]
    assert data["command"] == ""


@pytest.mark.parametrize(
    "args",
    [["--mode", "invalid"], ["--mode", "real", "--startup", "unsupported"]],
)
def test_invalid_submission_is_rejected(launchers, args):
    result = call(launchers, "submit.sh", args)
    assert result.returncode == 2
    assert not result.stdout


def test_stop_routes_to_existing_supervisor(launchers):
    result = call(launchers, "submit.sh", ["--mode", "real", "stop"])
    assert result.returncode == 0
    assert json.loads(result.stdout)["args"] == ["stop"]
