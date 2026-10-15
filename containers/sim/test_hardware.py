"""Exercise real adapter transitions through the normal simulation runner."""

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "containers/sim"), str(ROOT / "robot_safety"),
               str(ROOT / "packages/policy_safety/src")]

from local import run
from runner import Runner
from scenarios import SafetyScenario
from test_runner import hold


@pytest.mark.parametrize("case,tilt,phase", [
    ("recovery_tilt", 20, "recovery"),
    ("policy_tilt", 35, "damping"),
    ("gantry_tilt", 35, "damping"),
    ("policy_crouch", 0, "damping"),
])
def test_real_policy_safety_scenarios(tmp_path, case, tilt, phase):
    report = run("holosoma", tmp_path, seconds=8, scenario=case, tilt_degrees=tilt)
    assert report["scenario"]["passed"], report
    safety = report["hardware_safety"]
    assert safety["phase"] == phase
    assert report["checks"]["hardware_adapter"]
    assert not report["hardware_authorization"]
    if phase == "damping":
        assert safety["recovery_writes"] == 0
        assert safety["damping_at_secs"] == pytest.approx(report["scenario"]["event"]["time_secs"])
    with np.load(report["trace"], allow_pickle=False) as trace:
        phases = json.loads(str(trace["startup"]))
        assert phases[-1]["phase"] == phase
        assert len(phases) == len(trace["qpos"])


def test_recovery_can_be_overridden_and_cannot_resume_submission(tmp_path, monkeypatch):
    import local

    captured = []
    original = local._finish

    def finish(runner, output, extra):
        captured.append(runner)
        return original(runner, output, extra)

    monkeypatch.setattr(local, "_finish", finish)
    run("holosoma", tmp_path, seconds=3, scenario="recovery_tilt")
    runner = captured[0]
    assert runner.hardware.snapshot()["phase"] == "recovery"
    original_reason = runner.reason
    writes = runner.hardware.transport.body_writes
    state = runner.feedback(runner.sim_time)
    assert runner.command(hold(runner, state, runner.last_sequence + 1), runner.sim_time) == original_reason
    runner.handle({"op": "abort", "reason": "policy_container_exit"})
    runner.step(runner.sim_time)
    assert runner.hardware.transport.body_writes > writes
    assert runner.hardware.snapshot()["phase"] == "recovery"
    # Fresh posture feedback must stop an already latched recovery immediately.
    SafetyScenario("policy_tilt", 35).apply(runner)
    assert runner.hardware.snapshot()["phase"] == "damping"
    writes = runner.hardware.transport.body_writes
    runner.step(runner.sim_time)
    assert runner.hardware.transport.body_writes == writes
    assert runner.reason == original_reason


def test_missing_recovery_weights_prevent_output(monkeypatch):
    monkeypatch.setenv("HIM_HOLOSOMA_ONNX", "/missing/recovery.onnx")
    runner = Runner()
    state = runner.feedback(0)
    runner.command(hold(runner, state, 0), 0)
    runner.step(0)
    assert runner.reason == "hardware_arm_rejected"
    assert runner.hardware.adapter.arm_rejection_reason == "soft_reset_model_unavailable"
    assert runner.hardware.transport.body_writes == 0
    assert runner.hardware.snapshot()["phase"] == "damping"
    assert not runner.handle({"op": "arm"})["ok"]


def test_adapter_diagnostics_do_not_corrupt_plant_protocol():
    env = dict(os.environ, G1_STARTUP_POLICY="invalid-test-value",
               PYTHONPATH=os.pathsep.join(sys.path))
    result = subprocess.run([sys.executable, str(ROOT / "containers/sim/plant_server.py")],
                            input='{"op":"arm"}\n{"op":"report"}\n',
                            text=True, capture_output=True, env=env, check=True)
    responses = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(responses) == 2
    assert responses[0]["ok"]
    assert responses[1]["checks"]["hardware_adapter"]
    assert "Unknown G1_STARTUP_POLICY" in result.stderr
