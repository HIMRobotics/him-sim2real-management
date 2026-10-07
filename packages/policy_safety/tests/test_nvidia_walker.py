"""Official checkpoint provenance and Decoupled WBC observation semantics."""

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from policy_safety.adapter import NvidiaWalker, PolicyViolation
from policy_safety.bundle import prepare_g1
from policy_safety.simulation import Simulation

BUNDLE = Path(__file__).resolve().parents[3] / "examples/policies/walker"


def robot_config():
    cfg = yaml.safe_load((BUNDLE / "walker.yaml").read_text())
    return {
        "contract": cfg["contract"],
        "normalization": cfg["normalization"],
        "default_joint_positions": [j["default_position"] for j in cfg["joints"]],
        "action_scales": [j["action_scale"] for j in cfg["joints"]],
    }


class Session:
    def __init__(self, action):
        self.action = action
        self.calls = 0

    def get_inputs(self):
        return [
            SimpleNamespace(
                name="input", type="tensor(float)", shape=["batch_size", 516]
            )
        ]

    def get_outputs(self):
        return [
            SimpleNamespace(
                name="output", type="tensor(float)", shape=["batch_size", 15]
            )
        ]

    def run(self, names, inputs):
        self.calls += 1
        return [np.full((1, 15), self.action, dtype=np.float32)]


def test_official_checkpoint_hashes():
    manifest = json.loads((BUNDLE / "provenance.json").read_text())
    expected = {
        "walker.onnx": "7c82255b6905ffcc4468fa7f8ddcf7b70db168cf1042107ccab887cb6a8e5407",
        "balance.onnx": "f645da599d4ca3d29ed273c8f4712620bb680d34977469ca3aeabe5bb9631c18",
    }
    for name, digest in expected.items():
        assert hashlib.sha256((BUNDLE / name).read_bytes()).hexdigest() == digest
        assert manifest["files"][name]["sha256"] == digest


def test_observation_history_scaling_selection_and_restore():
    walk, balance = Session(0.4), Session(-0.2)
    policy = NvidiaWalker(None, robot_config(), walk, balance)
    quat = [np.sqrt(0.5), np.sqrt(0.5), 0, 0]
    obs = policy.observation(
        policy.default + 0.1,
        np.ones(29),
        quat,
        [100, 200, 300, 0.2, 0.4, 0.6],
        [0.3, 0.2, -0.4],
    )
    np.testing.assert_array_equal(obs[0, :430], 0)
    frame = obs[0, 430:]
    np.testing.assert_allclose(frame[:7], [0.6, 0.4, -0.2, 0.74, 0, 0, 0])
    np.testing.assert_allclose(frame[7:10], [0.1, 0.2, 0.3])
    np.testing.assert_allclose(frame[10:13], [0, -1, 0], atol=1e-6)
    np.testing.assert_allclose(frame[13:42], 0.1, atol=1e-6)
    np.testing.assert_allclose(frame[42:71], 0.05)
    raw = policy.infer(obs)
    target = policy.decode(raw)
    np.testing.assert_allclose(target[:15], policy.default[:15] + 0.1, atol=1e-7)
    np.testing.assert_array_equal(target[15:], 0)
    saved = policy.snapshot()
    obs2 = policy.observation(
        policy.default, np.zeros(29), [1, 0, 0, 0], np.zeros(6), [0, 0, 0]
    )
    np.testing.assert_array_equal(obs2[0, 344:430], frame)
    np.testing.assert_allclose(obs2[0, -15:], 0.4)
    policy.infer(obs2)
    assert walk.calls == balance.calls == 1
    policy.reset()
    policy.restore(saved)
    assert not policy.use_balance
    np.testing.assert_array_equal(policy.history, saved[0])
    np.testing.assert_array_equal(policy.previous_action, saved[1])


def test_nonfinite_output_and_wrong_model_rejected():
    policy = NvidiaWalker(None, robot_config(), Session(float("nan")), Session(0))
    policy.use_balance = False
    with pytest.raises(PolicyViolation, match="action"):
        policy.infer(np.zeros((1, 516), dtype=np.float32))
    wrong = Session(0)
    wrong.get_inputs = lambda: [
        SimpleNamespace(name="obs", type="tensor(float)", shape=[1, 99])
    ]
    with pytest.raises(ValueError, match="unexpected NVIDIA tensor"):
        NvidiaWalker(None, robot_config(), wrong, Session(0))


def test_portable_screening_bundle_runs_both_official_models(tmp_path):
    destination = tmp_path / "bundle"
    prepare_g1(BUNDLE, destination)
    assert (destination / "balance.onnx").is_file()
    assert (destination / "LICENSE").is_file()
    sim = Simulation(destination / "robot.json")
    sim.reset(0)
    policy = NvidiaWalker(destination / "walker.onnx", sim.robot)
    for command in ([0.3, 0, 0], [0, 0, 0]):
        obs = sim.observation(policy, command)
        raw = policy.infer(obs)
        expected_model = policy.balance_session if command[0] == 0 else policy.session
        expected = expected_model.run(None, {expected_model.get_inputs()[0].name: obs})[
            0
        ][0]
        np.testing.assert_array_equal(raw, expected)
        assert policy.decode(raw).shape == (29,)
