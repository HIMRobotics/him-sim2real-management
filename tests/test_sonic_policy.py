"""SONIC standing observation math, without the ONNX weights."""

import numpy as np
import pytest

from robot_safety.sonic_policy import (
    ACTION_SCALE,
    DEFAULT_ANGLES,
    ENCODER_DIM,
    HISTORY,
    KP,
    POLICY_DIM,
    SonicStandingPolicy,
    anchor_orientation,
    heading_quat,
    isaaclab_from_motor,
    motor_targets,
    require_sonic_weights,
)
from robot_safety.startup_balance import StartupBalance


class _Tensor:
    def __init__(self, name, size):
        self.name = name
        self.shape = [1, size]


class _Session:
    def __init__(self, name, in_size, out_size, action=None):
        self._in = _Tensor(name, in_size)
        self._out = out_size
        self.action = np.zeros(out_size) if action is None else np.asarray(action)
        self.seen = None

    def get_inputs(self):
        return [self._in]

    def get_outputs(self):
        return [_Tensor("out", self._out)]

    def run(self, _names, feeds):
        self.seen = next(iter(feeds.values())).reshape(-1).copy()
        return [self.action.reshape(1, -1)]


def policy():
    encoder = _Session("encoder", ENCODER_DIM, 64)
    decoder = _Session("actions", POLICY_DIM, 29)
    return SonicStandingPolicy(encoder=encoder, decoder=decoder), encoder, decoder


def test_zero_action_is_the_standing_pose():
    np.testing.assert_allclose(motor_targets(np.zeros(29)), DEFAULT_ANGLES)
    assert ACTION_SCALE.shape == (29,)
    assert KP.shape == (29,)
    assert KP[0] > KP[4]


def test_upright_heading_reference_is_identity_orientation():
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(anchor_orientation(quat, heading_quat(quat)), [1, 0, 0, 1, 0, 0])


def test_joint_offset_uses_isaaclab_order():
    offset = isaaclab_from_motor(np.zeros(29) - DEFAULT_ANGLES)
    np.testing.assert_allclose(offset[0], -DEFAULT_ANGLES[0])


def test_update_returns_twenty_nine_targets_and_records_history():
    standing, encoder, decoder = policy()
    standing.warmup()
    q = DEFAULT_ANGLES.copy()
    target = standing.update(q, np.zeros(29), [1, 0, 0, 0], np.zeros(3))
    assert target.shape == (29,)
    np.testing.assert_allclose(target, DEFAULT_ANGLES)
    assert encoder.seen.shape == (ENCODER_DIM,)
    assert decoder.seen.shape == (POLICY_DIM,)
    assert encoder.seen[0] == 0
    assert np.count_nonzero(encoder.seen) > 0
    assert standing.joint_position.maxlen == HISTORY


def test_sonic_reaches_the_standing_pose_before_the_network_runs():
    standing, encoder, decoder = policy()
    gate = StartupBalance(standing)
    sample = dict(q=np.zeros(29), dq=np.zeros(29), quat=[1, 0, 0, 0], gyro=np.zeros(3))
    submitted = {key: np.zeros(29) for key in ("q", "dq", "kp", "kd", "tau")}
    first = gate.command(0.0, 0.0, sample, submitted)
    np.testing.assert_allclose(first["q"], np.zeros(29))
    np.testing.assert_allclose(first["kp"], standing.kp)
    assert encoder.seen is None and decoder.seen is None
    middle = gate.command(1.5, 1.5, sample, submitted)
    np.testing.assert_allclose(middle["q"], 0.5 * DEFAULT_ANGLES)
    assert decoder.seen is None
    assert not gate.authorize(1.5)
    enabled = gate.command(3.0, 3.0, sample, submitted)
    assert decoder.seen is not None
    np.testing.assert_allclose(enabled["q"], DEFAULT_ANGLES)
    assert standing.heading is not None


def test_startup_uses_full_sonic_gains():
    standing, _encoder, _decoder = policy()
    gate = StartupBalance(standing)
    sample = dict(q=list(DEFAULT_ANGLES), dq=[0.0] * 29, quat=[1, 0, 0, 0], gyro=[0, 0, 0])
    command = gate.command(0.0, 0.0, sample, {key: np.zeros(29) for key in ("q", "dq", "kp", "kd", "tau")})
    np.testing.assert_allclose(command["kp"], standing.kp)
    np.testing.assert_allclose(command["kd"], standing.kd)


def test_missing_weights_name_the_groot_undo(tmp_path):
    with pytest.raises(FileNotFoundError, match="--startup groot"):
        require_sonic_weights(tmp_path)


def test_downloaded_default_sonic_stands_at_the_nominal_pose():
    from robot_safety.sonic_policy import resolve_sonic_dir

    directory = resolve_sonic_dir()
    if not (directory / "model_decoder.onnx").is_file():
        pytest.skip("SONIC weights are not downloaded")
    standing = SonicStandingPolicy(directory)
    standing.warmup()
    target = standing.update(DEFAULT_ANGLES, np.zeros(29), [1, 0, 0, 0], np.zeros(3))
    assert np.isfinite(target).all()
    assert float(np.max(np.abs(target - DEFAULT_ANGLES))) < 0.5
