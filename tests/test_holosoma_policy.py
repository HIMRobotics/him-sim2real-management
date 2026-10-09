"""Holosoma locomotion observation math, with and without the ONNX file."""

import json

import numpy as np
import pytest

from robot_safety.hardware_constants import G1_29_JOINT_NAMES
from robot_safety.holosoma_policy import (
    ACTOR_OBS_SLICES,
    DEFAULT_ANGLES,
    OBS_DIM,
    HolosomaLocomotionPolicy,
    holosoma_weights,
    projected_gravity,
)
from robot_safety.startup_balance import StartupBalance


class _Tensor:
    def __init__(self, name, shape):
        self.name = name
        self.shape = shape


class _Meta:
    def __init__(self, names):
        self.custom_metadata_map = {
            "kp": json.dumps(list(range(1, 30))),
            "kd": json.dumps([0.1] * 29),
            "dof_names": json.dumps(names),
        }


class _Session:
    def __init__(self, names, action=None):
        self._meta = _Meta(names)
        self.action = np.zeros(29) if action is None else np.asarray(action, dtype=float)
        self.seen = None

    def get_inputs(self):
        return [_Tensor("actor_obs", [1, OBS_DIM])]

    def get_modelmeta(self):
        return self._meta

    def run(self, _names, feed):
        self.seen = feed["actor_obs"].copy()
        return [self.action.reshape(1, 29).astype(np.float32)]


def _policy(names=None, action=None):
    names = names or [name + "_joint" for name in G1_29_JOINT_NAMES]
    return HolosomaLocomotionPolicy("unused.onnx", session=_Session(names, action))


def test_standing_observation_is_the_holosoma_100_vector():
    policy = _policy()
    q = DEFAULT_ANGLES.copy()
    target = policy.update(q, np.zeros(29), [1, 0, 0, 0], np.zeros(3), [0, 0, 0])
    obs = policy.session.seen.reshape(-1)
    assert obs.shape == (OBS_DIM,)
    assert list(ACTOR_OBS_SLICES) == sorted(ACTOR_OBS_SLICES)
    assert np.allclose(obs[0:29], 0)
    assert np.allclose(obs[29:35], 0)
    assert np.allclose(obs[35:37], -1)
    assert np.allclose(obs[37:95], 0)
    assert np.allclose(obs[95:98], [0, 0, -1])
    assert np.allclose(obs[98:100], 0)
    assert np.allclose(target, DEFAULT_ANGLES)
    assert np.allclose(projected_gravity([1, 0, 0, 0]), [0, 0, -1])


def test_a_walk_command_resets_the_phase_out_of_the_stand():
    policy = _policy(action=np.ones(29))
    policy.update(DEFAULT_ANGLES, np.zeros(29), [1, 0, 0, 0], np.zeros(3), [0, 0, 0])
    policy.update(DEFAULT_ANGLES, np.zeros(29), [1, 0, 0, 0], np.zeros(3), [0.3, 0, 0])
    obs = policy.session.seen.reshape(-1)
    assert np.allclose(obs[0:29], 1)
    assert np.allclose(obs[32:35], [0, 0.3, 0])
    assert np.allclose(obs[35:37], [1, -1])
    assert np.allclose(obs[98:100], 0)
    assert np.allclose(policy.last_action, 1)


def test_actions_follow_the_model_joint_order():
    swapped = [name + "_joint" for name in G1_29_JOINT_NAMES]
    swapped[0], swapped[1] = swapped[1], swapped[0]
    action = np.arange(29, dtype=float)
    policy = _policy(swapped, action)
    target = policy.update(DEFAULT_ANGLES, np.zeros(29), [1, 0, 0, 0], np.zeros(3), [0, 0, 0])
    assert policy.kp[0] == 2
    assert policy.kp[1] == 1
    assert target[0] == pytest.approx(DEFAULT_ANGLES[0] + 0.25 * 1)
    assert target[1] == pytest.approx(DEFAULT_ANGLES[1] + 0.25 * 0)


def test_startup_eases_to_the_crouch_before_the_network():
    policy = _policy()
    startup = StartupBalance(policy)
    state = {
        "q": np.zeros(29),
        "dq": np.zeros(29),
        "quat": [1, 0, 0, 0],
        "gyro": np.zeros(3),
    }
    submitted = {"q": np.zeros(29), "dq": np.zeros(29), "kp": np.ones(29),
                 "kd": np.ones(29), "tau": np.zeros(29)}
    first = startup.command(0.0, 0.0, state, submitted)
    assert policy.session.seen is None
    assert np.allclose(first["q"], 0)
    assert policy.entry_ramp_seconds == pytest.approx(10.0)
    middle = startup.command(5.0, 5.0, state, submitted)
    assert policy.session.seen is None
    assert np.allclose(middle["q"], 0.5 * DEFAULT_ANGLES)
    running = startup.command(10.0, 10.0, state, submitted)
    assert policy.standing
    assert np.allclose(running["q"], DEFAULT_ANGLES)
    assert np.allclose(policy.session.seen.reshape(-1)[35:37], -1)
    assert np.allclose(policy.session.seen.reshape(-1)[95:98], [0, 0, -1])


def test_fastsac_file_matches_the_checker_contract():
    try:
        path = holosoma_weights()
    except ValueError:
        pytest.skip("Holosoma ONNX is not on this machine")
    policy = HolosomaLocomotionPolicy(path)
    assert np.array_equal(policy._order, np.arange(29))
    assert policy.kp[0] == pytest.approx(40.179238471)
    assert policy.kp[3] == pytest.approx(99.098427777)
    assert policy.kp.shape == (29,)
    assert np.all(policy.kp > 0) and np.all(policy.kp <= 500)
    assert np.all(policy.kd >= 0)
    target = policy.update(
        DEFAULT_ANGLES, np.zeros(29), [1, 0, 0, 0], np.zeros(3), [0, 0, 0]
    )
    assert target.shape == (29,) and np.isfinite(target).all()
