"""Policy contract tests: reset/act interface, bounds, no joint qpos writes from policy."""

from __future__ import annotations

import numpy as np

from parkour_env import HOME_CTRL, ParkourEnv
from policy import NeutralPolicy, ScriptedParkourPolicy


def test_scripted_reset_and_act_shape_and_bounds():
    env = ParkourEnv()
    obs = env.reset(seed=0)
    policy = ScriptedParkourPolicy()
    policy.reset()
    action = policy.act(obs)
    assert action.shape == (env.nu,)
    assert np.all(action >= env.ctrl_low - 1e-9)
    assert np.all(action <= env.ctrl_high + 1e-9)
    assert policy.controller_type == "pretrained+scripted"
    assert policy.base_mode == "free-base"
    assert not hasattr(policy, "assist_target") or getattr(policy, "assist_target", None) is None
    env.close()


def test_neutral_reset_and_act_shape_and_bounds():
    env = ParkourEnv()
    obs = env.reset(seed=0)
    policy = NeutralPolicy()
    policy.reset()
    action = policy.act(obs)
    assert action.shape == (env.nu,)
    assert np.all(action >= env.ctrl_low - 1e-9)
    assert np.all(action <= env.ctrl_high + 1e-9)
    env.close()


def test_act_called_every_control_step_drives_ctrl_only():
    env = ParkourEnv()
    obs = env.reset(seed=1)
    policy = ScriptedParkourPolicy()
    policy.reset()
    q_before = env.data.qpos.copy()
    action = policy.act(obs)
    np.testing.assert_allclose(env.data.qpos, q_before)
    obs2 = env.step(action, policy=policy)
    assert "pelvis_pos" in obs2
    np.testing.assert_allclose(env.data.ctrl, np.clip(action, env.ctrl_low, env.ctrl_high), atol=1e-8)
    env.close()


def test_no_kinematic_root_assist_in_env():
    import inspect
    import parkour_env as pe

    src = inspect.getsource(pe.ParkourEnv)
    assert "_apply_kinematic_root" not in src
    assert "assist_target" not in src
    # step must not write free-joint qpos
    step_src = inspect.getsource(pe.ParkourEnv.step)
    assert "qpos[0:7]" not in step_src
    assert "qpos[0:3]" not in step_src


def test_home_ctrl_length_matches_actuators():
    env = ParkourEnv()
    assert HOME_CTRL.shape[0] == env.nu
    env.close()
