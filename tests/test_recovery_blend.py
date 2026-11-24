"""Opt-in recovery_blend_secs: recovery eases the arms from their pose at the stop."""

from dataclasses import replace

import pytest

from robot_safety.hardware_constants import ARM_INDICES, UPPER_BODY_INDICES
from tests.test_startup_hardware import handoff_policy, start
from tests.test_startup_hardware import rig as rig  # noqa: PLC0414

POSE = 0.5      # upper-body position when the stop happens
POLICY = -0.5   # recovery policy output on every joint
LEGS_AND_WAIST = range(15)


def stop_mid_motion(rig, blend):
    rig.adapter.recovery_blend_secs = blend
    start(rig)
    handoff_policy(rig)
    q = list(rig.adapter.feedback.q)
    for index in UPPER_BODY_INDICES:
        q[index] = POSE
    rig.adapter.feedback = replace(rig.adapter.feedback, q=tuple(q))
    rig.adapter.soft_reset_policy.update.return_value = [POLICY] * 29
    rig.transport.write_body.reset_mock()
    rig.adapter.abort('observation_timeout')
    assert rig.adapter.automatic_soft_reset


def recovery_write(rig, elapsed):
    """Run one recovery cycle ``elapsed`` seconds after entry and return its targets."""
    entry = rig.adapter._recovery_entry[0] if rig.adapter._recovery_entry else rig.now
    rig.now = entry + elapsed
    rig.adapter.feedback = replace(rig.adapter.feedback, received_at_secs=rig.now)
    rig.adapter.stop_once()
    assert rig.adapter.abort_reason == 'observation_timeout' and rig.adapter.walk_stop
    return rig.transport.write_body.call_args.args[0]


def test_default_recovery_targets_are_unchanged(rig):
    stop_mid_motion(rig, 0.0)
    targets = recovery_write(rig, 0.0)
    assert all(targets[i]['q'] == pytest.approx(POLICY) for i in range(29))


def test_blend_starts_at_the_stop_pose_and_reaches_the_policy(rig):
    stop_mid_motion(rig, 1.0)
    first = recovery_write(rig, 0.0)
    assert all(first[i]['q'] == pytest.approx(POSE) for i in ARM_INDICES)
    assert all(first[i]['q'] == pytest.approx(POLICY) for i in LEGS_AND_WAIST)
    middle = recovery_write(rig, 0.5)                       # smoothstep(0.5) = 0.5
    assert all(middle[i]['q'] == pytest.approx(0.0, abs=1e-9) for i in ARM_INDICES)
    assert all(middle[i]['q'] == pytest.approx(POLICY) for i in LEGS_AND_WAIST)
    done = recovery_write(rig, 1.0)
    assert all(done[i]['q'] == pytest.approx(POLICY) for i in range(29))
    later = recovery_write(rig, 2.0)
    assert all(later[i]['q'] == pytest.approx(POLICY) for i in range(29))


def test_blend_keeps_the_recovery_gains_and_zero_velocity(rig):
    stop_mid_motion(rig, 1.0)
    targets = recovery_write(rig, 0.25)
    policy = rig.adapter.soft_reset_policy
    for index in range(29):
        assert targets[index]['kp'] == policy.kp[index]
        assert targets[index]['kd'] == policy.kd[index]
        assert targets[index]['dq'] == 0.0 and targets[index]['tau'] == 0.0


def test_each_recovery_entry_restarts_the_blend(rig):
    stop_mid_motion(rig, 1.0)
    recovery_write(rig, 0.0)
    first_entry = rig.adapter._recovery_entry
    rig.adapter.soft_reset_active = False                    # a fresh recovery entry
    targets = recovery_write(rig, 0.3)
    assert rig.adapter._recovery_entry != first_entry
    assert all(targets[i]['q'] == pytest.approx(POSE) for i in ARM_INDICES)


def test_blend_still_damps_on_a_hard_stop(rig):
    stop_mid_motion(rig, 1.0)
    recovery_write(rig, 0.0)
    rig.adapter.abort('hardware_controller_double_start', request_damping=True)
    rig.adapter.stop_once()
    assert rig.transport.stop_calls == 1
