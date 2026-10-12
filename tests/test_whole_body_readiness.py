"""Whole-body readiness uses physical limits without a stationary-motion metric."""

from dataclasses import replace
import math

import pytest

from tests.test_startup_hardware import rig, start


@pytest.mark.parametrize('speed', [0.068722, -0.099266])
def test_small_measured_velocity_does_not_reject_whole_body_arming(rig, speed):
    speeds = [0.] * 29
    speeds[3] = speed
    rig.adapter.feedback = replace(rig.adapter.feedback, dq=tuple(speeds))
    assert abs(speed) > rig.adapter.profile.initial_velocity_tolerance_rad_secs
    assert rig.adapter.arm()
    assert rig.transport.release_calls == 0
    rig.transport.write_body.assert_not_called()


@pytest.mark.parametrize('index', [3, 9, 19])
def test_physical_speed_limit_still_rejects_before_ownership(rig, index):
    limits = dict(rig.adapter._monitored_joints())[index]
    speeds = [0.] * 29
    speeds[index] = limits.max_velocity_rad_secs + 0.01
    rig.adapter.feedback = replace(rig.adapter.feedback, dq=tuple(speeds))
    assert not rig.adapter.arm()
    assert 'joint_velocity_limit' in rig.adapter.arm_rejection_reason
    assert rig.transport.release_calls == 0
    rig.transport.write_body.assert_not_called()


@pytest.mark.parametrize(('tilt', 'recovery_available', 'expect_recovery'), [
    (0, True, True),
    (30, True, True),
    (30.01, True, False),
    (0, False, False),
])
def test_runtime_physical_speed_violation_selects_recovery_or_damping(
    rig, tilt, recovery_available, expect_recovery,
):
    start(rig)
    recovery = rig.adapter.soft_reset_policy
    if not recovery_available:
        rig.adapter.soft_reset_policy = None
    speeds = [0.] * 29
    speeds[3] = rig.adapter.profile.legs[3].max_velocity_rad_secs + 0.01
    pitch = math.radians(tilt) / 2
    quat = (math.cos(pitch), 0, math.sin(pitch), 0)
    rig.adapter.feedback = replace(rig.adapter.feedback, dq=tuple(speeds), quat=quat)
    rig.transport.write_body.reset_mock()
    rig.adapter._abort_if_motion_exceeds_limits()
    assert 'joint_velocity_limit' in rig.adapter.abort_reason
    reason = rig.adapter.abort_reason
    assert rig.adapter.command is None
    assert rig.adapter.stop_requested.is_set()
    assert rig.adapter.automatic_soft_reset is expect_recovery
    assert rig.adapter.hold_failed is not expect_recovery
    rig.adapter.stop_once()
    assert rig.adapter.abort_reason == reason
    if expect_recovery:
        recovery.update.assert_called_once_with(
            rig.adapter.feedback.q, tuple(speeds), quat, rig.adapter.feedback.gyro,
        )
        rig.transport.write_body.assert_called_once()
        targets, _ = rig.transport.write_body.call_args[0]
        assert len(targets) == 29
        assert all(target['dq'] == target['tau'] == 0 for target in targets.values())
        assert rig.transport.stop_calls == 0
    else:
        recovery.update.assert_not_called()
        rig.transport.write_body.assert_not_called()
        assert rig.transport.stop_calls == 1
