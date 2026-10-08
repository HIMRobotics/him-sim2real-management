"""Whole-body readiness uses physical limits without a stationary-motion metric."""

from dataclasses import replace

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


def test_runtime_physical_speed_violation_still_requests_damping(rig):
    start(rig)
    speeds = [0.] * 29
    speeds[3] = rig.adapter.profile.legs[3].max_velocity_rad_secs + 0.01
    rig.adapter.feedback = replace(rig.adapter.feedback, dq=tuple(speeds))
    rig.adapter._abort_if_motion_exceeds_limits()
    assert 'joint_velocity_limit' in rig.adapter.abort_reason
    assert rig.adapter.hold_failed
    rig.adapter.stop_once()
    assert rig.transport.stop_calls == 1
