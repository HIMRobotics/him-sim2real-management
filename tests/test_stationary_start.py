"""Bounded readiness waits with synthetic telemetry and no hardware access."""

from dataclasses import replace
from unittest.mock import patch

import pytest

from tests.test_startup_hardware import rig


def velocity(rig, value):
    speeds = [0.] * 29
    speeds[9] = value
    rig.adapter.feedback = replace(rig.adapter.feedback, dq=tuple(speeds))


def assert_inert(rig):
    assert not rig.adapter.armed
    assert rig.transport.release_calls == 0
    assert rig.transport.stop_calls == 0
    rig.transport.write_body.assert_not_called()


def test_waits_for_settling_and_leaves_arming_to_the_caller(rig):
    velocity(rig, -0.099266)

    def advance(seconds):
        rig.now += seconds
        rig.state()
        velocity(rig, -0.099266 if rig.now < 10.1 else 0)

    with patch('robot_safety.hardware_adapter.time.sleep', side_effect=advance):
        assert rig.adapter.wait_until_stationary()
    assert rig.now >= 10.35
    assert rig.adapter.time_ready_mode_confirmed_secs is None
    assert_inert(rig)
    assert rig.adapter.check_ready_mode()
    velocity(rig, -0.099266)
    assert not rig.adapter.arm()
    assert 'right_knee speed' in rig.adapter.arm_rejection_reason


def test_motion_resets_the_continuous_window(rig):
    def advance(seconds):
        rig.now += seconds
        rig.state()
        velocity(rig, 0.1 if 10.20 <= rig.now < 10.22 else 0)

    with patch('robot_safety.hardware_adapter.time.sleep', side_effect=advance):
        assert rig.adapter.wait_until_stationary()
    assert rig.now >= 10.47
    assert_inert(rig)


def test_persistent_motion_times_out_with_the_limiting_joint(rig):
    velocity(rig, -0.099266)

    def advance(seconds):
        rig.now += seconds
        rig.state()
        velocity(rig, -0.099266)

    with patch('robot_safety.hardware_adapter.time.sleep', side_effect=advance):
        assert not rig.adapter.wait_until_stationary()
    assert rig.now == pytest.approx(13.0, abs=0.006)
    assert 'timed out after 3s: right_knee speed' in rig.adapter.arm_rejection_reason
    assert_inert(rig)


def test_repeated_feedback_cannot_complete_window(rig):
    def advance(seconds):
        rig.now += seconds

    with patch('robot_safety.hardware_adapter.time.sleep', side_effect=advance):
        assert not rig.adapter.wait_until_stationary()
    assert 'fresh feedback required' in rig.adapter.arm_rejection_reason
    assert_inert(rig)


def test_abort_interrupts_wait_without_arming(rig):
    def advance(seconds):
        rig.now += seconds
        rig.adapter.abort('test_stop')

    with patch('robot_safety.hardware_adapter.time.sleep', side_effect=advance):
        assert not rig.adapter.wait_until_stationary()
    assert rig.adapter.arm_rejection_reason == 'already armed or stopped'
    assert_inert(rig)
