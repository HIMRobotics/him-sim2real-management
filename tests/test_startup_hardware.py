"""Startup faults through fake hardware, without SDK access."""

from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

import tests.test_robot_safety_hardware as hardware


@pytest.fixture
def rig():
    rig = hardware.HardwareAdapterTests()
    rig.setUp()
    rig.adapter.whole_body = True
    rig.adapter.balance_policy = Mock(update=Mock(return_value=np.full(15, 0.1)))
    rig.transport.write_body = Mock(return_value=True)
    return rig


def start(rig):
    assert rig.adapter.arm()
    rig.adapter.submit(rig.command, rig.now + 0.1)
    rig.adapter.step()
    rig.adapter.step()


def test_missing_balance_rejects_before_release(rig):
    rig.adapter.balance_policy = None
    with patch('robot_safety.hardware_adapter.BalancePolicy', side_effect=FileNotFoundError):
        assert not rig.adapter.arm()
    assert rig.transport.release_calls == 0
    rig.transport.write_body.assert_not_called()


def test_inference_failure_before_ownership_never_releases(rig):
    rig.adapter.balance_policy.update.side_effect = RuntimeError('failed')
    start(rig)
    assert rig.adapter.abort_reason
    assert rig.transport.release_calls == 0
    rig.transport.write_body.assert_not_called()


def test_inference_failure_after_ownership_requests_damping(rig):
    start(rig)
    rig.adapter.balance_policy.update.side_effect = RuntimeError('failed')
    rig.now += 0.02
    rig.state()
    rig.adapter.submit(rig.command, rig.now + 0.1)
    rig.adapter.step()
    assert rig.adapter.abort_reason
    assert rig.adapter.hold_failed
    assert not rig.adapter.startup.status()['control_ready']


def test_feedback_torque_check_uses_written_balance_gains(rig):
    start(rig)
    assert rig.adapter.applied_gains[0] == (150, 2)
    rig.adapter.profile = replace(rig.adapter.profile, legs=tuple(
        replace(limits, max_torque_nm=10) for limits in rig.adapter.profile.legs))
    rig.state()
    assert 'joint_torque_limit' in rig.adapter.abort_reason
    assert rig.adapter.hold_failed


@pytest.mark.parametrize('handoff', [False, True])
def test_abort_in_balance_or_handoff_latches_and_stops_writes(rig, handoff):
    start(rig)
    if handoff:
        rig.adapter.startup.ready = True
        assert rig.adapter.startup.authorize(rig.now)
    rig.adapter.abort('test_stop', request_damping=True)
    count = rig.transport.write_body.call_count
    rig.adapter.submit(rig.command, rig.now + 0.1)
    rig.adapter.step()
    assert rig.transport.write_body.call_count == count
    assert rig.adapter.startup.phase == 'fault'
    assert not rig.adapter.startup.status()['control_ready']
    rig.adapter.stop_once()
    assert rig.transport.stop_calls == 1


def test_stabilization_timeout_is_latched(rig):
    start(rig)
    rig.adapter.startup.timeout_seconds = 0.05
    for _ in range(8):
        rig.now += 0.01
        rig.state()
        rig.adapter.submit(rig.command, rig.now + 0.1)
        rig.adapter.step()
    assert rig.adapter.abort_reason
    assert rig.adapter.startup.handoff is None


def test_repeated_sdk_tick_cannot_refresh_telemetry(rig):
    start(rig)
    sample = SimpleNamespace(
        motor_state=[SimpleNamespace(q=0., dq=0., temperature=(25, 25), tau_est=0.) for _ in range(29)],
        mode_machine=5, wireless_remote=[0] * 40, tick=42,
        imu_state=SimpleNamespace(quaternion=(1., 0., 0., 0.), gyroscope=(0., 0., 0.)))
    rig.adapter.receive_state(sample)
    original = rig.adapter.feedback.received_at_secs
    for _ in range(25):
        rig.now += 0.01
        rig.adapter.receive_state(sample)
        rig.adapter.submit(rig.command, rig.now + 0.1)
        rig.adapter.step()
    assert rig.adapter.feedback.received_at_secs == original
    assert rig.adapter.abort_reason
    assert rig.adapter.startup.handoff is None
