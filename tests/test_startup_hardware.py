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


def finish_entry(rig):
    for _ in range(51):
        rig.now += 0.01
        rig.state()
        rig.adapter.submit(rig.command, rig.now + 0.1)
        rig.adapter.step()
    assert rig.adapter.abort_reason is None


@pytest.mark.parametrize('name', ['groot', 'holosoma'])
def test_startup_selection_preserves_soft_reset_policy(rig, name):
    rig.adapter.startup_policy_name = name
    balance = rig.adapter.balance_policy
    recovery = rig.adapter.soft_reset_policy
    with patch('robot_safety.hardware_adapter.HolosomaLocomotionPolicy') as factory, \
            patch('robot_safety.hardware_adapter.holosoma_weights', return_value='model.onnx'):
        factory.return_value.entry_ramp_seconds = 10.0
        assert rig.adapter._ensure_startup_policy()
        if name == 'groot':
            factory.assert_not_called()
            assert rig.adapter.startup_policy is balance
        else:
            factory.assert_called_once_with('model.onnx')
            factory.return_value.warmup.assert_called_once_with()
            assert rig.adapter.startup_policy is factory.return_value
        assert rig.adapter.balance_policy is balance
        assert rig.adapter.soft_reset_policy is recovery
        assert rig.adapter.startup_policy is not recovery
        assert rig.adapter._ensure_startup_policy()
        assert factory.call_count == (name == 'holosoma')


def test_missing_balance_rejects_before_release(rig):
    rig.adapter.balance_policy = None
    with patch('robot_safety.hardware_adapter.BalancePolicy', side_effect=FileNotFoundError):
        assert not rig.adapter.arm()
    assert rig.transport.release_calls == 0
    rig.transport.write_body.assert_not_called()


def test_missing_recovery_model_rejects_before_release(rig):
    rig.adapter.soft_reset_policy = None
    with patch('robot_safety.hardware_adapter.holosoma_weights', side_effect=ValueError('missing')):
        assert not rig.adapter.arm()
    assert rig.adapter.arm_rejection_reason == 'soft_reset_model_unavailable'
    assert rig.transport.release_calls == 0
    rig.transport.write_body.assert_not_called()


@pytest.mark.parametrize('failure', ['stale', 'nonfinite', 'shape', 'write', 'stop_during_inference'])
def test_recovery_faults_stop_body_output(rig, failure):
    start(rig)
    rig.press_f3_twice()
    rig.transport.write_body.reset_mock()
    policy = rig.adapter.soft_reset_policy
    if failure == 'stale':
        rig.now += 0.3
    elif failure == 'nonfinite':
        policy.update.return_value = [float('nan')] * 29
    elif failure == 'shape':
        policy.update.return_value = [0.0] * 15
    elif failure == 'write':
        rig.transport.write_body.return_value = False
    else:
        def stop(*args):
            rig.adapter.abort('hardware_controller_double_start', request_damping=True)
            return [0.0] * 29
        policy.update.side_effect = stop
    rig.adapter.step()
    if failure != 'write':
        rig.transport.write_body.assert_not_called()
    rig.adapter.stop_once()
    assert rig.transport.stop_calls >= 1
    assert rig.transport.body_cleared


def test_inference_failure_before_ownership_never_releases(rig):
    rig.adapter.balance_policy.update.side_effect = RuntimeError('failed')
    start(rig)
    assert rig.adapter.abort_reason
    assert rig.transport.release_calls == 0
    rig.transport.write_body.assert_not_called()


def test_inference_failure_after_ownership_requests_recovery_when_upright(rig):
    start(rig)
    rig.adapter.balance_policy.update.side_effect = RuntimeError('failed')
    rig.now += 0.02
    rig.state()
    rig.adapter.submit(rig.command, rig.now + 0.1)
    rig.adapter.step()
    assert rig.adapter.abort_reason
    assert rig.adapter.automatic_soft_reset
    assert not rig.adapter.hold_failed
    rig.adapter.step()
    rig.adapter.soft_reset_policy.update.assert_called_once()
    assert rig.transport.stop_calls == 0
    assert not rig.adapter.startup.status()['control_ready']


def test_feedback_torque_check_uses_written_balance_gains(rig):
    start(rig)
    finish_entry(rig)
    assert rig.adapter.applied_gains[0] == (150, 2)
    rig.adapter.profile = replace(rig.adapter.profile, legs=tuple(
        replace(limits, max_torque_nm=10) for limits in rig.adapter.profile.legs))
    rig.state()
    assert 'joint_torque_limit' in rig.adapter.abort_reason
    assert rig.adapter.automatic_soft_reset
    assert rig.adapter.q_applied is None


@pytest.mark.parametrize('handoff', [False, True])
def test_double_start_in_balance_or_handoff_latches_and_stops_writes(rig, handoff):
    start(rig)
    if handoff:
        finish_entry(rig)
        rig.adapter.startup.ready = True
        assert rig.adapter.startup.authorize(rig.now)
    rig.adapter.abort('hardware_controller_double_start', request_damping=True)
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


def test_balance_overshoot_is_limited_and_recorded(rig):
    limits = list(rig.adapter.profile.legs)
    limits[4] = replace(limits[4], max_torque_nm=35)
    rig.adapter.profile = replace(rig.adapter.profile, legs=tuple(limits))
    target = np.zeros(15)
    target[4] = 0.9
    rig.adapter.balance_policy.update.return_value = target
    start(rig)
    finish_entry(rig)
    assert rig.adapter.abort_reason is None
    written = rig.transport.write_body.call_args[0][0][4]
    assert written['q'] * written['kp'] == pytest.approx(31.5)
    detail = rig.adapter.joint_diagnostics_snapshot()['left_ankle_pitch']['torque_target_clipping']
    assert detail['hard_limit_nm'] == 35
    assert detail['requested_torque_nm'] == pytest.approx(36)


def test_measured_torque_fault_selects_recovery_when_upright(rig):
    start(rig)
    torques = [0.0] * 29
    torques[4] = rig.adapter.profile.legs[4].max_torque_nm + 0.1
    rig.adapter.feedback = replace(rig.adapter.feedback, tau_est=tuple(torques))
    rig.adapter._abort_if_motion_exceeds_limits()
    assert 'joint_torque_limit' in rig.adapter.abort_reason
    assert rig.adapter.automatic_soft_reset
    assert rig.adapter.command is None


def test_motion_after_limited_write_can_still_fault_and_recover(rig):
    test_balance_overshoot_is_limited_and_recorded(rig)
    q = list(rig.adapter.feedback.q)
    q[4] = -0.2
    rig.adapter.feedback = replace(rig.adapter.feedback, q=tuple(q))
    rig.adapter._abort_if_motion_exceeds_limits()
    assert 'joint_torque_limit' in rig.adapter.abort_reason
    assert rig.adapter.automatic_soft_reset
    assert rig.adapter.command is None


def test_whole_body_soft_reset_continues_balance_without_damping(rig):
    start(rig)
    rig.press_f3_twice()
    assert rig.adapter.abort_reason == 'hardware_controller_double_f3'
    assert rig.adapter.walk_stop
    assert not rig.adapter.hold_failed
    assert rig.adapter.q_applied is None
    for _ in range(20):
        rig.now += 0.02
        rig.state()
        rig.adapter.abort('hardware_controller_double_f3')
        rig.adapter.abort('policy_container_exit')
        rig.adapter.step()
        assert rig.adapter.last_worker_cycle_secs == rig.now
    assert rig.transport.write_body.call_count >= 20
    targets, _ = rig.transport.write_body.call_args[0]
    assert all(target["kp"] == 40.0 for target in targets.values())
    rig.adapter.soft_reset_policy.reset.assert_called_once()
    assert rig.transport.stop_calls == 0
    assert rig.adapter.walk_stop
    assert not rig.adapter.hold_failed
    rig.adapter.abort('hardware_controller_double_start', request_damping=True)
    rig.adapter.stop_once()
    assert rig.transport.stop_calls == 1


def test_real_fault_still_overrides_soft_reset(rig):
    start(rig)
    rig.press_f3_twice()
    rig.adapter.stop_once()
    rig.adapter.abort('hardware_output_worker_stalled')
    rig.adapter.stop_once()
    assert rig.transport.stop_calls == 1
    assert not rig.adapter.walk_stop


def test_startup_release_does_not_announce_or_request_soft_reset():
    from robot_safety.unitree_transport import UnitreeArmTransport

    transport = UnitreeArmTransport.__new__(UnitreeArmTransport)
    transport.output_enabled = True
    transport.switcher = Mock()
    transport.switcher.CheckMode.return_value = (0, {'name': 'ai'})
    transport.switcher.ReleaseMode.return_value = (0, None)
    transport._set_motion_silent = Mock()
    transport._say_soft_reset = Mock()
    transport.locomotion = Mock()
    with patch('robot_safety.unitree_transport.Thread') as thread:
        assert transport.release_motion() == 0
    thread.assert_not_called()
    transport._say_soft_reset.assert_not_called()
    transport.locomotion.SetFsmId.assert_not_called()


def test_release_writes_measured_pose_without_an_extra_output_tick(rig):
    def release():
        rig.now += 0.04
        rig.state()
        rig.adapter.feedback = replace(rig.adapter.feedback, q=(0.2,) * 29)
        rig.adapter.submit(rig.command, rig.now + 0.1)
        return 0

    rig.transport.release_motion = Mock(side_effect=release)
    assert rig.adapter.arm()
    rig.adapter.submit(rig.command, rig.now + 0.1)
    rig.adapter.step()
    rig.transport.release_motion.assert_called_once()
    rig.transport.write_body.assert_called_once()
    targets = rig.transport.write_body.call_args[0][0]
    assert [targets[i]['q'] for i in range(29)] == pytest.approx([0.2] * 29)
    assert rig.adapter.startup.entry_started == rig.now
    assert rig.adapter.startup.phase == 'entering_balance'
    assert rig.adapter.balance_policy.reset.call_count == 2


@pytest.mark.parametrize('failure', ['expired', 'stale', 'stop', 'inference'])
def test_fault_after_release_never_writes_and_uses_stop_path(rig, failure):
    def release():
        if failure == 'expired':
            rig.now += 0.11
            rig.state()
        elif failure == 'stale':
            rig.now += 0.21
        elif failure == 'stop':
            rig.adapter.abort('operator_stop')
        else:
            rig.adapter.balance_policy.update.side_effect = RuntimeError('inference failed')
        return 0

    rig.transport.release_motion = Mock(side_effect=release)
    assert rig.adapter.arm()
    rig.adapter.submit(rig.command, rig.now + 0.1)
    rig.adapter.step()
    rig.transport.write_body.assert_not_called()
    assert rig.adapter.abort_reason
    assert rig.adapter.startup.phase == 'fault'
    rig.adapter.stop_once()
    if failure in ('expired', 'inference'):
        assert rig.adapter.automatic_soft_reset
        rig.adapter.soft_reset_policy.update.assert_called_once()
        assert rig.transport.stop_calls == 0
    else:
        assert rig.transport.stop_calls == 1


def test_failed_release_and_failed_restore_do_not_claim_stock_ownership(rig):
    rig.transport.release_motion = Mock(return_value=3104)
    rig.transport.restore_motion = Mock(return_value=3104)
    assert rig.adapter.arm()
    rig.adapter.submit(rig.command, rig.now + 0.1)
    rig.adapter.step()
    rig.transport.write_body.assert_not_called()
    assert rig.adapter.balance_released
    assert rig.adapter.hold_failed
    assert rig.adapter.abort_reason == 'hardware_release_failed'


def test_abort_during_post_release_reset_preserves_fault(rig):
    assert rig.adapter.arm()
    rig.adapter.balance_policy.reset.side_effect = lambda: rig.adapter.abort(
        'hardware_controller_double_start', request_damping=True)
    rig.adapter.submit(rig.command, rig.now + 0.1)
    rig.adapter.step()
    rig.transport.write_body.assert_not_called()
    assert rig.adapter.startup.phase == 'fault'
    assert rig.adapter.hold_failed


@pytest.mark.parametrize('tilt', [-90, -30.01, -30, 0, 30, 30.01, 90, 180])
@pytest.mark.parametrize('yaw', [0, 90, 180])
def test_automatic_fault_uses_measured_up_axis(rig, tilt, yaw):
    start(rig)
    pitch, heading = np.radians([tilt, yaw]) / 2
    quat = (np.cos(heading) * np.cos(pitch), -np.sin(heading) * np.sin(pitch),
            np.cos(heading) * np.sin(pitch), np.sin(heading) * np.cos(pitch))
    rig.adapter.feedback = replace(rig.adapter.feedback, quat=quat)
    rig.adapter.abort('thrash_detected', request_damping=True)
    rig.adapter.stop_once()
    assert rig.adapter.abort_reason == 'thrash_detected'
    assert rig.adapter.command is None
    assert not rig.adapter.startup.status()['control_ready']
    if abs(tilt) <= 30:
        assert rig.adapter.automatic_soft_reset
        assert rig.transport.stop_calls == 0
        rig.adapter.soft_reset_policy.update.assert_called_once()
        assert rig.adapter.soft_reset_policy.update.call_args[0][2] == quat
        targets, _ = rig.transport.write_body.call_args[0]
        assert len(targets) == 29
        assert all(t['dq'] == t['tau'] == 0 for t in targets.values())
    else:
        assert rig.transport.stop_calls == 1
        rig.adapter.soft_reset_policy.update.assert_not_called()


@pytest.mark.parametrize('bad', [
    {'quat': ()}, {'quat': (0, 0, 0, 0)}, {'quat': (2, 0, 0, 0)},
    {'quat': (float('nan'), 0, 0, 0)}, {'gyro': ()},
    {'gyro': (0, float('inf'), 0)}, {'q': (0,)},
    {'received_at_secs': 9.79}, {'received_at_secs': 10.1},
])
def test_automatic_recovery_requires_valid_fresh_telemetry(rig, bad):
    start(rig)
    rig.adapter.feedback = replace(rig.adapter.feedback, **bad)
    rig.adapter.abort('thrash_detected')
    rig.adapter.stop_once()
    assert rig.transport.stop_calls == 1
    rig.adapter.soft_reset_policy.update.assert_not_called()


def test_automatic_recovery_survives_fault_echoes_and_submission_exit(rig):
    start(rig)
    rig.adapter.abort('thrash_detected')
    for _ in range(20):
        rig.now += 0.02
        rig.state()
        rig.adapter.abort('thrash_detected', request_damping=True)
        rig.adapter.abort('policy_container_exit')
        rig.adapter.submit(rig.command, rig.now + 0.1)
        rig.adapter.step()
        assert rig.adapter.command is None
        assert rig.adapter.last_worker_cycle_secs == rig.now
    assert rig.adapter.abort_reason == 'thrash_detected'
    assert rig.adapter.automatic_soft_reset
    assert rig.transport.stop_calls == 0
    assert rig.adapter.soft_reset_policy.update.call_count == 20
    rig.adapter.soft_reset_policy.reset.assert_called_once()
    rig.adapter.abort('controller_double_start')
    rig.adapter.stop_once()
    assert rig.transport.stop_calls == 1
    assert not rig.adapter.automatic_soft_reset


@pytest.mark.parametrize('failure', [
    'tilt', 'tilt_during_inference', 'stale', 'inference', 'write', 'watchdog',
])
def test_failed_automatic_recovery_falls_back_to_damping(rig, failure):
    start(rig)
    rig.adapter.abort('thrash_detected')
    rig.adapter.step()
    rig.transport.write_body.reset_mock()
    if failure == 'tilt':
        rig.adapter.feedback = replace(rig.adapter.feedback, quat=(0, 1, 0, 0))
    elif failure == 'tilt_during_inference':
        def tip(*args):
            rig.adapter.feedback = replace(rig.adapter.feedback, quat=(0, 1, 0, 0))
            return [0.0] * 29
        rig.adapter.soft_reset_policy.update.side_effect = tip
    elif failure == 'stale':
        rig.now += 0.3
    elif failure == 'inference':
        rig.adapter.soft_reset_policy.update.side_effect = RuntimeError('failed')
    elif failure == 'write':
        rig.transport.write_body.return_value = False
    else:
        rig.adapter.abort('hardware_output_worker_stalled')
    rig.adapter.step()
    rig.adapter.stop_once()
    assert rig.transport.stop_calls >= 1
    assert not rig.adapter.automatic_soft_reset
    if failure != 'write':
        rig.transport.write_body.assert_not_called()


@pytest.mark.parametrize('reason', [
    'controller_double_start', 'hardware_controller_double_start', 'hardware_shutdown',
    'hardware_write_failed', 'hardware_output_worker_stalled', 'hardware_output_loop_stall',
])
def test_unavailable_output_or_explicit_stop_never_enters_recovery(rig, reason):
    start(rig)
    rig.adapter.abort(reason)
    rig.adapter.stop_once()
    assert rig.transport.stop_calls == 1
    rig.adapter.soft_reset_policy.update.assert_not_called()


def test_automatic_fault_before_ownership_does_not_start_recovery(rig):
    assert rig.adapter.arm()
    rig.adapter.abort('thrash_detected')
    rig.adapter.stop_once()
    assert rig.transport.release_calls == 0
    assert rig.transport.stop_calls == 0
    rig.transport.write_body.assert_not_called()


def test_missing_recovery_policy_falls_back_to_damping(rig):
    start(rig)
    rig.adapter.soft_reset_policy = None
    rig.adapter.abort('thrash_detected')
    rig.adapter.stop_once()
    assert rig.transport.stop_calls == 1
