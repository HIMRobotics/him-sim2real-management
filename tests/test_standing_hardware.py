"""Standing guards and mode transitions through fake hardware only."""

import math
from dataclasses import replace
from types import SimpleNamespace

import pytest

from robot_safety.hardware_constants import STANDING_HEIGHT_MIN_M
from tests.test_standing import pose_at_height, squat_pose
from tests.test_startup_hardware import handoff_policy, start
from tests.test_startup_hardware import rig as rig  # noqa: PLC0414


@pytest.mark.parametrize('offset, allowed', [(-1e-6, False), (0, True), (1e-6, True)])
def test_height_boundary_controls_policy_recovery(rig, offset, allowed):
    start(rig)
    handoff_policy(rig)
    rig.adapter.feedback = replace(
        rig.adapter.feedback, q=pose_at_height(STANDING_HEIGHT_MIN_M + offset))
    rig.adapter.abort('thrash_detected')
    assert rig.adapter.automatic_soft_reset is allowed
    assert rig.adapter.hold_failed is not allowed
    rig.transport.write_body.reset_mock()
    rig.adapter.stop_once()
    if allowed:
        rig.adapter.soft_reset_policy.update.assert_called_once()
        rig.transport.write_body.assert_called_once()
        assert rig.transport.stop_calls == 0
    else:
        assert rig.transport.stop_calls == 1
        rig.adapter.soft_reset_policy.update.assert_not_called()


@pytest.mark.parametrize('launched', [False, True])
@pytest.mark.parametrize('failure', ['squat', 'seated', 'tilt'])
def test_feedback_posture_violation_damps_without_recovery(rig, launched, failure):
    start(rig)
    if launched:
        handoff_policy(rig)
    q = squat_pose() if failure == 'squat' else (0,) * 29
    quat = (1, 0, 0, 0)
    if failure == 'seated':
        q = tuple([-math.pi / 2, 0, 0, math.pi / 2, 0, 0] * 2 + [0] * 17)
    elif failure == 'tilt':
        angle = math.radians(30.01) / 2
        quat = (math.cos(angle), 0, math.sin(angle), 0)
    rig.transport.write_body.reset_mock()
    rig.adapter.receive_state(SimpleNamespace(
        motor_state=[SimpleNamespace(q=v, dq=0, temperature=(25, 25), tau_est=0) for v in q],
        imu_state=SimpleNamespace(quaternion=quat, gyroscope=(0, 0, 0)),
        mode_machine=5, wireless_remote=[0] * 40))
    assert rig.adapter.abort_reason == 'hardware_not_standing'
    assert rig.adapter.hold_failed
    assert rig.transport.body_cleared
    rig.adapter.step()
    rig.adapter.stop_once()
    rig.transport.write_body.assert_not_called()
    rig.adapter.soft_reset_policy.update.assert_not_called()
    assert rig.transport.stop_calls == 1


@pytest.mark.parametrize('launched', [False, True])
def test_same_upright_fault_damps_startup_but_recovers_policy(rig, launched):
    start(rig)
    if launched:
        handoff_policy(rig)
    rig.adapter.abort('thrash_detected')
    rig.adapter.stop_once()
    assert rig.adapter.automatic_soft_reset is launched
    assert rig.transport.stop_calls == (0 if launched else 1)
    assert rig.adapter.soft_reset_policy.update.call_count == int(launched)


@pytest.mark.parametrize('manual', [False, True])
@pytest.mark.parametrize('during_inference', [False, True])
def test_recovery_collapse_prevents_next_write(rig, manual, during_inference):
    start(rig)
    handoff_policy(rig)
    if manual:
        rig.press_f3_twice()
    else:
        rig.adapter.abort('thrash_detected')
    rig.adapter.step()
    rig.transport.write_body.reset_mock()

    def collapse(*args):
        rig.adapter.feedback = replace(rig.adapter.feedback, q=squat_pose())
        return [0.] * 29

    if during_inference:
        rig.adapter.soft_reset_policy.update.side_effect = collapse
    else:
        collapse()
    rig.adapter.step()
    rig.adapter.stop_once()
    rig.transport.write_body.assert_not_called()
    assert rig.transport.stop_calls >= 1
    assert not rig.adapter.walk_stop
    assert not rig.adapter.automatic_soft_reset


def test_low_startup_pose_never_releases_or_writes(rig):
    assert rig.adapter.arm()
    rig.adapter.feedback = replace(rig.adapter.feedback, q=squat_pose())
    rig.adapter.submit(rig.command, rig.now + 0.1)
    rig.adapter.step()
    assert 'hardware_not_standing' in rig.adapter.abort_reason
    assert rig.transport.release_calls == 0
    rig.transport.write_body.assert_not_called()


def test_manual_recovery_rejects_low_pose_before_releasing_stock(rig):
    rig.press_f3_twice()
    rig.adapter.feedback = replace(rig.adapter.feedback, q=squat_pose())
    rig.adapter.stop_once()
    assert rig.transport.release_calls == 0
    rig.transport.write_body.assert_not_called()
    rig.adapter.soft_reset_policy.update.assert_not_called()
    assert rig.transport.stop_calls == 1


def test_recovery_feedback_collapse_clears_command_before_next_policy_tick(rig):
    start(rig)
    handoff_policy(rig)
    rig.adapter.abort('thrash_detected')
    rig.adapter.step()
    rig.transport.body_cleared = False
    rig.adapter.receive_state(SimpleNamespace(
        motor_state=[SimpleNamespace(q=v, dq=0, temperature=(25, 25), tau_est=0)
                     for v in squat_pose()],
        imu_state=SimpleNamespace(quaternion=(1, 0, 0, 0), gyroscope=(0, 0, 0)),
        mode_machine=5, wireless_remote=[0] * 40))
    assert rig.transport.body_cleared
    assert rig.adapter.hold_failed
    assert not rig.adapter.walk_stop
    rig.adapter.stop_once()
    assert rig.transport.stop_calls == 1


def test_startup_collapse_during_inference_prevents_write(rig):
    start(rig)

    def collapse(*args):
        rig.adapter.feedback = replace(rig.adapter.feedback, q=squat_pose())
        return [0.] * 15

    rig.adapter.balance_policy.update.side_effect = collapse
    rig.now += 0.02
    rig.state()
    rig.transport.write_body.reset_mock()
    rig.adapter.submit(rig.command, rig.now + 0.1)
    rig.adapter.step()
    rig.adapter.stop_once()
    rig.transport.write_body.assert_not_called()
    rig.adapter.soft_reset_policy.update.assert_not_called()
    assert rig.transport.stop_calls == 1
