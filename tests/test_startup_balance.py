"""Startup gating and failures with an inert balance policy."""

from unittest.mock import Mock, patch

import numpy as np
import pytest

from robot_safety.startup_balance import StartupBalance


def state():
    return dict(q=[0.0] * 29, dq=[0.0] * 29, quat=[1, 0, 0, 0], gyro=[0, 0, 0])


def submitted():
    return {key: np.full(29, value) for key, value in
            dict(q=0.5, dq=0.1, kp=20, kd=2, tau=0.1).items()}


def gate():
    return StartupBalance(Mock(update=Mock(return_value=np.full(15, 0.1))))


def advance(startup, end=2.1, begin=0.0, sample=None):
    sample = sample or state()
    for now in np.arange(begin, end + 0.001, 0.02):
        startup.command(round(float(now), 8), round(float(now), 8), sample, submitted())


def test_persistence_and_early_authorization_with_continuous_balance():
    startup = gate()
    advance(startup, 0.5)
    assert not startup.authorize(0.5)
    advance(startup, begin=0.52)
    assert startup.ready
    assert startup.handoff is None
    assert startup.policy.update.call_count > 90
    assert startup.authorize(2.1)
    origin = startup.command(2.1, 2.1, state(), submitted())
    for field, values in startup.origin.items():
        np.testing.assert_array_equal(origin[field], values)
    middle = startup.command(2.6, 2.6, state(), submitted())
    np.testing.assert_allclose(middle['kp'], (origin['kp'] + submitted()['kp']) / 2)
    finish = startup.command(3.1, 3.1, state(), submitted())
    np.testing.assert_allclose(finish['q'], submitted()['q'])
    assert startup.phase == 'running'


def test_instability_revokes_readiness_and_requires_new_window():
    startup = gate()
    advance(startup)
    moving = state()
    moving['dq'][0] = 0.7
    startup.command(2.12, 2.12, moving, submitted())
    assert not startup.ready
    assert not startup.authorize(2.12)
    advance(startup, 2.9, begin=2.14)
    assert not startup.ready
    advance(startup, 3.2, begin=2.92)
    assert startup.ready


def test_duplicate_samples_never_advance_window_and_stale_fails():
    startup = gate()
    advance(startup, 0.0)
    for now in [0.05, 0.1, 0.19]:
        startup.command(now, 0.0, state(), submitted())
    assert startup.stable_since is None
    assert not startup.ready
    with pytest.raises(ValueError, match='stale_telemetry'):
        startup.command(0.2, 0.0, state(), submitted())
    assert startup.handoff is None


@pytest.mark.parametrize('field,value', [('q', [float('nan')] * 29), ('gyro', []),
                                       ('quat', [0, 0, 0, 0]), ('dq', [float('inf')] * 29)])
def test_invalid_telemetry_prevents_handoff(field, value):
    sample = state()
    sample[field] = value
    with pytest.raises(ValueError):
        gate().command(0, 0, sample, submitted())


def test_timeout_never_authorizes():
    startup = gate()
    moving = state()
    moving['gyro'][0] = 0.4
    advance(startup, 29.98, sample=moving)
    with pytest.raises(ValueError, match='timeout'):
        startup.command(30, 30, state(), submitted())
    assert not startup.authorize(30)


def test_missing_model_and_inference_failure_propagate():
    with patch('robot_safety.startup_balance.BalancePolicy', side_effect=FileNotFoundError):
        with pytest.raises(FileNotFoundError):
            StartupBalance().prepare()
    startup = gate()
    startup.policy.update.side_effect = RuntimeError('inference failed')
    with pytest.raises(RuntimeError, match='inference failed'):
        startup.command(0, 0, state(), submitted())
    assert startup.handoff is None


def test_automatic_handoff_obeys_same_stability_gate():
    startup = gate()
    for now in np.arange(0, 2.1, 0.02):
        startup.command(float(now), float(now), state(), submitted(), automatic=True)
    assert startup.handoff >= 2.0
    assert startup.status()['control_ready']


def test_stable_wait_does_not_timeout_or_handoff_without_authorization():
    startup = gate()
    advance(startup, 65.0)
    assert startup.ready
    assert startup.phase == 'ready'
    assert startup.handoff is None
    assert not startup.status()['control_ready']
    assert startup.policy.update.call_count > 3000
    output = startup.command(65.02, 65.02, state(), submitted())
    np.testing.assert_allclose(output['q'][:15], 0.1)
    assert startup.authorize(65.02)


def test_losing_stability_after_long_wait_starts_recovery_deadline():
    startup = gate()
    advance(startup, 35.0)
    startup.timeout_seconds = 0.1
    moving = state()
    moving['gyro'][0] = 0.4
    startup.command(35.02, 35.02, moving, submitted())
    assert not startup.ready
    assert not startup.authorize(35.02)
    startup.command(35.04, 35.04, moving, submitted())
    with pytest.raises(ValueError, match='startup_balance_timeout'):
        startup.command(35.14, 35.14, moving, submitted())


def test_recovered_stability_clears_recovery_deadline():
    startup = gate()
    advance(startup)
    moving = state()
    moving['dq'][0] = 0.7
    startup.command(2.12, 2.12, moving, submitted())
    advance(startup, 65.0, begin=2.14)
    assert startup.ready
    assert startup.handoff is None


def test_stable_wait_retains_telemetry_deadline():
    startup = gate()
    advance(startup)
    with pytest.raises(ValueError, match='stale_telemetry'):
        startup.command(2.4, 2.1, state(), submitted())
    assert not startup.ready


def test_entry_starts_at_measured_pose_and_blends_live_balance_targets():
    startup = gate()
    sample = state()
    sample['q'] = [0.2] * 15 + [0.0] * 14
    first = startup.command(10, 10, sample, submitted())
    np.testing.assert_allclose(first['q'], sample['q'])
    assert startup.phase == 'entering_balance'
    assert not startup.authorize(10)
    startup.policy.update.return_value = np.full(15, 0.4)
    middle = startup.command(10.25, 10.25, sample, submitted())
    np.testing.assert_allclose(middle['q'][:15], 0.3)
    np.testing.assert_allclose(middle['q'][15:], 0.0)
    np.testing.assert_array_equal(middle['kp'], first['kp'])
    finish = startup.command(10.5, 10.5, sample, submitted())
    np.testing.assert_allclose(finish['q'][:15], 0.4)
    assert not startup.ready
    assert startup.phase == 'balancing'


def test_startup_eases_arms_to_sim_pose_before_handoff():
    startup = gate()
    sample = state()
    initial_arms = np.array([0.28, 0.1, 0, 0.98, 0, 0, 0,
                             0.28, -0.1, 0, 0.98, 0, 0, 0])
    sample['q'][15:] = initial_arms.tolist()
    ramp = 1.5 * 0.98 / startup.arm_speed
    previous = None
    for now in np.arange(0.0, 6.001, 0.02):
        now = round(float(now), 8)
        if ramp - 0.1 < now < ramp + 1.0:
            assert not startup.authorize(now)
        command = startup.command(now, now, sample, submitted())
        arms = command['q'][15:]
        if previous is not None:
            assert np.max(np.abs(arms - previous)) / 0.02 <= startup.arm_speed + 1e-9
        previous = arms
        if now >= ramp:
            np.testing.assert_allclose(arms, 0.0, atol=1e-12)
        np.testing.assert_array_equal(command['dq'][15:], 0)
        np.testing.assert_array_equal(command['tau'][15:], 0)
        # Later feedback must not redefine the ramp.
        sample['q'][15:] = (initial_arms + 0.02).tolist()
    assert startup.ready
    assert startup.handoff is None
    assert startup.authorize(6.0)
    command = startup.command(6.0, 6.0, sample, submitted())
    np.testing.assert_allclose(command['q'][15:], 0.0, atol=1e-12)
    command = startup.command(7.0, 7.0, sample, submitted())
    np.testing.assert_allclose(command['q'][15:], submitted()['q'][15:])


def test_arms_already_at_sim_pose_add_no_startup_delay():
    startup = gate()
    advance(startup)
    assert startup.arm_ramp_seconds == 0.0
    assert startup.ready


def test_entry_does_not_count_as_a_stable_window():
    startup = gate()
    startup.min_seconds = 0
    advance(startup, 1.0)
    assert not startup.ready
    assert startup.stable_since == pytest.approx(0.5)
    advance(startup, 1.5, begin=1.02)
    assert startup.ready
