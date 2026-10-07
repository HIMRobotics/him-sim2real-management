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
    assert startup.stable_since == 0
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
