"""Torque-aware targets retain the configured physical ceilings."""

from dataclasses import replace

import pytest

from robot_safety.joint_checks import (
    JointHardwareLimits, checked_targets, limit_torque_target, torque_demand,
)


LIMITS = JointHardwareLimits(-0.87267, 0.5236, 40, 2, 30, 35)


def test_reported_ankle_overshoot_is_limited_without_raising_ceiling():
    q, dq = -0.4587, 0.0065
    demand = 35.25107482424937
    target = q + (demand + LIMITS.kd * dq) / LIMITS.kp
    warnings = {}
    output = limit_torque_target('ankle', LIMITS, target, 0, 0, q, dq, 0.9, warnings)
    assert torque_demand(output, 0, 40, 2, 0, q, dq) == pytest.approx(31.5)
    assert target - output == pytest.approx((demand - 31.5) / 40)
    assert LIMITS.max_torque_nm == 35
    assert warnings['ankle']['torque_target_clipping']['active']


@pytest.mark.parametrize('q,dq,target,velocity,tau', [
    (-0.4, 0, 0.52, 0, 0), (0.3, 0, -0.87, 0, 0),
    (-0.2, -3, 0.52, 0.1, 1), (0.2, 3, -0.87, -0.1, -1),
])
def test_limiter_accounts_for_velocity_and_feedforward(q, dq, target, velocity, tau):
    output = limit_torque_target('ankle', LIMITS, target, velocity, tau, q, dq, 0.9)
    assert LIMITS.position_min_rad <= output <= LIMITS.position_max_rad
    assert abs(torque_demand(output, velocity, 40, 2, tau, q, dq)) <= 31.5 + 1e-10


def test_normal_target_is_unchanged_and_diagnostics_clear_activity():
    warnings = {}
    limit_torque_target('ankle', LIMITS, 0.52, 0, 0, -0.4, 0, 0.9, warnings)
    assert limit_torque_target('ankle', LIMITS, -0.2, 0, 0, -0.3, 0, 0.9, warnings) == -0.2
    assert not warnings['ankle']['torque_target_clipping']['active']
    assert warnings['ankle']['torque_target_clipping']['count'] == 1


def test_impossible_effort_interval_faults_instead_of_breaking_position_bounds():
    with pytest.raises(ValueError, match='no_feasible_torque_target'):
        limit_torque_target('ankle', LIMITS, 0, 0, 0, 0.5, 30, 0.9)


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_nonfinite_proposals_are_not_hidden(bad):
    with pytest.raises(ValueError, match='nonfinite'):
        limit_torque_target('ankle', LIMITS, bad, 0, 0, 0, 0, 0.9)


def test_feedforward_violation_still_faults():
    command = dict(q=[0], dq=[0], tau=[36])
    with pytest.raises(ValueError, match='joint_feedforward_torque_limit'):
        checked_targets(command, [(0, 'ankle', LIMITS)], [0], [0],
                        initial=False, initial_velocity=0.05, torque_target_fraction=0.9)


def test_actual_gain_changes_are_used():
    command = dict(q=[0.5], dq=[0], tau=[0])
    limits = replace(LIMITS, kp=80)
    result = checked_targets(command, [(0, 'ankle', limits)], [-0.4], [0],
                             initial=False, initial_velocity=0.05, torque_target_fraction=0.9)
    assert result[0]['kp'] == 80
    assert torque_demand(result[0]['q'], 0, 80, 2, 0, -0.4, 0) == pytest.approx(31.5)
