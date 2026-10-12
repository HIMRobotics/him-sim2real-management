"""Shared pre-command limits and parity with the hardware checker."""

import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from joint_checks import JointViolation, check_base_motion
from policy_safety.joint_rules import DEFAULT_JOINT_PROFILE, JointRules


@pytest.fixture
def rules(bundle):
    robot = json.loads((bundle / "robot.json").read_text())
    return JointRules(robot["joint_names"])


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("initial", [False, True])
def test_pose_gap_has_no_limit_for_every_joint(rules, sign, initial):
    for i, _name, limits in rules.entries:
        q, dq, tau, measured = (np.zeros(29) for _ in range(4))
        q[i] = limits.position_min_rad if sign < 0 else limits.position_max_rad
        measured[i] = limits.position_max_rad if sign < 0 else limits.position_min_rad
        np.testing.assert_array_equal(rules.check_command(q, dq, tau, measured, initial=initial), q)


@pytest.mark.parametrize("gap", [0.4, 0.501])
def test_hardware_and_simulation_agree(rules, gap, monkeypatch):
    root = Path(__file__).resolve().parents[3]
    monkeypatch.syspath_prepend(str(root))
    from robot_safety.hardware_adapter import G1HardwareAdapter
    from robot_safety.hardware_profile import HardwareProfile

    hardware = G1HardwareAdapter(HardwareProfile.load(DEFAULT_JOINT_PROFILE))
    q, dq, tau, measured = (np.zeros(29) for _ in range(4))
    q[15], measured[15] = gap, 0
    # Use a lower gain so this test isolates the command gap from torque.
    joints = list(hardware.profile.joints)
    joints[3] = replace(joints[3], kp=1)
    hardware.profile = replace(hardware.profile, joints=tuple(joints))
    hardware.command = {"q": q, "dq": dq, "tau": tau}
    hardware.feedback = SimpleNamespace(q=measured, dq=dq)
    hardware.last_output_secs = 0
    assert hardware.checked_targets(0.02)[15]["q"] == gap
    assert rules.check_command(q, dq, tau, measured)[15] == gap


@pytest.mark.parametrize("sign", [-1, 1])
def test_official_velocity_and_torque_boundaries(rules, sign):
    path = Path(__file__).resolve().parents[3] / "robot_safety/profiles/g1_type5.official_limits.json"
    ratings = json.loads(path.read_text())["joints"]
    for i, name, limits in rules.entries:
        assert limits.max_velocity_rad_secs == ratings[name]["velocity"]
        assert limits.max_torque_nm == ratings[name]["effort"]
        for field, method, reason in (
            (limits.max_velocity_rad_secs, rules.check_measured_velocity, "joint_velocity_limit"),
            (limits.max_torque_nm, rules.check_torque, "joint_torque_limit"),
        ):
            values = np.zeros(29)
            values[i] = sign * field
            method(values)
            values[i] += sign * 1e-6
            with pytest.raises(JointViolation, match=reason):
                method(values)


def test_desired_velocity_above_the_model_cap_is_clipped(rules):
    q, dq, tau, measured = (np.zeros(29) for _ in range(4))
    _index, name, limits = rules.entries[0]
    dq[0] = limits.max_velocity_rad_secs + 1
    warnings = {}
    rules.check_command(q, dq, tau, measured, warnings=warnings)
    assert dq[0] == limits.max_velocity_rad_secs
    assert warnings[name]["velocity_reason"] == "joint_velocity_clipped"
    assert warnings[name]["dq_proposed"] == limits.max_velocity_rad_secs + 1


def test_startup_velocity_and_finite_checks_remain(rules):
    q, dq, tau, measured = (np.zeros(29) for _ in range(4))
    dq[15] = rules.profile["initial_velocity_tolerance_rad_secs"] + 0.01
    with pytest.raises(ValueError, match="initial_joint_target_velocity_limit"):
        rules.check_command(q, dq, tau, measured, initial=True)
    q[15] = float("nan")
    with pytest.raises(ValueError, match="nonfinite"):
        rules.check_command(q, dq, tau, measured)


@pytest.mark.parametrize("side", ["position_min_rad", "position_max_rad"])
def test_position_clipping_warns_and_preserves_proposals(rules, side):
    for i, name, limits in rules.entries:
        bound = getattr(limits, side)
        q, dq, tau, measured = (np.zeros(29) for _ in range(4))
        measured[i] = bound
        q[i] = bound + (-0.02 if side == "position_min_rad" else 0.02)
        original = q.copy()
        warnings = {}
        clipped = rules.check_command(q, dq, tau, measured, initial=True, warnings=warnings)
        assert clipped[i] == bound
        np.testing.assert_array_equal(q, original)
        assert warnings[name]["q_proposed"] == q[i]
        assert warnings[name]["q_commanded"] == bound
        assert warnings[name]["count"] == 1
        rules.check_command(q, dq, tau, measured, warnings=warnings)
        assert warnings[name]["count"] == 2
        measured[i] = bound + (0.501 if side == "position_min_rad" else -0.501)
        assert rules.check_command(q, dq, tau, measured)[i] == bound


def test_body_limits_are_per_axis_and_tilt_is_optional():
    check_base_motion([6, 6, 6])
    with pytest.raises(JointViolation, match="base_angular_speed_limit"):
        check_base_motion([6.01, 0, 0])
    with pytest.raises(JointViolation, match="base_tilt_limit"):
        check_base_motion([0, 0, 0], [0, 1, 0, 0])


def test_packaged_profile_matches_standard_template():
    path = Path(__file__).resolve().parents[3] / "robot_safety/profiles/g1_type5.template.json"
    assert DEFAULT_JOINT_PROFILE.read_bytes() == path.read_bytes()
