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
def test_command_gap_boundaries_for_every_joint(rules, sign):
    for i, name, limits in rules.entries:
        q_proposed, dq, tau, q_measured = (np.zeros(29) for _ in range(4))
        q_proposed[i] = 0.1
        q_measured[i] = q_proposed[i] - sign * 0.5
        rules.check_command(q_proposed, dq, tau, q_measured)
        q_measured[i] -= sign * 1e-6
        with pytest.raises(JointViolation, match="joint_command_gap_limit") as error:
            rules.check_command(q_proposed, dq, tau, q_measured)
        assert error.value.details["joint"] == name
        assert error.value.details["limit"] == 0.5


def test_gap_uses_current_measurement_not_previous_command(rules):
    q, dq, tau, measured = (np.zeros(29) for _ in range(4))
    q[15], measured[15] = 0.4, 0.0
    rules.check_command(q, dq, tau, measured)
    # The same command must be rejected if the joint moves away from it.
    measured[15] = -0.11
    with pytest.raises(JointViolation, match="joint_command_gap_limit"):
        rules.check_command(q, dq, tau, measured)
    # A large change of command is allowed when close to the actual position.
    q[15], measured[15] = -0.4, -0.3
    rules.check_command(q, dq, tau, measured)


@pytest.mark.parametrize("gap", [0.4, 0.501])
def test_hardware_and_simulation_agree(rules, gap, monkeypatch):
    root = Path(__file__).resolve().parents[3]
    monkeypatch.syspath_prepend(str(root))
    from robot_safety.hardware_adapter import G1ArmAdapter
    from robot_safety.hardware_profile import HardwareProfile

    hardware = G1ArmAdapter(HardwareProfile.load(DEFAULT_JOINT_PROFILE))
    q, dq, tau, measured = (np.zeros(29) for _ in range(4))
    q[15], measured[15] = gap, 0
    # Use a lower gain so this test isolates the command gap from torque.
    joints = list(hardware.profile.joints)
    joints[3] = replace(joints[3], kp=1)
    hardware.profile = replace(hardware.profile, joints=tuple(joints))
    hardware.command = {"q": q, "dq": dq, "tau": tau}
    hardware.feedback = SimpleNamespace(q=measured, dq=dq)
    hardware.last_output_secs = 0
    if gap <= 0.5:
        hardware.checked_targets(0.02)
        rules.check_command(q, dq, tau, measured)
    else:
        with pytest.raises(ValueError, match="joint_command_gap_limit"):
            hardware.checked_targets(0.02)
        with pytest.raises(JointViolation, match="joint_command_gap_limit"):
            rules.check_command(q, dq, tau, measured)


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


def test_position_initial_and_finite_checks_remain(rules):
    q, dq, tau, measured = (np.zeros(29) for _ in range(4))
    for value, initial, reason in ((2.8, False, "joint_target_position_limit"),
                                   (0.031, True, "initial_joint_position_mismatch"),
                                   (float("nan"), False, "nonfinite")):
        q[15] = value
        with pytest.raises(ValueError, match=reason):
            rules.check_command(q, dq, tau, measured, initial=initial)


def test_body_limits_are_per_axis_and_tilt_is_optional():
    check_base_motion([6, 6, 6])
    with pytest.raises(JointViolation, match="base_angular_speed_limit"):
        check_base_motion([6.01, 0, 0])
    with pytest.raises(JointViolation, match="base_tilt_limit"):
        check_base_motion([0, 0, 0], [0, 1, 0, 0])


def test_packaged_profile_matches_standard_template():
    path = Path(__file__).resolve().parents[3] / "robot_safety/profiles/g1_type5.template.json"
    assert DEFAULT_JOINT_PROFILE.read_bytes() == path.read_bytes()
