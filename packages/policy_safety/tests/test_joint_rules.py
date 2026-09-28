"""Exercise joint-rule boundaries and parity with the hardware checker."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from policy_safety.adapter import PolicyViolation
from policy_safety.joint_rules import DEFAULT_JOINT_PROFILE, JointRules


@pytest.fixture
def rules(bundle):
    robot = json.loads((bundle / "robot.json").read_text())
    return JointRules(robot["joint_names"])


def command_case(rules, case):
    q, dq, tau, measured = (np.zeros(29) for _ in range(4))
    previous = {"target": q.copy(), "velocity": dq.copy()}
    i = rules.sdk_names.index("left_shoulder_pitch")
    if case == "position":
        q[i] = 2.7
        measured[i] = q[i]
    elif case == "velocity":
        dq[i] = 38
    elif case == "position_delta":
        q[i] = 0.75
        measured[i] = q[i]
    elif case == "velocity_delta":
        dq[i] = 0.021
    elif case == "tracking":
        q[i] = 0.31
    elif case == "initial_position":
        q[i] = 0.031
        previous = None
    elif case == "initial_velocity":
        dq[i] = 0.051
        previous = None
    elif case == "torque":
        tau[i] = 0.1
    elif case == "finite":
        q[i] = float("nan")
    return q, dq, tau, measured, previous


@pytest.mark.parametrize(
    ("case", "reason"),
    [
        ("position", "joint_target_position_limit"),
        ("velocity", "joint_target_velocity_limit"),
        ("position_delta", "joint_position_target_change_limit"),
        ("velocity_delta", "joint_velocity_target_change_limit"),
        ("tracking", "joint_tracking_error_limit"),
        ("initial_position", "initial_joint_position_mismatch"),
        ("initial_velocity", "initial_joint_target_velocity_limit"),
        ("torque", "nonzero_feedforward_torque"),
        ("finite", "nonfinite_or_invalid_joint_command"),
    ],
)
def test_each_rule_rejects(rules, case, reason):
    with pytest.raises(PolicyViolation, match=reason) as failure:
        rules.check_command(*command_case(rules, case), 0.02)
    if case != "finite":
        assert failure.value.details["joint"] == "left_shoulder_pitch"
        assert "limit" in failure.value.details


@pytest.mark.parametrize(
    "case",
    [
        "valid",
        "position",
        "velocity",
        "position_delta",
        "velocity_delta",
        "tracking",
        "initial_position",
        "initial_velocity",
        "torque",
        "finite",
    ],
)
def test_hardware_checker_acceptance_matches(rules, case, monkeypatch):
    root = Path(__file__).resolve().parents[3]
    if not (root / "robot_safety").is_dir():
        pytest.skip("hardware parity test requires the repository")
    monkeypatch.syspath_prepend(str(root))
    from robot_safety.hardware_adapter import G1ArmAdapter
    from robot_safety.hardware_constants import G1_29_JOINT_NAMES
    from robot_safety.hardware_profile import HardwareProfile

    assert list(G1_29_JOINT_NAMES) == rules.sdk_names
    q, dq, tau, measured, previous = command_case(rules, case)
    hardware = G1ArmAdapter(
        HardwareProfile.load(root / "robot_safety/profiles/g1_type5.template.json")
    )
    hardware.command = {"q": q, "dq": dq, "tau": tau}
    hardware.feedback = SimpleNamespace(q=measured, dq=np.zeros(29))
    hardware.previous_targets = None if previous is None else previous["target"]
    hardware.previous_velocities = None if previous is None else previous["velocity"]
    hardware.last_output_secs = None if previous is None else 0.0
    expected = case == "valid"
    try:
        hardware.checked_targets(0.02)
        accepted = True
    except ValueError:
        accepted = False
    assert accepted == expected
    if accepted:
        rules.check_command(q, dq, tau, measured, previous, 0.02)
    else:
        with pytest.raises(PolicyViolation):
            rules.check_command(q, dq, tau, measured, previous, 0.02)


def test_all_joint_velocity_change_boundaries(rules):
    for i, name, limits in rules.entries:
        q, dq, tau, measured, previous = command_case(rules, "valid")
        dq[i] = limits.max_acceleration_rad_secs2 * 0.02
        rules.check_command(q, dq, tau, measured, previous, 0.02)
        dq[i] += 1e-6
        with pytest.raises(
            PolicyViolation, match="joint_velocity_target_change_limit"
        ) as failure:
            rules.check_command(q, dq, tau, measured, previous, 0.02)
        assert failure.value.details["joint"] == name


def test_tracking_boundaries_and_coverage(rules):
    for i, name, limits in rules.entries:
        q, dq, tau, measured, previous = command_case(rules, "valid")
        q[i] = limits.max_tracking_error_rad
        previous["target"] = q.copy()
        rules.check_command(q, dq, tau, measured, previous, 0.02)
        rules.check_tracking(q, measured)
        q[i] += 1e-6
        with pytest.raises(
            PolicyViolation, match="joint_tracking_error_limit"
        ) as failure:
            rules.check_tracking(q, measured)
        assert failure.value.details["joint"] == name
    coverage = rules.coverage()
    assert len(coverage["covered_joints"]) == 29
    assert len(coverage["uncovered_joints"]) == 0


def test_packaged_profile_matches_standard_template():
    path = (
        Path(__file__).resolve().parents[3]
        / "robot_safety/profiles/g1_type5.template.json"
    )
    if not path.is_file():
        pytest.skip("source template is outside standalone distribution")
    assert DEFAULT_JOINT_PROFILE.read_bytes() == path.read_bytes()


def test_invalid_or_missing_joint_limits_rejected(rules, tmp_path):
    profile = rules.profile
    del profile["joints"]["waist_yaw"]
    path = tmp_path / "incomplete.json"
    path.write_text(json.dumps(profile))
    with pytest.raises(ValueError, match="all 29"):
        JointRules(rules.names, path)


@pytest.mark.parametrize("sign", [-1, 1])
def test_all_joint_torque_boundaries(rules, sign):
    for i, name, limits in rules.entries:
        torque = np.zeros(29)
        torque[i] = sign * limits.max_torque_nm
        rules.check_torque(torque)
        torque[i] += sign * 1e-6
        with pytest.raises(PolicyViolation, match="joint_torque_limit") as failure:
            rules.check_torque(torque)
        assert failure.value.details["joint"] == name


def test_leg_position_delta(rules):
    for i, name, limits in rules.entries:
        if i >= 12:
            continue
        q, dq, tau, measured, previous = command_case(rules, "valid")
        q[i] = measured[i] = limits.max_velocity_rad_secs * 0.001 + 1e-6
        with pytest.raises(
            PolicyViolation, match="joint_position_target_change_limit"
        ) as failure:
            rules.check_command(q, dq, tau, measured, previous, 0.001)
        assert failure.value.details["joint"] == name
