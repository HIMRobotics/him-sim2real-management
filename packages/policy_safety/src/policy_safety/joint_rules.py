"""Per-joint command rules matching the existing hardware adapter."""

from pathlib import Path

import numpy as np
from joint_checks import (
    JointHardwareLimits,
    JointViolation,
    check_command,
    check_torque,
    check_tracking,
)

from .adapter import PolicyViolation
from .config import positive, read_json

DEFAULT_JOINT_PROFILE = Path(__file__).parent / "profiles/g1_type5.template.json"


class JointRules:
    """Validate selected limits without importing ROS or hardware code."""

    def __init__(self, joint_names, path=DEFAULT_JOINT_PROFILE):
        self.path = Path(path).resolve()
        self.profile = read_json(self.path)
        self.names = list(joint_names)
        self.sdk_names = [name.removesuffix("_joint") for name in self.names]
        if len(set(self.sdk_names)) != len(self.sdk_names):
            raise ValueError("ambiguous joint names in movement-rule mapping")
        data = self.profile
        if data["expected_machine_type"] != 5:
            raise ValueError("joint profile must target G1 machine type 5")
        if set(data["joints"]) != set(self.sdk_names):
            raise ValueError("joint profile must include all 29 robot joints")
        for key in (
            "initial_position_tolerance_rad",
            "initial_velocity_tolerance_rad_secs",
        ):
            positive(data[key], key)
        self.entries = [
            (self.sdk_names.index(name), name, JointHardwareLimits(**limits))
            for name, limits in data["joints"].items()
        ]

    def coverage(self):
        covered = [self.names[i] for i, _, _ in self.entries]
        return {
            "profile_file": self.path.name,
            "covered_joints": covered,
            "uncovered_joints": [name for name in self.names if name not in covered],
            "profile_values": self.profile,
            "rules": [
                "target_position",
                "target_velocity",
                "position_target_change",
                "velocity_target_change",
                "tracking_error",
                "initial_target_mismatch",
                "initial_target_velocity",
                "zero_feedforward_torque",
                "torque_demand_and_applied_bounds",
            ],
            "semantics": "Velocity-change limits apply to commanded dq, not measured or position-derived acceleration. Simulation validates the initial hold, then checks policy motion against that baseline. Profile gains and hardware ownership handover are not imported.",
        }

    def check_command(self, target, velocity, torque, measured, previous, dt):
        for values in (target, velocity, torque, measured):
            if (
                np.asarray(values).shape != (len(self.names),)
                or not np.isfinite(values).all()
            ):
                raise PolicyViolation("nonfinite_or_invalid_joint_command")
        try:
            for i, name, limits in self.entries:
                prior = (
                    None
                    if previous is None
                    else (previous["target"][i], previous["velocity"][i])
                )
                check_command(
                    name,
                    limits,
                    target[i],
                    velocity[i],
                    torque[i],
                    measured[i],
                    prior,
                    dt,
                    self.profile["initial_position_tolerance_rad"],
                    self.profile["initial_velocity_tolerance_rad_secs"],
                )
        except JointViolation as exc:
            raise PolicyViolation(exc.reason, exc.details) from exc

    def check_tracking(self, target, measured):
        try:
            for i, name, limits in self.entries:
                check_tracking(name, limits, target[i], measured[i])
        except JointViolation as exc:
            raise PolicyViolation(exc.reason, exc.details) from exc

    def check_torque(self, torque):
        try:
            for i, name, limits in self.entries:
                check_torque(name, limits, torque[i])
        except JointViolation as exc:
            raise PolicyViolation(exc.reason, exc.details) from exc
