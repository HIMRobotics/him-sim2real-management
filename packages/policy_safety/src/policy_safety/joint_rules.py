"""Per-joint command rules matching the existing hardware adapter."""

from pathlib import Path

import numpy as np
from joint_checks import (
    check_command,
    check_measured_velocity,
    check_torque,
    checked_targets,
    clip_position,
    joint_profile_entries,
)

from .adapter import PolicyViolation
from .config import read_json

DEFAULT_JOINT_PROFILE = Path(__file__).parent / "profiles/g1_type5.template.json"


class JointRules:
    """Validate selected limits without importing ROS or hardware code."""

    def __init__(self, joint_names, path=DEFAULT_JOINT_PROFILE):
        self.path = Path(path).resolve()
        self.profile = read_json(self.path)
        self.names = list(joint_names)
        self.sdk_names = [name.removesuffix("_joint") for name in self.names]
        self.entries = joint_profile_entries(self.profile, self.sdk_names)

    def process_command(self, command, q, dq, *, initial=False, warnings=None):
        """Return a complete command screened with hardware profile gains."""
        targets = checked_targets(
            command, self.entries, q, dq, initial=initial,
            initial_velocity=self.profile["initial_velocity_tolerance_rad_secs"],
            warnings=warnings,
        )
        return {field: np.array([targets[i][field] for i in range(len(self.names))])
                for field in ("q", "dq", "kp", "kd", "tau")}

    def coverage(self):
        covered = [self.names[i] for i, _, _ in self.entries]
        return {
            "profile_file": self.path.name,
            "covered_joints": covered,
            "uncovered_joints": [name for name in self.names if name not in covered],
            "profile_values": self.profile,
            "rules": [
                "position_clipping_with_warning",
                "target_velocity",
                "initial_target_velocity",
                "initial_target_velocity",
                "feedforward_torque",
                "torque_demand_and_applied_bounds",
            ],
            "semantics": "Before actuation: clip proposed position to bounds with a warning and enforce speed and torque limits. Command gap and response error are diagnostics only, including at startup. PD torque demand still uses position error.",
        }

    def clip_positions(self, q_proposed, warnings=None, bounds=None):
        """Clip to profile bounds, intersected with model bounds when supplied."""
        q_commanded = np.asarray(q_proposed, dtype=float).copy()
        if q_commanded.shape != (len(self.names),) or not np.isfinite(q_commanded).all():
            raise PolicyViolation("nonfinite_or_invalid_joint_command")
        for i, name, limits in self.entries:
            lower, upper = limits.position_min_rad, limits.position_max_rad
            if bounds is not None:
                lower, upper = max(lower, bounds[i, 0]), min(upper, bounds[i, 1])
                if lower > upper:
                    raise ValueError(f"disjoint position bounds: {name}")
            q_commanded[i] = clip_position(name, q_commanded[i], lower, upper, warnings)
        return q_commanded

    def check_command(self, q_proposed, velocity, torque, q_measured, *, initial=False, warnings=None):
        for values in (q_proposed, velocity, torque, q_measured):
            if np.asarray(values).shape != (len(self.names),) or not np.isfinite(values).all():
                raise PolicyViolation("nonfinite_or_invalid_joint_command")
        q_commanded = self.clip_positions(q_proposed, warnings)
        for i, name, limits in self.entries:
            check_command(name, limits, q_commanded[i], velocity[i], torque[i], q_measured[i],
                          initial,
                          self.profile["initial_velocity_tolerance_rad_secs"])
        return q_commanded

    def check_measured_velocity(self, measured_dq):
        for i, name, limits in self.entries:
            check_measured_velocity(name, limits, measured_dq[i])

    def check_torque(self, torque, *, kind=None):
        for i, name, limits in self.entries:
            check_torque(name, limits, torque[i], kind=kind)
