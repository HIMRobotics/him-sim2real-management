"""Per-joint command rules matching the existing hardware adapter."""

from pathlib import Path

import numpy as np

from .adapter import PolicyViolation
from .config import positive, read_json, vector

DEFAULT_JOINT_PROFILE = Path(__file__).parent / "profiles/g1_type5.template.json"
UPPER_JOINTS = {
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    *(
        f"{side}_{joint}"
        for side in ("left", "right")
        for joint in (
            "shoulder_pitch",
            "shoulder_roll",
            "shoulder_yaw",
            "elbow",
            "wrist_roll",
            "wrist_pitch",
            "wrist_yaw",
        )
    ),
}


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
        if not UPPER_JOINTS <= set(data["joints"]):
            raise ValueError("joint profile must include all 17 arm/waist joints")
        if set(data["joints"]) - set(self.sdk_names):
            raise ValueError("joint profile contains joints absent from the robot")
        for key in (
            "initial_position_tolerance_rad",
            "initial_velocity_tolerance_rad_secs",
        ):
            positive(data[key], key)
        self.entries = []
        for name, limits in data["joints"].items():
            bounds = vector(
                [limits["position_min_rad"], limits["position_max_rad"]],
                2,
                f"{name} bounds",
            )
            if bounds[0] >= bounds[1]:
                raise ValueError(f"{name}: unordered position bounds")
            for key in (
                "max_velocity_rad_secs",
                "max_acceleration_rad_secs2",
                "max_tracking_error_rad",
            ):
                positive(limits[key], f"{name}.{key}")
            self.entries.append((self.sdk_names.index(name), name, limits))

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
            ],
            "semantics": "Velocity-change limits apply to commanded dq, not measured or position-derived acceleration. Profile gains and hardware ownership handover are not imported.",
        }

    @staticmethod
    def reject(reason, joint, value, limit, units):
        raise PolicyViolation(
            reason,
            {"joint": joint, "value": float(value), "limit": limit, "units": units},
        )

    def check_command(self, target, velocity, torque, measured, previous, dt):
        positive(dt, "command interval")
        for values in (target, velocity, torque, measured):
            if (
                np.asarray(values).shape != (len(self.names),)
                or not np.isfinite(values).all()
            ):
                raise PolicyViolation("nonfinite_or_invalid_joint_command")
        for i, name, limits in self.entries:
            q, dq, tau = float(target[i]), float(velocity[i]), float(torque[i])
            if not limits["position_min_rad"] <= q <= limits["position_max_rad"]:
                self.reject(
                    "joint_target_position_limit",
                    name,
                    q,
                    [limits["position_min_rad"], limits["position_max_rad"]],
                    "rad",
                )
            if abs(dq) > limits["max_velocity_rad_secs"]:
                self.reject(
                    "joint_target_velocity_limit",
                    name,
                    abs(dq),
                    limits["max_velocity_rad_secs"],
                    "rad/s",
                )
            if tau != 0:
                self.reject("nonzero_feedforward_torque", name, tau, 0.0, "Nm")
            error = abs(q - measured[i])
            if error > limits["max_tracking_error_rad"]:
                self.reject(
                    "joint_tracking_error_limit",
                    name,
                    error,
                    limits["max_tracking_error_rad"],
                    "rad",
                )
            if previous is None:
                if error > self.profile["initial_position_tolerance_rad"]:
                    self.reject(
                        "initial_joint_position_mismatch",
                        name,
                        error,
                        self.profile["initial_position_tolerance_rad"],
                        "rad",
                    )
                if abs(dq) > self.profile["initial_velocity_tolerance_rad_secs"]:
                    self.reject(
                        "initial_joint_target_velocity_limit",
                        name,
                        abs(dq),
                        self.profile["initial_velocity_tolerance_rad_secs"],
                        "rad/s",
                    )
            else:
                delta = abs(q - previous["target"][i])
                if delta > limits["max_velocity_rad_secs"] * dt:
                    self.reject(
                        "joint_position_target_change_limit",
                        name,
                        delta,
                        limits["max_velocity_rad_secs"] * dt,
                        "rad",
                    )
                delta = abs(dq - previous["velocity"][i])
                if delta > limits["max_acceleration_rad_secs2"] * dt:
                    self.reject(
                        "joint_velocity_target_change_limit",
                        name,
                        delta,
                        limits["max_acceleration_rad_secs2"] * dt,
                        "rad/s",
                    )

    def check_tracking(self, target, measured):
        for i, name, limits in self.entries:
            error = abs(float(target[i] - measured[i]))
            if error > limits["max_tracking_error_rad"]:
                self.reject(
                    "joint_tracking_error_limit",
                    name,
                    error,
                    limits["max_tracking_error_rad"],
                    "rad",
                )
