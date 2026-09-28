"""Shared command checks without SDK, ROS or simulator dependencies."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class JointHardwareLimits:
    """Joint bounds, PD gains and local movement tolerances."""

    position_min_rad: float
    position_max_rad: float
    kp: float
    kd: float
    max_velocity_rad_secs: float
    max_acceleration_rad_secs2: float
    max_tracking_error_rad: float
    max_torque_nm: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in self.__dict__.values()):
            raise ValueError("joint limits must be finite")
        if self.position_min_rad >= self.position_max_rad:
            raise ValueError("joint position bounds must be ordered")
        if any(
            v <= 0 for k, v in self.__dict__.items() if not k.startswith("position_")
        ):
            raise ValueError("joint gains and limits must be positive")


class JointViolation(ValueError):
    """A failed check with reportable joint and threshold details."""

    def __init__(self, reason, joint, value, limit, units):
        self.reason = reason
        super().__init__(f"{joint}: {reason} value={value} limit={limit} {units}")
        self.details = {
            "joint": joint,
            "value": float(value),
            "limit": limit,
            "units": units,
        }


def check_bound(reason, joint, value, limit, units):
    if not math.isfinite(value) or value > limit:
        raise JointViolation(reason, joint, value, limit, units)


def check_tracking(joint, limits, target, measured):
    check_bound(
        "joint_tracking_error_limit",
        joint,
        abs(target - measured),
        limits.max_tracking_error_rad,
        "rad",
    )


def check_command(
    joint,
    limits,
    q,
    dq,
    tau,
    measured,
    previous,
    dt,
    initial_position,
    initial_velocity,
):
    """Apply identical command rules at hardware and simulation boundaries."""
    if not all(math.isfinite(v) for v in (q, dq, tau, measured)):
        raise ValueError("nonfinite_or_invalid_joint_command")
    if not limits.position_min_rad <= q <= limits.position_max_rad:
        raise JointViolation(
            "joint_target_position_limit",
            joint,
            q,
            [limits.position_min_rad, limits.position_max_rad],
            "rad",
        )
    check_bound(
        "joint_target_velocity_limit",
        joint,
        abs(dq),
        limits.max_velocity_rad_secs,
        "rad/s",
    )
    check_bound("nonzero_feedforward_torque", joint, abs(tau), 0, "Nm")
    check_tracking(joint, limits, q, measured)
    if previous is None:
        check_bound(
            "initial_joint_position_mismatch",
            joint,
            abs(q - measured),
            initial_position,
            "rad",
        )
        check_bound(
            "initial_joint_target_velocity_limit",
            joint,
            abs(dq),
            initial_velocity,
            "rad/s",
        )
    else:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("command interval must be finite and positive")
        check_bound(
            "joint_position_target_change_limit",
            joint,
            abs(q - previous[0]),
            limits.max_velocity_rad_secs * dt,
            "rad",
        )
        check_bound(
            "joint_velocity_target_change_limit",
            joint,
            abs(dq - previous[1]),
            limits.max_acceleration_rad_secs2 * dt,
            "rad/s",
        )


def check_torque(joint, limits, torque):
    """Check estimated demand or measured effort against the same bound."""
    check_bound("joint_torque_limit", joint, abs(torque), limits.max_torque_nm, "Nm")
