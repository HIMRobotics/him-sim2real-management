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
    max_torque_nm: float
    max_command_gap_rad: float

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


def clip_position(joint, q_proposed, lower, upper, warnings=None):
    """Bound a finite proposal and summarize clipping per joint."""
    if not math.isfinite(q_proposed):
        raise ValueError("nonfinite_or_invalid_joint_command")
    q_commanded = max(lower, min(upper, q_proposed))
    if q_commanded != q_proposed and warnings is not None:
        previous = warnings.get(joint, {})
        warnings[joint] = {
            "reason": "joint_position_clipped",
            "q_proposed": float(q_proposed),
            "q_commanded": float(q_commanded),
            "bounds_rad": [float(lower), float(upper)],
            "count": previous.get("count", 0) + 1,
            "max_correction_rad": max(previous.get("max_correction_rad", 0),
                                      abs(float(q_proposed - q_commanded))),
        }
    return q_commanded


BASE_ANGULAR_SPEED_LIMIT_RAD_S = 6.0
BASE_TILT_LIMIT_RAD = 1.0
MOTOR_WINDING_LIMIT_C = 120.0
MOTOR_CASING_LIMIT_C = 85.0
MIN_BATTERY_PERCENT = 20.0


def check_base_motion(gyro, quat=None):
    """Check each gyro axis and, when requested, tilt from upright."""
    if len(gyro) != 3:
        raise ValueError("invalid_imu")
    for axis, speed in enumerate(gyro):
        check_bound("base_angular_speed_limit", str(axis), abs(speed),
                    BASE_ANGULAR_SPEED_LIMIT_RAD_S, "rad/s")
    if quat is not None:
        if len(quat) != 4 or not all(math.isfinite(v) for v in quat):
            raise ValueError("invalid_imu")
        norm = sum(v * v for v in quat)
        if norm <= 0:
            raise ValueError("invalid_imu")
        upright = 1 - 2 * (quat[1] ** 2 + quat[2] ** 2) / norm
        tilt = math.acos(max(-1.0, min(1.0, upright)))
        check_bound("base_tilt_limit", "base", tilt, BASE_TILT_LIMIT_RAD, "rad")


def check_measured_velocity(joint, limits, measured_dq):
    """Reject a joint that is spinning faster than its allowed speed."""
    check_bound(
        "joint_velocity_limit",
        joint,
        abs(measured_dq),
        limits.max_velocity_rad_secs,
        "rad/s",
    )


def check_command(
    joint, limits, q_proposed, dq_proposed, tau, q_measured,
    initial, initial_position, initial_velocity, warnings=None,
):
    """Validate a proposal against current measurements before actuation."""
    if not all(math.isfinite(v) for v in (q_proposed, dq_proposed, tau, q_measured)):
        raise ValueError("nonfinite_or_invalid_joint_command")
    q_commanded = clip_position(joint, q_proposed, limits.position_min_rad,
                               limits.position_max_rad, warnings)
    check_bound("joint_target_velocity_limit", joint, abs(dq_proposed),
                limits.max_velocity_rad_secs, "rad/s")
    check_bound("joint_feedforward_torque_limit", joint, abs(tau), limits.max_torque_nm, "Nm")
    command_gap = q_commanded - q_measured
    check_bound("joint_command_gap_limit", joint, abs(command_gap),
                limits.max_command_gap_rad, "rad")
    if initial:
        check_bound("initial_joint_position_mismatch", joint, abs(command_gap),
                    initial_position, "rad")
        check_bound("initial_joint_target_velocity_limit", joint, abs(dq_proposed),
                    initial_velocity, "rad/s")
    return q_commanded


def check_torque(joint, limits, torque):
    """Check estimated demand or measured effort against the same bound."""
    check_bound("joint_torque_limit", joint, abs(torque), limits.max_torque_nm, "Nm")
