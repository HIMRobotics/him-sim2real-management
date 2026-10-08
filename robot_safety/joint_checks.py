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

    def __post_init__(self):
        if not all(math.isfinite(v) for v in self.__dict__.values()):
            raise ValueError("joint limits must be finite")
        if self.position_min_rad >= self.position_max_rad:
            raise ValueError("joint position bounds must be ordered")
        if any(
            v <= 0 for k, v in self.__dict__.items() if not k.startswith("position_")
        ):
            raise ValueError("joint gains and limits must be positive")


def joint_profile_entries(data, names):
    """Validate movement settings and map profile joints to command indices."""
    if data["expected_machine_type"] != 5:
        raise ValueError("joint profile must target G1 machine type 5")
    if len(names) != 29 or len(set(names)) != 29 or set(data["joints"]) != set(names):
        raise ValueError("joint profile must include all 29 robot joints")
    value = data["initial_velocity_tolerance_rad_secs"]
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError("initial_velocity_tolerance_rad_secs must be finite and positive")
    return tuple((i, name, JointHardwareLimits(**data["joints"][name]))
                 for i, name in enumerate(names))


def torque_demand(q_des, dq_des, kp, kd, tau, q, dq):
    """PD effort, usable with scalars or arrays."""
    return kp * (q_des - q) + kd * (dq_des - dq) + tau


# Leave headroom for measurement changes between output cycles.
TORQUE_TARGET_FRACTION = 0.9


def limit_torque_target(joint, limits, position, velocity, tau, q, dq,
                        fraction, warnings=None):
    """Project a position target into the feasible PD effort interval."""
    if not all(math.isfinite(v) for v in (position, velocity, tau, q, dq, fraction)):
        raise ValueError("nonfinite_or_invalid_joint_command")
    if not 0 < fraction < 1:
        raise ValueError("torque target fraction must be between zero and one")
    budget = fraction * limits.max_torque_nm
    offset = limits.kd * (velocity - dq) + tau
    lower = max(limits.position_min_rad, q + (-budget - offset) / limits.kp)
    upper = min(limits.position_max_rad, q + (budget - offset) / limits.kp)
    if lower > upper:
        raise ValueError("no_feasible_torque_target:" + joint)
    limited = max(lower, min(upper, position))
    requested = torque_demand(position, velocity, limits.kp, limits.kd, tau, q, dq)
    previous = (warnings or {}).get(joint, {}).get("torque_target_clipping", {})
    if limited != position:
        _warn(warnings, joint, {"torque_target_clipping": {
            "active": True, "q_requested": float(position), "q_commanded": float(limited),
            "requested_torque_nm": float(requested), "command_budget_nm": float(budget),
            "hard_limit_nm": limits.max_torque_nm,
            "count": previous.get("count", 0) + 1,
            "max_correction_rad": max(previous.get("max_correction_rad", 0),
                                      abs(limited - position)),
        }})
    elif previous:
        previous["active"] = False
    return limited


def checked_targets(command, entries, q, dq, *, initial,
                    initial_velocity, warnings=None, enforce_torque=True,
                    torque_target_fraction=None):
    """Clip and screen targets using the profile gains that will be applied."""
    targets = {}
    for index, joint, limits in entries:
        position, velocity = check_command(
            joint, limits, command["q"][index], command["dq"][index],
            command["tau"][index], q[index], initial,
            initial_velocity, warnings, enforce_torque=enforce_torque,
        )
        if torque_target_fraction is not None:
            position = limit_torque_target(
                joint, limits, position, velocity, command["tau"][index],
                q[index], dq[index], torque_target_fraction, warnings)
        target = {"q": position, "dq": velocity, "kp": limits.kp,
                  "kd": limits.kd, "tau": command["tau"][index]}
        demand = torque_demand(
            position, target["dq"], target["kp"], target["kd"], target["tau"],
            q[index], dq[index],
        )
        if enforce_torque:
            check_torque(joint, limits, demand, kind="PD demand")
        else:
            note_limit(warnings, joint, "joint_torque_limit", abs(demand),
                       limits.max_torque_nm, "Nm", torque_kind="PD demand")
        targets[index] = target
    return targets


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


def _warn(warnings, joint, fields):
    if warnings is None:
        return
    previous = warnings.setdefault(joint, {})
    previous.update(fields)


def clip_position(joint, q_proposed, lower, upper, warnings=None):
    """Bound a finite proposal and summarize clipping per joint."""
    if not math.isfinite(q_proposed):
        raise ValueError("nonfinite_or_invalid_joint_command")
    q_commanded = max(lower, min(upper, q_proposed))
    if q_commanded != q_proposed:
        previous = (warnings or {}).get(joint, {})
        _warn(warnings, joint, {
            "reason": "joint_position_clipped",
            "q_proposed": float(q_proposed),
            "q_commanded": float(q_commanded),
            "bounds_rad": [float(lower), float(upper)],
            "count": previous.get("count", 0) + 1,
            "max_correction_rad": max(previous.get("max_correction_rad", 0),
                                      abs(float(q_proposed - q_commanded))),
        })
    return q_commanded


def clip_velocity(joint, dq_proposed, limit, warnings=None):
    """Cap desired joint speed at the model limit and record a warning."""
    if not math.isfinite(dq_proposed):
        raise ValueError("nonfinite_or_invalid_joint_command")
    dq_commanded = max(-limit, min(limit, dq_proposed))
    if dq_commanded != dq_proposed:
        previous = (warnings or {}).get(joint, {})
        _warn(warnings, joint, {
            "velocity_reason": "joint_velocity_clipped",
            "dq_proposed": float(dq_proposed),
            "dq_commanded": float(dq_commanded),
            "velocity_limit_rad_secs": float(limit),
            "velocity_count": previous.get("velocity_count", 0) + 1,
            "max_velocity_correction_rad_secs": max(
                previous.get("max_velocity_correction_rad_secs", 0),
                abs(float(dq_proposed - dq_commanded)),
            ),
        })
    return dq_commanded


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


def note_limit(warnings, joint, reason, value, limit, units, **extra):
    """Record the worst exceedance of a limit and let the caller continue."""
    if not math.isfinite(value) or not math.isfinite(limit):
        raise ValueError("nonfinite_or_invalid_joint_command")
    if warnings is None or value <= limit:
        return False
    entry = warnings.setdefault(joint, {})
    noted = entry.setdefault("limits", [])
    kind = extra.get("torque_kind")
    previous = next((item for item in noted
                     if item["reason"] == reason and item.get("torque_kind") == kind), None)
    if previous is None:
        noted.append({
            "reason": reason,
            "value": float(value),
            "limit": float(limit),
            "units": units,
            "count": 1,
            **extra,
        })
        return True
    previous["count"] += 1
    if value >= previous["value"]:
        previous["value"] = float(value)
        previous.update(extra)
    return True


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
    initial, initial_velocity, warnings=None, *, enforce_torque=True,
):
    """Validate a proposal against current measurements before actuation."""
    if not all(math.isfinite(v) for v in (q_proposed, dq_proposed, tau, q_measured)):
        raise ValueError("nonfinite_or_invalid_joint_command")
    q_commanded = clip_position(joint, q_proposed, limits.position_min_rad,
                               limits.position_max_rad, warnings)
    dq_commanded = clip_velocity(joint, dq_proposed, limits.max_velocity_rad_secs, warnings)
    if enforce_torque:
        check_bound("joint_feedforward_torque_limit", joint, abs(tau), limits.max_torque_nm, "Nm")
    else:
        note_limit(warnings, joint, "joint_feedforward_torque_limit", abs(tau),
                   limits.max_torque_nm, "Nm", torque_kind="feedforward")
    if initial:
        check_bound("initial_joint_target_velocity_limit", joint, abs(dq_commanded),
                    initial_velocity, "rad/s")
    return q_commanded, dq_commanded


def check_torque(joint, limits, torque, *, kind=None):
    """Check estimated demand or measured effort against the same bound."""
    try:
        check_bound("joint_torque_limit", joint, abs(torque), limits.max_torque_nm, "Nm")
    except JointViolation as exc:
        if kind:
            exc.details["torque_kind"] = kind
        raise
