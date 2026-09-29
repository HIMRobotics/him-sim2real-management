"""Explicit robot contracts and reproducible scenario settings."""

import copy
import json
import math
from pathlib import Path

import numpy as np

DEFAULT_SUITE = {
    "version": 1,
    "seeds": [0],
    "nominal_seconds": 10.0,
    "branch_seconds": 3.0,
    "counterfactual_seconds": 5.0,
    "checkpoints": [0.25, 0.5, 0.75],
    # Each command is [forward speed (m/s), lateral speed (m/s), yaw rate (rad/s)].
    "commands": {
        "stand": [0, 0, 0],
        "forward": [0.3, 0, 0],
        "left": [0, 0.15, 0],
        "right": [0, -0.15, 0],
        "turn_left": [0, 0, 0.3],
        "turn_right": [0, 0, -0.3],
    },
    "perturbations": True,
    "push_bodies": [
        "pelvis",
        "torso_link",
        "left_hip_yaw_link",
        "left_knee_link",
        "left_ankle_roll_link",
        "right_hip_yaw_link",
        "right_knee_link",
        "right_ankle_roll_link",
        "left_shoulder_roll_link",
        "left_elbow_link",
        "left_wrist_yaw_link",
        "right_shoulder_roll_link",
        "right_elbow_link",
        "right_wrist_yaw_link",
    ],
    # The head mesh is fixed to the torso, so this push is applied at the mesh.
    "push_points": [
        {"name": "head", "body": "torso_link", "mesh": "head_link"},
    ],
    # Trunk targets are shoved in every gait. Limb segments are shoved only while walking forward.
    "push_schedule": [
        {
            "names": ["pelvis", "torso_link", "head"],
            "commands": ["stand", "forward", "left", "right", "turn_left", "turn_right"],
            "checkpoints": [0.5],
            "axes": [0, 1],
            "signs": [-1, 1],
        },
        {
            "names": [
                "left_hip_yaw_link",
                "left_knee_link",
                "left_ankle_roll_link",
                "right_hip_yaw_link",
                "right_knee_link",
                "right_ankle_roll_link",
                "left_shoulder_roll_link",
                "left_elbow_link",
                "left_wrist_yaw_link",
                "right_shoulder_roll_link",
                "right_elbow_link",
                "right_wrist_yaw_link",
            ],
            "commands": ["forward"],
            "checkpoints": [0.5],
            "axes": [0, 1],
            "signs": [-1, 1],
        },
    ],
    "push_levels": [
        {"name": "small", "force_n": 10.0},
        {"name": "medium", "force_n": 50.0},
        {"name": "heavy", "force_n": 150.0},
    ],
    "push_seconds": 0.1,
    "push_start_seconds": 1.0,
    # Nominal gaits only. 0.95 is a weak motor. 1.05 is a high gain.
    "torque_scales": [0.95, 1.05],
    "poses": ["standing", "seated", "supine", "prone", "left_side", "right_side"],
}
METRICS = {
    "joint_velocity",
    "joint_acceleration",
    "actuator_torque",
    "tracking_error",
    "target_rate",
    "base_linear_speed",
    "base_angular_speed",
    "contact_force",
    "clipping",
}


def read_json(path):
    return json.loads(Path(path).read_text())


def positive(value, name, zero=False):
    if (
        type(value) not in (int, float)
        or not math.isfinite(value)
        or (value < 0 if zero else value <= 0)
    ):
        raise ValueError(
            f"{name} must be finite and {'nonnegative' if zero else 'positive'}"
        )


def vector(value, size, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def suite_config(path=None):
    """Merge suite overrides and validate timing and scenario selections."""
    suite = copy.deepcopy(DEFAULT_SUITE)
    if path:
        overrides = read_json(path)
        if set(overrides) - set(suite):
            raise ValueError("unknown suite fields")
        suite.update(overrides)
    if suite["version"] != 1 or type(suite["perturbations"]) is not bool:
        raise ValueError("unsupported suite version or perturbations setting")
    _validate_timing(suite)
    _validate_scenarios(suite)
    return suite


def _validate_timing(suite):
    """Require valid durations and keep each push inside its branch."""
    for name in (
        "nominal_seconds",
        "branch_seconds",
        "counterfactual_seconds",
        "push_seconds",
    ):
        positive(suite[name], name)
    _validate_push_levels(suite)
    _validate_torque_scales(suite)
    positive(suite["push_start_seconds"], "push_start_seconds", zero=True)
    if (
        suite["perturbations"]
        and suite["push_start_seconds"] + suite["push_seconds"]
        > suite["branch_seconds"]
    ):
        raise ValueError("push must fit inside branch duration")


def _validate_scenarios(suite):
    """Check seeds, command vectors, checkpoints and starting poses."""
    if not suite["seeds"] or any(type(s) is not int or s < 0 for s in suite["seeds"]):
        raise ValueError("seeds must be nonnegative integers")
    if len(set(suite["seeds"])) != len(suite["seeds"]):
        raise ValueError("duplicate seeds")
    if not suite["commands"]:
        raise ValueError("at least one nominal command is required")
    for command in suite["commands"].values():
        vector(command, 3, "command")
    if any(type(f) not in (float, int) or not 0 < f < 1 for f in suite["checkpoints"]):
        raise ValueError("checkpoint fractions must be between zero and one")
    if len(set(suite["checkpoints"])) != len(suite["checkpoints"]):
        raise ValueError("duplicate checkpoints")
    if set(suite["poses"]) - set(DEFAULT_SUITE["poses"]):
        raise ValueError("unknown counterfactual pose")
    bodies = suite["push_bodies"]
    if suite["perturbations"] and not bodies and not suite["push_points"]:
        raise ValueError("push bodies cannot be empty")
    if any(type(body) is not str or not body for body in bodies):
        raise ValueError("push bodies must be nonempty strings")
    if len(set(bodies)) != len(bodies):
        raise ValueError("duplicate push bodies")
    points = suite["push_points"]
    if type(points) is not list:
        raise ValueError("push points must be a list")
    names = []
    for point in points:
        if set(point) != {"name", "body", "mesh"} or any(
            type(point[key]) is not str or not point[key] for key in point
        ):
            raise ValueError("push point requires name, body, and mesh")
        names.append(point["name"])
    if len(set(names)) != len(names) or set(names) & set(bodies):
        raise ValueError("push point names must be unique and distinct from bodies")
    _validate_push_schedule(suite, set(bodies) | set(names))


def _validate_push_levels(suite):
    """Require a unique positive force for every named shove strength."""
    levels = suite["push_levels"]
    if type(levels) is not list or not levels:
        raise ValueError("push levels must be a nonempty list")
    names = []
    for level in levels:
        if set(level) != {"name", "force_n"}:
            raise ValueError("push level requires name and force_n")
        if type(level["name"]) is not str or not level["name"]:
            raise ValueError("push level name must be a nonempty string")
        positive(level["force_n"], "push force")
        names.append(level["name"])
    if len(set(names)) != len(names):
        raise ValueError("duplicate push levels")


def _validate_torque_scales(suite):
    """Nominal reruns use a positive gain other than the normal motor."""
    scales = suite["torque_scales"]
    if type(scales) is not list:
        raise ValueError("torque scales must be a list")
    for scale in scales:
        positive(scale, "torque scale")
        if scale == 1:
            raise ValueError("torque scale of 1 duplicates the nominal run")
    if len(set(scales)) != len(scales):
        raise ValueError("duplicate torque scales")


def _validate_push_schedule(suite, selected):
    """Require an explicit, non-overlapping shove plan for every selected target."""
    groups = suite["push_schedule"]
    if type(groups) is not list or not groups:
        raise ValueError("push schedule must list at least one group")
    fields = {"names", "commands", "checkpoints", "axes", "signs"}
    seen = []
    for group in groups:
        if set(group) != fields:
            raise ValueError(
                "push schedule group requires names, commands, checkpoints, axes, and signs"
            )
        if any(type(name) is not str or not name for name in group["names"]):
            raise ValueError("push schedule names must be nonempty strings")
        if any(type(command) is not str or not command for command in group["commands"]):
            raise ValueError("push schedule commands must be nonempty strings")
        if any(fraction not in suite["checkpoints"] for fraction in group["checkpoints"]):
            if suite["perturbations"]:
                raise ValueError("push schedule checkpoints must be suite checkpoints")
        if any(axis not in (0, 1) for axis in group["axes"]) or any(
            sign not in (-1, 1) for sign in group["signs"]
        ):
            raise ValueError("push directions must use axes 0 or 1 and signs -1 or 1")
        for key in ("names", "commands", "checkpoints", "axes", "signs"):
            if len(set(group[key])) != len(group[key]) or not group[key]:
                raise ValueError(f"push schedule {key} must be unique and nonempty")
        seen.extend(group["names"])
    if len(set(seen)) != len(seen):
        raise ValueError("push schedule names must be unique")
    if suite["perturbations"] and selected - set(seen):
        raise ValueError("every push target needs a schedule group")


def scheduled_directions(suite, command_name, fraction):
    """Return (target, axis, sign) shoves for one command at one checkpoint."""
    if not suite["perturbations"]:
        return []
    selected = set(suite["push_bodies"]) | {point["name"] for point in suite["push_points"]}
    directions = []
    for group in suite["push_schedule"]:
        if command_name not in group["commands"] or fraction not in group["checkpoints"]:
            continue
        for name in group["names"]:
            if name not in selected:
                continue
            for axis in group["axes"]:
                for sign in group["signs"]:
                    directions.append((name, axis, sign))
    return directions


def profile_config(path):
    profile = read_json(path)
    if profile["version"] != 1 or type(profile["calibrated"]) is not bool:
        raise ValueError("unsupported acceptance profile")
    positive(profile["joint_position_tolerance_rad"], "position tolerance", zero=True)
    positive(profile["fall_height_m"], "fall height")
    if not -1 <= profile["fall_upright_min"] <= 1:
        raise ValueError("fall upright threshold must be in [-1, 1]")
    if set(profile["maxima"]) - METRICS:
        raise ValueError("unknown metric limit")
    for name, limit in profile["maxima"].items():
        positive(limit, name, zero=True)
    if profile["calibrated"] and (
        METRICS - set(profile["maxima"]) or profile.get("uncalibrated_metrics")
    ):
        raise ValueError(
            "calibrated profiles require all metric limits and no uncalibrated metrics"
        )
    return profile
