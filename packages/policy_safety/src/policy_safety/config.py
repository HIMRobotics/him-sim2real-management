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
    ],
    "push_force_n": 10.0,
    "push_seconds": 0.1,
    "push_start_seconds": 1.0,
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
        "push_force_n",
        "push_seconds",
    ):
        positive(suite[name], name)
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
    if suite["perturbations"] and not suite["push_bodies"]:
        raise ValueError("push bodies cannot be empty")


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
