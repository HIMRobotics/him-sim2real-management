"""Generate Unitree's arm7 example targets without creating an SDK publisher."""

import math
from dataclasses import dataclass

from robot_safety.config import COMMAND_FIELDS
from robot_safety.hardware_constants import (
    LEFT_ELBOW_INDEX,
    LEFT_SHOULDER_ROLL_INDEX,
    NUM_JOINTS,
    RIGHT_ELBOW_INDEX,
    RIGHT_SHOULDER_ROLL_INDEX,
    UPPER_BODY_INDICES,
)


@dataclass(frozen=True)
class UnitreeArmSettings:
    """Original arm-example gains and timing, with a separate missed-update limit."""

    command_period_secs: float = 0.02
    max_publish_gap_secs: float = 0.1
    zero_pose_end_secs: float = 3.0
    raised_pose_end_secs: float = 9.0
    duration_secs: float = 18.0
    stiffness_nm_per_rad: float = 60.0
    damping_nm_sec_per_rad: float = 1.5


class UnitreeArmSequence:
    """Zero upper body, raise both arms, then return toward zero using live positions."""

    def __init__(self, settings=None):
        self.settings = UnitreeArmSettings() if settings is None else settings

    def command_at(self, elapsed_secs, measured_positions):
        """Match the SDK example's feedback-based formulas and zero velocity targets."""
        if (
            not math.isfinite(elapsed_secs)
            or elapsed_secs < 0
            or len(measured_positions) != NUM_JOINTS
            or not all(math.isfinite(value) for value in measured_positions)
        ):
            raise ValueError(
                "expected nonnegative time and 29 finite measured positions"
            )
        settings = self.settings
        targets = [0.0] * NUM_JOINTS
        targets[LEFT_SHOULDER_ROLL_INDEX] = math.pi / 2  # Left shoulder roll.
        targets[RIGHT_SHOULDER_ROLL_INDEX] = -math.pi / 2  # Right shoulder roll.
        targets[LEFT_ELBOW_INDEX] = targets[RIGHT_ELBOW_INDEX] = math.pi / 2  # Both elbows.
        if elapsed_secs < settings.zero_pose_end_secs:
            fraction = elapsed_secs / settings.zero_pose_end_secs
            targets = [0.0] * NUM_JOINTS
        elif elapsed_secs < settings.raised_pose_end_secs:
            fraction = (elapsed_secs - settings.zero_pose_end_secs) / (
                settings.raised_pose_end_secs - settings.zero_pose_end_secs
            )
        else:
            fraction = min(
                (elapsed_secs - settings.raised_pose_end_secs)
                / (settings.duration_secs - settings.raised_pose_end_secs),
                1.0,
            )
            targets = [0.0] * NUM_JOINTS
        command = {field: [0.0] * NUM_JOINTS for field in COMMAND_FIELDS}
        command["q"] = list(measured_positions)
        for index in UPPER_BODY_INDICES:
            command["q"][index] = (1 - fraction) * measured_positions[
                index
            ] + fraction * targets[index]
            command["kp"][index] = settings.stiffness_nm_per_rad
            command["kd"][index] = settings.damping_nm_sec_per_rad
        return command
