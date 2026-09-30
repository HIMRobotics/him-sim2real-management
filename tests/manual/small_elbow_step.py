"""Hold the starting pose, then request a small, smooth left-elbow movement."""

import math
from dataclasses import dataclass

from robot_safety.config import COMMAND_FIELDS
from robot_safety.hardware_constants import LEFT_ELBOW_INDEX, NUM_JOINTS, UPPER_BODY_INDICES



@dataclass(frozen=True)
class SmallElbowSettings:
    """Physical test timing and displacement, independent of robot joint limits."""

    angle_rad: float = -0.05
    move_duration_secs: float = 10.0
    duration_secs: float = 12.0
    command_period_secs: float = 0.02
    max_publish_gap_secs: float = 0.1
    final_position_tolerance_rad: float = 0.02


class SmallElbowStep:
    """Keep every other target fixed and extend the left elbow by 0.05 rad."""

    def __init__(self, measured_positions, settings=None):
        if len(measured_positions) != NUM_JOINTS or not all(
            math.isfinite(value) for value in measured_positions
        ):
            raise ValueError("expected 29 finite measured joint positions")
        self.start_positions = tuple(measured_positions)
        self.settings = settings or SmallElbowSettings()

    def command_at(self, elapsed_secs):
        """Use a quintic ramp with zero endpoint velocity and acceleration."""
        if not math.isfinite(elapsed_secs) or elapsed_secs < 0:
            raise ValueError("elapsed time must be finite and nonnegative")
        settings = self.settings
        progress = min(elapsed_secs / settings.move_duration_secs, 1.0)
        fraction = 10 * progress**3 - 15 * progress**4 + 6 * progress**5
        command = {field: [0.0] * NUM_JOINTS for field in COMMAND_FIELDS}
        command["q"] = list(self.start_positions)
        command["q"][LEFT_ELBOW_INDEX] += settings.angle_rad * fraction
        command["dq"][LEFT_ELBOW_INDEX] = (
            settings.angle_rad
            * 30
            * progress**2
            * (1 - progress) ** 2
            / settings.move_duration_secs
        )
        for index in UPPER_BODY_INDICES:
            command["kp"][index] = 60.0
            command["kd"][index] = 1.5
        return command

    def followed_target(self, measured_positions):
        """Require the final elbow measurement to be close to the requested target."""
        target = self.start_positions[LEFT_ELBOW_INDEX] + self.settings.angle_rad
        return (
            abs(measured_positions[LEFT_ELBOW_INDEX] - target)
            <= self.settings.final_position_tolerance_rad
        )
