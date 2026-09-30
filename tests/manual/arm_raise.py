"""Generate one slow left shoulder raise for diagnostic commands only."""

import math
from dataclasses import dataclass

from robot_safety.config import COMMAND_FIELDS
from robot_safety.hardware_constants import LEFT_SHOULDER_ROLL_INDEX, NUM_JOINTS

MAX_TEST_EXCURSION_RAD = 0.15
MIN_TEST_DURATION_SECS = 10.0


@dataclass(frozen=True)
class ArmRaiseSettings:
    """Dry-run excursion, duration and placeholder PD gains, not hardware limits."""

    angle_rad: float = MAX_TEST_EXCURSION_RAD
    duration_secs: float = MIN_TEST_DURATION_SECS
    max_publish_gap_secs: float = 0.02
    stiffness_nm_per_rad: float = 1.0
    damping_nm_sec_per_rad: float = 1.0

    def __post_init__(self):
        for value in (
            self.angle_rad,
            self.duration_secs,
            self.max_publish_gap_secs,
            self.stiffness_nm_per_rad,
            self.damping_nm_sec_per_rad,
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("arm raise settings must be finite and positive")
        # Prevent ROS parameter overrides from turning this slow test into a fast move.
        if self.angle_rad > MAX_TEST_EXCURSION_RAD:
            raise ValueError("test excursion must not exceed 0.15 rad")
        if self.duration_secs < MIN_TEST_DURATION_SECS:
            raise ValueError("test duration must be at least 10 seconds")


class SlowArmRaise:
    """Keep the measured starting pose except for one shoulder-roll target."""

    def __init__(self, measured_positions, settings):
        if len(measured_positions) != NUM_JOINTS or not all(
            math.isfinite(value) for value in measured_positions
        ):
            raise ValueError("expected 29 finite measured positions")
        self.start_positions = tuple(measured_positions)
        self.settings = settings

    def command_at(self, elapsed_secs):
        """Return a full-body test frame with smooth shoulder position/velocity."""
        if not math.isfinite(elapsed_secs) or elapsed_secs < 0:
            raise ValueError("elapsed time must be finite and nonnegative")
        progress = min(elapsed_secs / self.settings.duration_secs, 1.0)
        # Quintic interpolation has zero velocity and acceleration at both ends.
        fraction = 10 * progress**3 - 15 * progress**4 + 6 * progress**5
        speed = 30 * progress**2 * (1 - progress) ** 2
        command = {name: [0.0] * len(self.start_positions) for name in COMMAND_FIELDS}
        command["q"] = list(self.start_positions)
        command["q"][LEFT_SHOULDER_ROLL_INDEX] += self.settings.angle_rad * fraction
        command["dq"][LEFT_SHOULDER_ROLL_INDEX] = (
            self.settings.angle_rad * speed / self.settings.duration_secs
        )
        command["kp"] = [self.settings.stiffness_nm_per_rad] * len(self.start_positions)
        command["kd"] = [self.settings.damping_nm_sec_per_rad] * len(
            self.start_positions
        )
        return command
