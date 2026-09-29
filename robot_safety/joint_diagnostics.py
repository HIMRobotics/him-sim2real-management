"""Track joint command and measurement ranges without changing safety decisions."""

import math
from dataclasses import dataclass

from .hardware_constants import G1_29_JOINT_NAMES, NUM_JOINTS


@dataclass
class ObservedRange:
    """Signed extrema from finite samples, with no assumed zero sample."""

    minimum: float = None
    maximum: float = None
    samples: int = 0

    def record(self, value):
        if not math.isfinite(value):
            return
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)
        self.samples += 1

    def snapshot(self):
        return {
            "min": self.minimum,
            "max": self.maximum,
            "max_abs": max(abs(self.minimum), abs(self.maximum))
            if self.samples
            else None,
            "samples": self.samples,
        }


class JointDeltaDiagnostics:
    """Accumulate ranges at output checks, including targets later rejected."""

    def __init__(self):
        self.joints = {
            index: {
                name: ObservedRange()
                for name in (
                    "tracking_error_rad",
                    "target_step_rad",
                    "measured_velocity_rad_secs",
                )
            }
            for index in range(NUM_JOINTS)
        }

    def record(self, command, feedback, previous_written_targets):
        """Record all joints before validation can reject one of them."""
        for index, ranges in self.joints.items():
            ranges["tracking_error_rad"].record(command["q"][index] - feedback.q[index])
            ranges["measured_velocity_rad_secs"].record(feedback.dq[index])
            if previous_written_targets is not None:
                ranges["target_step_rad"].record(
                    command["q"][index] - previous_written_targets[index]
                )

    def snapshot(self):
        return {
            G1_29_JOINT_NAMES[index]: {
                name: observed.snapshot() for name, observed in ranges.items()
            }
            for index, ranges in self.joints.items()
        }
