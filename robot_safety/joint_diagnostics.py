"""Record proposal gaps and responses without changing safety decisions."""

import math
from dataclasses import dataclass

from .hardware_constants import G1_29_JOINT_NAMES, NUM_JOINTS


@dataclass
class ObservedRange:
    """Signed extrema from finite samples."""

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
            "min": self.minimum, "max": self.maximum,
            "max_abs": max(abs(self.minimum), abs(self.maximum)) if self.samples else None,
            "samples": self.samples,
        }


class JointDeltaDiagnostics:
    """Separate pending proposals from responses to successful writes."""

    def __init__(self):
        self.joints = {
            i: {name: ObservedRange() for name in (
                "command_gap_rad", "response_error_rad", "measured_velocity_rad_secs"
            )} for i in range(NUM_JOINTS)
        }

    def record(self, command, feedback):
        """Sample bounded commands before remaining checks, including rejections."""
        for index, ranges in self.joints.items():
            q_proposed, q_measured = command["q"][index], feedback.q[index]
            ranges["command_gap_rad"].record(q_proposed - q_measured)
            ranges["measured_velocity_rad_secs"].record(feedback.dq[index])

    def record_response(self, q_applied, feedback):
        """Use feedback received after a successful full-ownership write."""
        for index, position in q_applied.items():
            self.joints[index]["response_error_rad"].record(feedback.q[index] - position)

    def snapshot(self):
        return {G1_29_JOINT_NAMES[i]: {name: observed.snapshot()
                for name, observed in ranges.items()} for i, ranges in self.joints.items()}
