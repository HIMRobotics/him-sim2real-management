"""Convert G1 telemetry to readable state: joint positions, velocities,
physical Start button state and local receipt time.
"""

import math
import queue
from dataclasses import dataclass
from threading import Event

from .controller import decode_start_pressed
from .hardware_constants import G1_29_JOINT_NAMES

SDK_DISCOVERY_TIMEOUT_SECS = 5.0


@dataclass(frozen=True)
class FeedbackSample:
    """Copied SDK state with local callback receipt time in seconds."""

    received_at_secs: float
    q: tuple
    dq: tuple
    start_button_pressed: bool
    temperature: tuple = ()
    tau_est: tuple = ()


def feedback_from_sdk(message, received_at_secs):
    """Extract 29 telemetry slots without inferring installed hardware."""
    if len(message.motor_state) < len(G1_29_JOINT_NAMES):
        raise ValueError("missing G1 motor state slots")
    motors = message.motor_state[: len(G1_29_JOINT_NAMES)]
    q = tuple(float(motor.q) for motor in motors)
    dq = tuple(float(motor.dq) for motor in motors)
    if not all(math.isfinite(value) for value in (received_at_secs,) + q + dq):
        raise ValueError("nonfinite feedback")
    return FeedbackSample(
        received_at_secs, q, dq, decode_start_pressed(message.wireless_remote),
        tuple(tuple(float(t) for t in motor.temperature) for motor in motors)
        if all(hasattr(motor, "temperature") for motor in motors) else (),
        tuple(float(motor.tau_est) for motor in motors)
        if all(hasattr(motor, "tau_est") for motor in motors) else (),
    )


class FeedbackInbox:
    """Preserve button edges in order and latch any dropped or invalid input."""

    def __init__(self, capacity):
        if capacity <= 0:
            raise ValueError("feedback capacity must be positive")
        self.samples = queue.Queue(maxsize=capacity)
        self.input_fault = Event()

    def receive(self, message, received_at_secs):
        if self.input_fault.is_set():
            return
        try:
            self.samples.put_nowait(feedback_from_sdk(message, received_at_secs))
        except (AttributeError, TypeError, ValueError, OverflowError, queue.Full):
            self.input_fault.set()

    def take(self):
        """Return the oldest sample or None without blocking the executor."""
        try:
            return self.samples.get_nowait()
        except queue.Empty:
            return None
