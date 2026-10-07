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
    quat: tuple = ()
    gyro: tuple = ()


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
        tuple(float(v) for v in message.imu_state.quaternion)
        if hasattr(message, "imu_state") else (),
        tuple(float(v) for v in message.imu_state.gyroscope)
        if hasattr(message, "imu_state") else (),
    )


def policy_state(sample, observation_id, boot_id):
    """Expose measured joints and IMU in the container's existing layout."""
    quat, gyro = sample.quat, sample.gyro
    if (len(quat) != 4 or len(gyro) != 3
            or not all(math.isfinite(v) for v in quat + gyro)
            or abs(sum(v * v for v in quat) - 1) > 0.01):
        raise ValueError("invalid_policy_imu")
    return {
        "q": sample.q, "dq": sample.dq, "quat": quat, "gyro": gyro,
        "upright": 1 - 2 * (quat[1] ** 2 + quat[2] ** 2),
        "observation_id": observation_id, "session_id": "",
        "received_at_secs": sample.received_at_secs, "host_boot_id": boot_id,
    }


def base_state_from_sdk(message, received_at_secs, velocity_frame, quat):
    """Convert qualified SDK base measurements to world-frame position/velocity."""
    stamp = message.stamp
    source_time = int(stamp.sec) + int(stamp.nanosec) * 1e-9
    position = tuple(float(v) for v in message.position)
    velocity = tuple(float(v) for v in message.velocity)
    if (len(position) != 3 or len(velocity) != 3 or source_time <= 0
            or not all(math.isfinite(v) for v in position + velocity + (source_time,))):
        raise ValueError("invalid_base_state")
    if velocity_frame == "body":
        if (len(quat) != 4 or not all(math.isfinite(v) for v in quat)
                or abs(sum(v * v for v in quat) - 1) > 0.01):
            raise ValueError("invalid_base_orientation")
        w, x, y, z = quat
        vx, vy, vz = velocity
        tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
        velocity = (vx + w * tx + y * tz - z * ty,
                    vy + w * ty + z * tx - x * tz,
                    vz + w * tz + x * ty - y * tx)
    elif velocity_frame != "world":
        raise ValueError("base velocity frame must be explicitly world or body")
    return {"position": position, "linear_velocity": velocity,
            "base_source_time_secs": source_time, "base_received_at_secs": received_at_secs}


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
