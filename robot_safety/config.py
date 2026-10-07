"""Shared gateway settings and sanity bounds, separate from physical limits."""

import math
from dataclasses import dataclass, field

from .hardware_constants import NUM_JOINTS

MAX_CONFIGURED_JOINTS = 35  # Existing software cap, not a robot capability claim.
EXPIRY_ROUNDING_TOLERANCE_SECS = 1e-9  # Allow floating-point subtraction roundoff.
COMMAND_FIELDS = ("q", "dq", "kp", "kd", "tau")


def validate_bounds(bounds, nonnegative=False):
    """Require an immutable pair of finite, ordered numeric bounds."""
    if (
        not isinstance(bounds, tuple)
        or len(bounds) != 2
        or not all(
            type(value) in (int, float) and math.isfinite(value) for value in bounds
        )
        or bounds[0] > bounds[1]
        or (nonnegative and bounds[0] < 0)
    ):
        raise ValueError("bounds must be finite and ordered, with nonnegative gains")


@dataclass(frozen=True)
class JointCommandLimits:
    """Inclusive per-field sanity bounds applied uniformly to all test joints."""

    position_rad: tuple = (-10.0, 10.0)  # q: desired joint angle.
    velocity_rad_per_sec: tuple = (-50.0, 50.0)  # dq: desired angular velocity.
    stiffness_nm_per_rad: tuple = (0.0, 500.0)  # kp: position-error gain.
    damping_nm_sec_per_rad: tuple = (0.0, 50.0)  # kd: velocity-error gain.
    feedforward_torque_nm: tuple = (-200.0, 200.0)  # tau: torque added to PD control.

    def __post_init__(self):
        for field_name, bounds in self.items():
            validate_bounds(bounds, nonnegative=field_name in ("kp", "kd"))

    def items(self):
        """Map command field names to their inclusive bounds."""
        return zip(
            COMMAND_FIELDS,
            (
                self.position_rad,
                self.velocity_rad_per_sec,
                self.stiffness_nm_per_rad,
                self.damping_nm_sec_per_rad,
                self.feedforward_torque_nm,
            ),
        )


@dataclass(frozen=True)
class ThrashConfig:
    """Provisional oscillation settings, calibrated independently of policy commands."""

    mode: str = "monitor"
    slow_cutoff_hz: float = 4.0
    fast_cutoff_hz: float = 25.0
    averaging_secs: float = 0.15
    persistence_secs: float = 0.2
    history_secs: float = 2.0
    min_cycles: int = 3
    deadband_rad_secs: float = 0.1
    min_excursion_rad: float = 0.02
    activity_rms: float = 1.0
    severe_rms: float = 2.5
    irregularity_cv: float = 0.35
    local_joints: int = 2
    distributed_joints: int = 3
    velocity_scales: tuple = ()
    regions: tuple = ()

    def __post_init__(self):
        if self.mode not in ("off", "monitor", "enforce"):
            raise ValueError("thrash mode must be off, monitor or enforce")
        for name, value in self.__dict__.items():
            if name in ("mode", "velocity_scales", "regions"):
                continue
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"invalid thrash setting: {name}")
        if self.fast_cutoff_hz <= self.slow_cutoff_hz or self.severe_rms <= self.activity_rms:
            raise ValueError("thrash cutoffs and severity thresholds must be ordered")
        for name in ("min_cycles", "local_joints", "distributed_joints"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"thrash {name} must be an integer")
        if not 2 <= self.min_cycles <= 6:
            raise ValueError("thrash min_cycles must be between 2 and 6")
        if not isinstance(self.velocity_scales, tuple) or any(
            type(v) not in (int, float) or not math.isfinite(v) or v <= 0
            for v in self.velocity_scales
        ):
            raise ValueError("thrash velocity scales must be finite and positive")
        if not isinstance(self.regions, tuple) or any(
            type(v) is not int or v < 0 for v in self.regions
        ):
            raise ValueError("thrash regions must be nonnegative integers")


@dataclass(frozen=True)
class Config:
    """Supervisor settings with time values in seconds and test-only gains."""

    joints: int = NUM_JOINTS
    period: float = 0.005
    action_timeout: float = 0.2
    observation_timeout: float = 0.2
    robot_timeout: float = 0.2
    loop_timeout: float = 0.2
    damping: float = 1.0  # Placeholder kd for dry-run tests, not validated on the G1.
    double_press_window_secs: float = 0.5  # Maximum interval between Start press edges.

    observation_history_size: int = 512
    runtime_queue_capacity: int = 64  # Maximum observations or controls per cycle.
    command_limits: JointCommandLimits = field(default_factory=JointCommandLimits)
    thrash: ThrashConfig = field(default_factory=ThrashConfig)
    robot_position_bounds_rad: tuple = (-1e6, 1e6)
    robot_velocity_bounds_rad_per_sec: tuple = (-1e6, 1e6)

    def __post_init__(self):
        if not 1 <= self.joints <= MAX_CONFIGURED_JOINTS:
            raise ValueError("invalid joint count")
        for key in (
            "period",
            "action_timeout",
            "observation_timeout",
            "robot_timeout",
            "loop_timeout",
            "damping",
            "double_press_window_secs",
        ):
            value = getattr(self, key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError("configuration values must be finite and positive")

        if (
            type(self.observation_history_size) is not int
            or self.observation_history_size < 1
        ):
            raise ValueError("observation history size must be a positive integer")
        if (
            type(self.runtime_queue_capacity) is not int
            or self.runtime_queue_capacity < 1
        ):
            raise ValueError("runtime queue capacity must be a positive integer")
        if not isinstance(self.command_limits, JointCommandLimits):
            raise TypeError("command_limits must be JointCommandLimits")
        if not isinstance(self.thrash, ThrashConfig):
            raise TypeError("thrash must be ThrashConfig")
        for values in (self.thrash.velocity_scales, self.thrash.regions):
            if values and len(values) != self.joints:
                raise ValueError("thrash scales and regions must match the joint count")
        validate_bounds(self.robot_position_bounds_rad)
        validate_bounds(self.robot_velocity_bounds_rad_per_sec)
