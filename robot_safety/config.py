"""Dry-run settings and sanity bounds, not validated G1 hardware limits."""
from dataclasses import dataclass, field
import math


MAX_CONFIGURED_JOINTS = 35  # Existing software cap, not a robot capability claim.
REMOTE_BUTTON_BITS = 16
COMMAND_FIELDS = ('q', 'dq', 'kp', 'kd', 'tau')


def validate_bounds(bounds, nonnegative=False):
    """Require an immutable pair of finite, ordered numeric bounds."""
    if (not isinstance(bounds, tuple) or len(bounds) != 2 or
            not all(type(value) in (int, float) and math.isfinite(value) for value in bounds) or
            bounds[0] > bounds[1] or (nonnegative and bounds[0] < 0)):
        raise ValueError('bounds must be finite and ordered, with nonnegative gains')


@dataclass(frozen=True)
class JointCommandLimits:
    """Inclusive per-field sanity bounds applied uniformly to all test joints."""

    position_rad: tuple = (-10., 10.)  # q: desired joint angle.
    velocity_rad_per_sec: tuple = (-50., 50.)  # dq: desired angular velocity.
    stiffness_nm_per_rad: tuple = (0., 500.)  # kp: position-error gain.
    damping_nm_sec_per_rad: tuple = (0., 50.)  # kd: velocity-error gain.
    feedforward_torque_nm: tuple = (-200., 200.)  # tau: torque added to PD control.

    def __post_init__(self):
        for field_name, bounds in self.items():
            validate_bounds(bounds, nonnegative=field_name in ('kp', 'kd'))

    def items(self):
        """Map command field names to their inclusive bounds."""
        return zip(COMMAND_FIELDS, (
            self.position_rad, self.velocity_rad_per_sec,
            self.stiffness_nm_per_rad, self.damping_nm_sec_per_rad,
            self.feedforward_torque_nm,
        ))


@dataclass(frozen=True)
class Config:
    """Supervisor settings with time values in seconds and test-only gains."""

    joints: int = 29
    period: float = .005
    action_timeout: float = .2
    observation_timeout: float = .2
    robot_timeout: float = .2
    loop_timeout: float = .2
    damping: float = 1.0  # Placeholder kd for dry-run tests, not validated on the G1.
    stop_bit: int = 3  # Select in SDK wireless_remote bytes 2:4.

    observation_history_size: int = 512
    command_limits: JointCommandLimits = field(default_factory=JointCommandLimits)
    robot_position_bounds_rad: tuple = (-1e6, 1e6)
    robot_velocity_bounds_rad_per_sec: tuple = (-1e6, 1e6)

    def __post_init__(self):
        if not 1 <= self.joints <= MAX_CONFIGURED_JOINTS or not 0 <= self.stop_bit < REMOTE_BUTTON_BITS:
            raise ValueError('invalid joint count or stop bit')
        for key in ('period', 'action_timeout', 'observation_timeout', 'robot_timeout', 'loop_timeout', 'damping'):
            value = getattr(self, key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError('configuration values must be finite and positive')

        if type(self.observation_history_size) is not int or self.observation_history_size < 1:
            raise ValueError('observation history size must be a positive integer')
        if not isinstance(self.command_limits, JointCommandLimits):
            raise ValueError('command_limits must be JointCommandLimits')
        validate_bounds(self.robot_position_bounds_rad)
        validate_bounds(self.robot_velocity_bounds_rad_per_sec)

