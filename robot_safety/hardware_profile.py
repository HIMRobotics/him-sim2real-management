"""Load explicitly supplied G1 upper-body gains and operating limits."""

import json
import math
from dataclasses import dataclass
from pathlib import Path

from .feedback import G1_29_JOINT_NAMES

UPPER_BODY_INDICES = tuple(range(12, 29))
ARM_WEIGHT_INDEX = 29
DAMPING_FSM_ID = 1
WALK_FSM_ID = 500  # LocoClient.Start. Balances, and may step.
WALK_REFRESH_SECS = 0.5
ZERO_VELOCITY_DURATION_SECS = 1.0
HARDWARE_PERIOD_SECS = 0.02  # Matches the tested Unitree arm7 example.
HARDWARE_WATCHDOG_POLL_SECS = 0.005
HARDWARE_FEEDBACK_TIMEOUT_SECS = 0.2
READINESS_RPC_TIMEOUT_SECS = 2.0
READINESS_QUERY_ATTEMPTS = 2
SDK_RPC_TIMEOUT_CODE = 3104  # Unitree RPC_ERR_CLIENT_API_TIMEOUT.
SDK_RPC_SEND_CODE = 3102  # Unitree RPC_ERR_CLIENT_SEND.
READINESS_CHECK_TIMEOUT_SECS = 5.0
DAMPING_RPC_TIMEOUT_SECS = 0.1
DAMPING_RETRY_SECS = 0.1
SHUTDOWN_WAIT_SECS = 1.0
JOINT_DIAGNOSTICS_PERIOD_SECS = 1.0


@dataclass(frozen=True)
class JointHardwareLimits:
    """One joint's configured bounds and PD gains, in radians, seconds and Nm."""

    position_min_rad: float
    position_max_rad: float
    kp: float
    kd: float
    max_velocity_rad_secs: float
    max_acceleration_rad_secs2: float
    max_tracking_error_rad: float

    def __post_init__(self):
        if not all(
            type(value) in (int, float) and math.isfinite(value)
            for value in self.__dict__.values()
        ):
            raise ValueError("joint limits must be finite numbers")
        if self.position_min_rad >= self.position_max_rad:
            raise ValueError("joint position bounds must be ordered")
        for name in (
            "kp",
            "kd",
            "max_velocity_rad_secs",
            "max_acceleration_rad_secs2",
            "max_tracking_error_rad",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True)
class HardwareProfile:
    """Required physical settings, with no inferred or default hardware gains."""

    expected_machine_type: int
    allowed_start_fsm_ids: tuple
    handover_secs: float
    max_publish_gap_secs: float
    initial_position_tolerance_rad: float
    initial_velocity_tolerance_rad_secs: float
    joints: tuple

    @classmethod
    def load(cls, path):
        """Require all 17 arm/waist entries using the SDK joint names."""
        data = json.loads(Path(path).read_text())
        if data["expected_machine_type"] != 5:
            raise ValueError(
                "this adapter is restricted to the identified machine type 5"
            )
        modes = data["allowed_start_fsm_ids"]
        if not modes or any(
            type(mode) is not int or mode <= DAMPING_FSM_ID for mode in modes
        ):
            raise ValueError(
                "supply verified standing FSM IDs, excluding passive modes"
            )
        for name in (
            "handover_secs",
            "max_publish_gap_secs",
            "initial_position_tolerance_rad",
            "initial_velocity_tolerance_rad_secs",
        ):
            value = data[name]
            if (
                type(value) not in (int, float)
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if data["max_publish_gap_secs"] > HARDWARE_FEEDBACK_TIMEOUT_SECS:
            raise ValueError("publish gap must not exceed the feedback deadline")
        names = [G1_29_JOINT_NAMES[index] for index in UPPER_BODY_INDICES]
        if set(data["joints"]) != set(names):
            raise ValueError("profile must specify exactly the 17 arm and waist joints")
        data["joints"] = tuple(
            JointHardwareLimits(**data["joints"][name]) for name in names
        )
        data["allowed_start_fsm_ids"] = tuple(modes)
        return cls(**data)
