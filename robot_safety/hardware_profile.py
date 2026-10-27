"""Load explicitly supplied G1 gains and operating limits."""

import json
import math
from dataclasses import dataclass
from pathlib import Path

from .hardware_constants import (
    DAMPING_FSM_ID,
    G1_29_JOINT_NAMES,
    UPPER_BODY_INDICES,
)
from .joint_checks import joint_profile_entries

HARDWARE_PERIOD_SECS = 0.02  # Policy inference and safety check for every whole-body controller.
BODY_REPEAT_SECS = 0.002  # Shared rt/lowcmd refresh. Networks stay on the 20 ms tick.
STATE_POLL_SECS = 0.005  # Newest rt/lowstate sample. The robot publishes at 1 kHz.
# Policy state goes to the container once per network tick, from the newest
# sample. Half a poll of margin keeps one state per tick despite poll jitter.
POLICY_STATE_SECS = HARDWARE_PERIOD_SECS - STATE_POLL_SECS / 2
HARDWARE_WATCHDOG_POLL_SECS = 0.005
HARDWARE_FEEDBACK_TIMEOUT_SECS = 0.2
HARDWARE_BATTERY_TIMEOUT_SECS = 5.0
READINESS_RPC_TIMEOUT_SECS = 2.0
READINESS_QUERY_ATTEMPTS = 2
READINESS_CHECK_TIMEOUT_SECS = 5.0
DAMPING_RPC_TIMEOUT_SECS = 0.1
DAMPING_RETRY_SECS = 0.1
SHUTDOWN_WAIT_SECS = 1.0
JOINT_DIAGNOSTICS_PERIOD_SECS = 1.0


@dataclass(frozen=True)
class HardwareProfile:
    """Required physical settings, with no inferred or default hardware gains."""

    expected_machine_type: int
    allowed_start_fsm_ids: tuple
    handover_secs: float
    max_publish_gap_secs: float
    initial_velocity_tolerance_rad_secs: float
    joints: tuple
    legs: tuple

    @classmethod
    def load(cls, path):
        """Require all 29 entries using the SDK joint names."""
        data = json.loads(Path(path).read_text())
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
        limits = [entry[2] for entry in joint_profile_entries(data, G1_29_JOINT_NAMES)]
        data["legs"] = tuple(limits[:12])
        data["joints"] = tuple(limits[index] for index in UPPER_BODY_INDICES)
        data["allowed_start_fsm_ids"] = tuple(modes)
        return cls(**data)
