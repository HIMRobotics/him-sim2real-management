"""Independent postcheck watchdog and fake output selection, without ROS I/O."""

import math
from copy import deepcopy

from .config import COMMAND_FIELDS, EXPIRY_ROUNDING_TOLERANCE_SECS, Config
from .core import SupervisorPhase, vector


class MotorCommandGate:
    """Accept checked frames for one explicitly armed session on this host.

    Abort stays latched for this object's lifetime. Reset coordination is
    intentionally deferred. All timestamps are local monotonic seconds.
    """

    def __init__(self, host_boot_id, config=None):
        self.config = Config() if config is None else config
        self.host_boot_id = host_boot_id
        self.supervisor_phase = SupervisorPhase.INHIBITED
        self.abort_reason = None
        self.policy_session_id = None
        self.last_command_sequence = -1
        self.last_checked_command = None
        self.time_command_expires_secs = None
        # Starts on arm, then refreshes only for a newly accepted command.
        self.time_command_watchdog_refreshed_secs = None
        self.time_last_cycle_secs = None

    def abort(self, reason):
        """Blow the fuse and discard previously accepted motion."""
        if self.abort_reason is None:
            self.abort_reason = reason
        self.supervisor_phase = SupervisorPhase.STOP_LATCHED
        self.last_checked_command = None

    def arm(self, session_id, cur_time_secs):
        """Authorize one checker session without automatically resuming motion."""
        if self.supervisor_phase != SupervisorPhase.INHIBITED or not session_id:
            return False
        self.policy_session_id = session_id
        self.time_command_watchdog_refreshed_secs = cur_time_secs
        self.supervisor_phase = SupervisorPhase.ARMED
        return True

    def check_deadlines(self, cur_time_secs):
        """Watch postcheck silence even when the checker process is frozen."""
        if self.supervisor_phase not in (
            SupervisorPhase.ARMED,
            SupervisorPhase.RUNNING,
        ):
            return
        if (
            self.time_last_cycle_secs is not None
            and cur_time_secs - self.time_last_cycle_secs >= self.config.loop_timeout
        ):
            self.abort("motor_gate_loop_timeout")
        elif (
            cur_time_secs - self.time_command_watchdog_refreshed_secs
            >= self.config.action_timeout
        ):
            self.abort("postcheck_timeout")
        elif (
            self.time_command_expires_secs is not None
            and cur_time_secs >= self.time_command_expires_secs
        ):
            self.abort("checked_command_expired")

    def receive(
        self, command, host_boot_id, checked_at_secs, expires_at_secs, cur_time_secs
    ):
        """Accept a fresh postcheck frame without extending its expiry."""
        self.check_deadlines(cur_time_secs)
        if self.supervisor_phase not in (
            SupervisorPhase.ARMED,
            SupervisorPhase.RUNNING,
        ):
            return False
        valid = (
            host_boot_id == self.host_boot_id
            and all(
                type(value) in (int, float) and math.isfinite(value)
                for value in (checked_at_secs, expires_at_secs)
            )
            and checked_at_secs <= cur_time_secs < expires_at_secs
            and expires_at_secs - checked_at_secs
            <= self.config.action_timeout + EXPIRY_ROUNDING_TOLERANCE_SECS
            and isinstance(command, dict)
            and command.get("session") == self.policy_session_id
            and type(command.get("seq")) is int
            and command["seq"] > self.last_command_sequence
        )
        # Defense at the final boundary, even though the checker owns validation.
        for name, bounds in self.config.command_limits.items():
            valid = valid and vector(command.get(name), self.config.joints, *bounds)
        if not valid:
            self.abort("invalid_postcheck")
            return False
        self.last_command_sequence = command["seq"]
        self.last_checked_command = deepcopy(command)
        self.time_command_expires_secs = expires_at_secs
        self.time_command_watchdog_refreshed_secs = cur_time_secs
        self.supervisor_phase = SupervisorPhase.RUNNING
        return True

    def cycle(self, cur_time_secs):
        """Return fake policy or damping output at the independent gate cadence."""
        self.check_deadlines(cur_time_secs)
        self.time_last_cycle_secs = cur_time_secs
        if self.supervisor_phase == SupervisorPhase.RUNNING:
            return dict(deepcopy(self.last_checked_command), kind="policy")
        output = {name: [0.0] * self.config.joints for name in COMMAND_FIELDS}
        output["kd"] = [self.config.damping] * self.config.joints
        output["kind"] = "damping"
        return output
