"""Dry-run supervisor state machine using monotonic timestamps in seconds."""

import math
import uuid
from collections import OrderedDict
from copy import deepcopy
from enum import Enum

from .config import COMMAND_FIELDS, Config
from .controller import StartDoublePressDetector


def vector(value, size, low, high):
    return (
        isinstance(value, list)
        and len(value) == size
        and all(
            type(v) in (int, float) and math.isfinite(v) and low <= v <= high
            for v in value
        )
    )


class SupervisorPhase(str, Enum):
    """Operating modes that determine whether policy output is allowed."""

    INHIBITED = "INHIBITED"  # No policy authority until explicitly armed.
    ARMED = "ARMED"  # Session opened, waiting for its first valid action.
    RUNNING = "RUNNING"  # Valid policy actions may be selected for output.
    STOP_LATCHED = "STOP_LATCHED"  # Policy blocked until reset and rearm.


class Supervisor:
    """Arbitrate dry-run commands through serial calls from one owner.

    Every ``cur_time_secs`` is the local processing time in seconds from
    ``time.monotonic()``, or an injected test clock. Use the same clock
    throughout, with nondecreasing values. Do not pass wall-clock time,
    robot timestamps, or policy-provided timestamps.
    """

    def __init__(self, config=None):
        if config is None:
            config = Config()
        self.start_double_press_detector = StartDoublePressDetector(
            config.double_press_window_secs, config.robot_timeout
        )
        self.config = config  # Immutable joint and watchdog settings.
        self.supervisor_instance_id = (
            uuid.uuid4().hex
        )  # Distinguishes supervisor instances.

        # Operating mode and stop diagnostics.
        self.supervisor_phase = SupervisorPhase.INHIBITED
        self.stop_reason = None  # First fault or stop cause for the current latch.
        self.start_button_pressed = (
            False  # Physical Start state, blocks reset while pressed.
        )
        self.time_stop_latched_secs = (
            None  # Most recent latch time, retained after reset.
        )
        self.time_first_damping_selected_secs = (
            None  # First fake damping cycle for that latch.
        )

        # Policy authority and the latest accepted proposal.
        self.policy_session_id = None  # Token issued on arm and revoked on stop.
        self.last_action_sequence = (
            -1
        )  # Last accepted sequence, rejects replayed actions.
        self.accepted_policy_command = (
            None  # Accepted proposal, not necessarily selected output.
        )
        self.time_action_watchdog_refreshed_secs = (
            None  # Watchdog baseline, set on arm or valid action.
        )

        # Robot feedback and observation provenance.
        self.latest_observation_id = (
            0  # Increasing observation ID, never reused after reset.
        )
        self.latest_robot_observation = (
            None  # Latest measurements exposed to policy clients.
        )
        self.time_last_robot_state_received_secs = (
            None  # Latest valid observation's local receipt time.
        )
        self.observation_received_times_secs = (
            OrderedDict()
        )  # Recent ID-to-time lookup for action freshness.

        # Output history differs from a newly received action or stop request.
        self.time_last_cycle_secs = None  # Last completed arbitration's timestamp.
        self.last_selected_command = (
            None  # Last cycle's selection, retained for status reporting.
        )

    def stop(self, stop_reason, cur_time_secs):
        """Latch a stop at ``cur_time_secs`` with a diagnostic ``stop_reason``."""
        if self.supervisor_phase != SupervisorPhase.STOP_LATCHED:
            (
                self.stop_reason,
                self.time_stop_latched_secs,
                self.time_first_damping_selected_secs,
            ) = stop_reason, cur_time_secs, None
        self.supervisor_phase, self.policy_session_id, self.accepted_policy_command = (
            SupervisorPhase.STOP_LATCHED,
            None,
            None,
        )

    def state(
        self,
        observation,
        start_button_pressed,
        cur_time_secs,
        *,
        stop_requested=False,
        received_time_secs=None,
    ):
        """Process feedback and return whether this sample requests a stop.

        Args:
            observation: Dict with joint positions ``q`` in radians and
                velocities ``dq`` in radians/second, each a list of
                ``config.joints`` values in the configured joint order.
            start_button_pressed: Raw Start state from the controller decoder.
                A double press requests a stop. A held Start blocks reset.
            stop_requested: Independent one-shot stop event, even if Start
                is released. False never clears a latched stop.
            cur_time_secs: Local monotonic processing time in seconds.
            received_time_secs: Local receipt time before queueing. Defaults
                to processing time for callers without an input queue.
        """
        # Check deadlines before a new packet can replace the old timestamp.
        self.watchdogs(cur_time_secs)
        if received_time_secs is None:
            received_time_secs = cur_time_secs
        if (
            not math.isfinite(received_time_secs)
            or not 0 <= cur_time_secs - received_time_secs < self.config.robot_timeout
            or (
                self.time_last_robot_state_received_secs is not None
                and received_time_secs < self.time_last_robot_state_received_secs
            )
        ):
            self.input_fault("stale_robot_input", cur_time_secs)
            return
        joint_count = self.config.joints
        if (
            type(start_button_pressed) is not bool
            or type(stop_requested) is not bool
            or not isinstance(observation, dict)
            or not all(
                vector(observation.get(field_name), joint_count, *bounds)
                for field_name, bounds in (
                    ("q", self.config.robot_position_bounds_rad),
                    ("dq", self.config.robot_velocity_bounds_rad_per_sec),
                )
            )
        ):
            self.start_double_press_detector.reset()
            self.stop("invalid_robot_state", cur_time_secs)
            return
        self.time_last_robot_state_received_secs, self.start_button_pressed = (
            received_time_secs,
            start_button_pressed,
        )
        self.latest_observation_id += 1
        self.observation_received_times_secs[self.latest_observation_id] = (
            received_time_secs
        )
        while (
            len(self.observation_received_times_secs)
            > self.config.observation_history_size
        ):
            self.observation_received_times_secs.popitem(last=False)
        self.latest_robot_observation = deepcopy(
            dict(observation, state_id=self.latest_observation_id)
        )
        double_press_detected = self.start_double_press_detector.update(
            start_button_pressed, received_time_secs
        )
        if stop_requested or double_press_detected:
            self.stop("controller_stop", cur_time_secs)
        return stop_requested or double_press_detected

    def input_fault(self, stop_reason, cur_time_secs):
        """Latch an input fault and require fresh feedback before reset."""
        self.start_double_press_detector.reset()
        self.time_last_robot_state_received_secs = None
        self.stop(stop_reason, cur_time_secs)

    def control(self, command, cur_time_secs):
        """Handle ``stop``, ``reset``, ``arm``, or ``status`` at ``cur_time_secs``.

        Return a status dict on success or an error dict on rejection.
        """
        self.watchdogs(cur_time_secs)
        if command == "stop":
            self.stop("operator_stop", cur_time_secs)
        elif command in ("arm", "reset"):
            if (
                self.time_stop_latched_secs is not None
                and cur_time_secs <= self.time_stop_latched_secs
            ):
                return {"error": "stop takes precedence in this cycle"}
            if (
                self.start_button_pressed
                or self.time_last_robot_state_received_secs is None
                or cur_time_secs - self.time_last_robot_state_received_secs
                >= self.config.robot_timeout
            ):
                return {"error": "fresh state and released stop required"}
            if command == "reset":
                if self.supervisor_phase != SupervisorPhase.STOP_LATCHED:
                    return {"error": "reset requires STOP_LATCHED"}
                self.supervisor_phase, self.stop_reason = (
                    SupervisorPhase.INHIBITED,
                    None,
                )
                self.start_double_press_detector.reset()
                self.policy_session_id, self.accepted_policy_command = None, None
            elif self.supervisor_phase == SupervisorPhase.INHIBITED:
                self.supervisor_phase, self.policy_session_id = (
                    SupervisorPhase.ARMED,
                    uuid.uuid4().hex,
                )
                self.last_action_sequence, self.accepted_policy_command = -1, None
                self.time_action_watchdog_refreshed_secs = cur_time_secs
            else:
                return {"error": "arm requires INHIBITED"}
        elif command != "status":
            return {"error": "unknown command"}
        return self.status(cur_time_secs)

    def accept(self, message, cur_time_secs):
        """Validate a proposed action at ``cur_time_secs`` and return acceptance.

        ``message`` contains the active ``session``, increasing integer
        ``seq``, and supervisor-issued ``state_id`` used for inference.
        Joint lists are ``q`` (rad), ``dq`` (rad/s), ``kp`` (Nm/rad),
        ``kd`` (Nm s/rad), and ``tau`` (Nm), in observation joint order.
        """
        # Only an armed or running session can accept policy commands.
        if self.supervisor_phase not in (
            SupervisorPhase.ARMED,
            SupervisorPhase.RUNNING,
        ):
            return False
        config = self.config
        # Require a message from the currently authorized policy session.
        valid = (
            isinstance(message, dict)
            and message.get("session") == self.policy_session_id
        )
        # Reject duplicate, out-of-order, and non-integer sequence numbers.
        valid = (
            valid
            and type(message.get("seq")) is int
            and message["seq"] > self.last_action_sequence
        )
        # Resolve the source observation using supervisor-owned timestamps.
        source_observation_id = (
            message.get("state_id") if isinstance(message, dict) else None
        )
        observation_received_time_secs = (
            self.observation_received_times_secs.get(source_observation_id)
            if type(source_observation_id) is int
            else None
        )
        # Reject unknown observations and timestamps outside the freshness window.
        valid = (
            valid
            and observation_received_time_secs is not None
            and 0
            <= cur_time_secs - observation_received_time_secs
            < config.observation_timeout
        )
        # Check joint counts, finite numeric values, and configured bounds.
        for field_name, bounds in config.command_limits.items():
            valid = valid and vector(message.get(field_name), config.joints, *bounds)
        # Invalid input latches a stop without refreshing the action watchdog.
        if not valid:
            self.stop("invalid_action", cur_time_secs)
            return False
        # Deadline checks also apply before accepting a late replacement.
        self.watchdogs(cur_time_secs)
        if self.supervisor_phase == SupervisorPhase.STOP_LATCHED:
            return False
        self.last_action_sequence, self.time_action_watchdog_refreshed_secs = (
            message["seq"],
            cur_time_secs,
        )
        self.accepted_policy_command = {k: list(message[k]) for k in COMMAND_FIELDS}
        self.accepted_policy_command["state_id"] = source_observation_id
        self.supervisor_phase = SupervisorPhase.RUNNING
        return True

    def watchdogs(self, cur_time_secs):
        """Check deadlines at ``cur_time_secs`` and latch any detected timeout.

        Checks run only when called, not in an independent background task.
        """
        if (
            self.time_last_cycle_secs is not None
            and cur_time_secs - self.time_last_cycle_secs >= self.config.loop_timeout
        ):
            self.stop("loop_progress", cur_time_secs)
        if self.supervisor_phase in (SupervisorPhase.ARMED, SupervisorPhase.RUNNING):
            if (
                cur_time_secs - self.time_action_watchdog_refreshed_secs
                >= self.config.action_timeout
            ):
                self.stop("action_timeout", cur_time_secs)
            elif (
                self.time_last_robot_state_received_secs is None
                or cur_time_secs - self.time_last_robot_state_received_secs
                >= self.config.robot_timeout
            ):
                self.stop("robot_state_timeout", cur_time_secs)
            elif (
                self.accepted_policy_command
                and cur_time_secs
                - self.observation_received_times_secs.get(
                    self.accepted_policy_command["state_id"], float("-inf")
                )
                >= self.config.observation_timeout
            ):
                self.stop("observation_timeout", cur_time_secs)

    def cycle(self, cur_time_secs):
        """Select and return one dry-run output at ``cur_time_secs``.

        The caller schedules cycles. This method neither sleeps nor sends
        motor commands.
        """
        self.watchdogs(cur_time_secs)
        self.time_last_cycle_secs = cur_time_secs
        joint_count = self.config.joints
        if self.supervisor_phase == SupervisorPhase.RUNNING:
            self.last_selected_command = dict(
                self.accepted_policy_command, kind="policy"
            )
        else:
            self.last_selected_command = {
                "kind": "damping",
                "q": [0.0] * joint_count,
                "dq": [0.0] * joint_count,
                "kp": [0.0] * joint_count,
                "kd": [self.config.damping] * joint_count,
                "tau": [0.0] * joint_count,
            }
            if (
                self.supervisor_phase == SupervisorPhase.STOP_LATCHED
                and self.time_first_damping_selected_secs is None
            ):
                self.time_first_damping_selected_secs = cur_time_secs
        return self.last_selected_command

    def status(self, cur_time_secs):
        """Return a status snapshot with ages in seconds relative to ``cur_time_secs``.

        ``cur_time_secs`` is local monotonic time. This method does not run watchdogs.
        """
        return {
            "boot_id": self.supervisor_instance_id,
            "phase": self.supervisor_phase.value,
            "fault_reason": self.stop_reason,
            "session": self.policy_session_id,
            "dry_run": True,
            "motor_publisher": False,
            "can_enforce_motor_damping": False,
            "motor_application_latency": None,
            "controller_radio_live": None,
            "state": deepcopy(self.latest_robot_observation),
            "start_button_pressed": self.start_button_pressed,
            "action_age": None
            if self.time_action_watchdog_refreshed_secs is None
            else cur_time_secs - self.time_action_watchdog_refreshed_secs,
            "robot_state_age": None
            if self.time_last_robot_state_received_secs is None
            else cur_time_secs - self.time_last_robot_state_received_secs,
            "stop_at": self.time_stop_latched_secs,
            "first_fake_damping_at": self.time_first_damping_selected_secs,
            "output": deepcopy(self.last_selected_command),
        }
