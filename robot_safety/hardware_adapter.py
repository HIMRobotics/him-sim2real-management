"""Send safety-approved commands to the robot through the Unitree SDK."""

import math
import os
import sys
import time
from copy import deepcopy
from dataclasses import replace
from threading import Event, Lock, RLock, Thread

from .balance_policy import MODEL_PATH, BalancePolicy
from .config import Config
from .controller import (
    LeftAndBDetector,
    StartDoublePressDetector,
    decode_b_pressed,
    decode_f3_pressed,
    decode_left_pressed,
)
from .feedback import feedback_from_sdk
from .hardware_constants import (
    DAMPING_FSM_ID,
    G1_29_JOINT_NAMES,
    SDK_RPC_SEND_CODE,
    UPPER_BODY_INDICES,
)
from .hardware_profile import (
    DAMPING_RETRY_SECS,
    HARDWARE_BATTERY_TIMEOUT_SECS,
    HARDWARE_FEEDBACK_TIMEOUT_SECS,
    BODY_REPEAT_SECS,
    HARDWARE_PERIOD_SECS,
    HARDWARE_WATCHDOG_POLL_SECS,
    SHUTDOWN_WAIT_SECS,
)
from .joint_checks import (
    MIN_BATTERY_PERCENT,
    MOTOR_CASING_LIMIT_C,
    MOTOR_WINDING_LIMIT_C,
    TORQUE_TARGET_FRACTION,
    check_base_motion,
    check_bound,
    check_measured_velocity,
    check_torque,
    checked_targets,
    clip_position,
    torque_demand,
)
from .joint_diagnostics import JointDeltaDiagnostics
from .holosoma_policy import HolosomaLocomotionPolicy, holosoma_weights
from .control_trace import ControlTrace, default_trace_path
from .sdk_latest import poll_deadline
from .startup_balance import StartupBalance

# Python hands the interpreter to a waiting thread only this often. The
# default 5 ms turns the 2 ms refresh into about 170 Hz.
INTERPRETER_SWITCH_SECS = 0.001
SOFT_RESET_UPRIGHT_MIN = math.cos(math.radians(30.0))
# These stops cannot rely on the recovery output worker or explicitly request damping.
DAMP_ONLY_REASONS = frozenset({
    "controller_double_start", "hardware_controller_double_start", "controller_stop",
    "operator_stop", "external_abort",
    "hardware_shutdown", "hardware_output_worker_stalled", "hardware_output_loop_stall",
    "hardware_write_failed", "hardware_release_failed", "hardware_machine_type_changed",
    "hardware_feedback_timeout", "invalid_hardware_feedback", "missing_hardware_imu",
    "low_battery", "invalid_battery_feedback", "battery_feedback_timeout",
})


def whole_body_repeat_wait(next_tick, now, period=BODY_REPEAT_SECS):
    """Return the sleep and the deadline after this 2 ms refresh.

    A late cycle waits one full period. Spinning holds the interpreter and
    the low-rate battery topic stops arriving.
    """
    return poll_deadline(next_tick, now, period)


class G1HardwareAdapter:
    """Apply approved motor commands and handle recovery or damping."""

    def __init__(self, profile, clock=time.monotonic, *, whole_body=False,
                 launch_gate=False, stance_blend_secs=5.0):
        self.profile = profile
        self.whole_body = whole_body
        self.launch_gate = bool(launch_gate or whole_body)
        if not math.isfinite(stance_blend_secs) or stance_blend_secs < 0:
            raise ValueError("stance blend duration must be finite and non-negative")
        self.stance_blend_secs = float(stance_blend_secs)
        self.policy_launched = not self.launch_gate
        self.launch_requested = False
        self.stance_from = None
        self.stance_q = None
        self.stance_since = None
        self.stance_held_announced = False
        self.launch_announced = False
        self.launch_detector = LeftAndBDetector()
        self.left_b_count = 0
        self.left_b_at_secs = None
        self.release_pending = False
        self.motion_released_at_secs = None
        self.clock = clock
        self.transport = None
        self.lock = RLock()
        self.stop_lock = Lock()
        self.battery_lock = Lock()
        self.trace = None
        self._trace_record = None
        self._policy_q = None
        self._last_step_secs = None
        self.feedback = None
        self.feedback_tick = None
        self.battery_percent = None
        self.battery_time = None
        self.machine_type = None
        self.abort_reason = None
        self.armed = False
        self.time_ready_mode_confirmed_secs = None
        self.arm_rejection_reason = None
        self._reset_session()
        self.last_submission_secs = None
        self.stop_requested = Event()
        self.shutdown_requested = Event()
        self.damping_rpc_code = None
        self.walk_rpc_code = None
        self.walk_stop = False
        self.balance_policy = None
        self.soft_reset_policy = None
        self.startup_policy = None
        requested = os.environ.get("G1_STARTUP_POLICY", "groot").strip().lower()
        if requested not in ("groot", "holosoma"):
            print(
                "Unknown G1_STARTUP_POLICY {0!r}; startup stays on GR00T Balance.".format(requested),
                flush=True,
            )
            requested = "groot"
        self.startup_policy_name = requested
        self.balance_released = False
        self.quat = (1.0, 0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.f3_pressed = False
        self.reported_fsm_id = None
        self.arm_release_written = False
        self.hold_failed = False
        self.release_damp_when_confirmed = False
        self.damp_settled = False
        self.pose_snapshot = None
        self.pose_snapshot_secs = None
        self.workers = []
        window = Config().double_press_window_secs
        self.button_detector = StartDoublePressDetector(
            window, HARDWARE_FEEDBACK_TIMEOUT_SECS
        )
        self.f3_detector = StartDoublePressDetector(
            window, HARDWARE_FEEDBACK_TIMEOUT_SECS
        )

    def receive_state(self, message):
        """Read machine type, measured joints and double Start directly from the SDK."""
        with self.lock:
            try:
                sample = feedback_from_sdk(message, self.clock())
                self.machine_type = message.mode_machine
                stop = self.button_detector.update(
                    sample.start_button_pressed, sample.received_at_secs
                )
                self.f3_pressed = decode_f3_pressed(message.wireless_remote)
                soft = self.f3_detector.update(
                    self.f3_pressed, sample.received_at_secs
                )
                launch = self.launch_detector.update(
                    decode_left_pressed(message.wireless_remote),
                    decode_b_pressed(message.wireless_remote),
                )
                tick = getattr(message, "tick", None)
                if tick is not None and self.feedback_tick is not None:
                    delta = (int(tick) - self.feedback_tick) % (2 ** 32)
                    if delta >= 2 ** 31:
                        raise ValueError("hardware_feedback_tick_reversed")
                    if delta == 0 and self.feedback is not None:
                        sample = replace(sample, received_at_secs=self.feedback.received_at_secs)
                if tick is not None:
                    self.feedback_tick = int(tick)
                if launch and (self.feedback is None or sample.received_at_secs > self.feedback.received_at_secs):
                    self.left_b_count += 1
                    self.left_b_at_secs = sample.received_at_secs
                self.feedback = sample
                imu = getattr(message, "imu_state", None)
                if imu is not None:
                    self.quat = tuple(float(value) for value in imu.quaternion)
                    self.gyro = tuple(float(value) for value in imu.gyroscope)
                if self.armed and self.abort_reason is None:
                    self._abort_if_motion_exceeds_limits()
                    if self.whole_body and self.first_output_secs is not None:
                        self.startup.observe(self.clock(), sample.received_at_secs,
                                             sample.q, sample.dq, sample.quat, sample.gyro)
                        if launch and not stop and not soft and not self.abort_reason:
                            self.startup.authorize(self.clock())
                    elif launch and self.launch_gate and not self.whole_body:
                        self.launch_requested = True
                with self.stop_lock:
                    self.pose_snapshot = tuple(
                        sample.q[index] for index in UPPER_BODY_INDICES
                    )
                    self.pose_snapshot_secs = sample.received_at_secs
                    if stop:
                        self.hold_failed = True
                        self.walk_stop = False
                        self.release_damp_when_confirmed = not self.whole_body
                    elif soft and not self.hold_failed:
                        self.walk_stop = True
                if stop:
                    self.abort(
                        "hardware_controller_double_start", request_damping=True
                    )
                elif soft:
                    self.abort("hardware_controller_double_f3")
                if (
                    self.armed
                    and self.machine_type != self.profile.expected_machine_type
                ):
                    self.abort("hardware_machine_type_changed")
            except (AttributeError, TypeError, ValueError, OverflowError):
                self.abort("invalid_hardware_feedback")

    def receive_battery(self, message):
        """Copy the separate, lower-frequency BMS reading.

        This lock is not the feedback lock. The 500 Hz state callback must
        not be able to delay a battery sample for the whole timeout.
        """
        try:
            charge = float(message.soc)
            if not math.isfinite(charge) or not 0 <= charge <= 100:
                raise ValueError("invalid_battery_feedback")
        except (AttributeError, TypeError, ValueError):
            with self.stop_lock:
                armed = self.armed
            self.abort("invalid_battery_feedback", request_damping=armed)
            return
        with self.stop_lock:
            armed = self.armed
        with self.battery_lock:
            self.battery_percent, self.battery_time = charge, self.clock()
        if armed and charge < MIN_BATTERY_PERCENT:
            self.abort("low_battery", request_damping=True)

    def check_health(self):
        """Require current health telemetry before enabling or continuing output."""
        if self.whole_body and (self.feedback is None or not self.feedback.quat or not self.feedback.gyro):
            raise ValueError("missing_hardware_imu")
        with self.battery_lock:
            battery_time = self.battery_time
            battery_percent = self.battery_percent
        if (battery_time is None
                or not 0 <= self.clock() - battery_time < HARDWARE_BATTERY_TIMEOUT_SECS):
            raise ValueError("battery_feedback_timeout")
        if battery_percent < MIN_BATTERY_PERCENT:
            raise ValueError("low_battery")
        if self.feedback is None or len(self.feedback.temperature) != 29:
            raise ValueError("missing_motor_temperature")
        for index, temperatures in enumerate(self.feedback.temperature):
            if len(temperatures) != 2:
                raise ValueError("invalid_motor_temperature")
            name = G1_29_JOINT_NAMES[index]
            check_bound("motor_casing_temperature_limit", name, temperatures[0], MOTOR_CASING_LIMIT_C, "C")
            check_bound("motor_winding_temperature_limit", name, temperatures[1], MOTOR_WINDING_LIMIT_C, "C")
        if self.feedback.tau_est:
            for index, limits in self._monitored_joints():
                check_torque(G1_29_JOINT_NAMES[index], limits, self.feedback.tau_est[index])

    def feedback_fresh(self, now):
        return (
            self.feedback is not None
            and 0
            <= now - self.feedback.received_at_secs
            < HARDWARE_FEEDBACK_TIMEOUT_SECS
        )

    def check_ready_mode(self):
        """Read stock mode before opening a policy session, without motor writes."""
        self.time_ready_mode_confirmed_secs = None
        self.arm_rejection_reason = None
        if self.armed or self.abort_reason:
            return self.reject_arm("already armed or stopped")
        code, fsm_id = self.transport.read_ready_fsm()
        with self.lock:
            if self.abort_reason:
                return self.reject_arm(f"already stopped: {self.abort_reason}")
            if code != 0:
                return self.reject_arm(
                    f"ready-mode query failed: rpc_code={code}, fsm_id={fsm_id}"
                )
            allowed = self.profile.allowed_start_fsm_ids
            if self.startup_policy_name == "holosoma":
                allowed = allowed + (DAMPING_FSM_ID,)
            if fsm_id not in allowed:
                return self.reject_arm(
                    f"ready mode: measured={fsm_id}, "
                    f"allowed={allowed}"
                )
            self.time_ready_mode_confirmed_secs = self.clock()
            return True

    def arm(self):
        """Require confirmed stock mode, fresh telemetry and physical limits."""
        with self.lock:
            now = self.clock()
            if self.abort_reason:
                return self.reject_arm(f"already stopped: {self.abort_reason}")
            if self.armed:
                return self.reject_arm("already armed")
            confirmed = self.time_ready_mode_confirmed_secs
            self.time_ready_mode_confirmed_secs = None
            if (
                confirmed is None
                or not 0 <= now - confirmed < HARDWARE_FEEDBACK_TIMEOUT_SECS
            ):
                return self.reject_arm(
                    "ready-mode confirmation missing or older than 0.2 secs"
                )
            if self.machine_type != self.profile.expected_machine_type:
                return self.reject_arm(
                    f"machine type: measured={self.machine_type}, "
                    f"required={self.profile.expected_machine_type}"
                )
            if not self.feedback_fresh(now):
                age = (
                    None
                    if self.feedback is None
                    else now - self.feedback.received_at_secs
                )
                return self.reject_arm(
                    f"feedback age: measured={age} secs, "
                    f"required=0 <= age < {HARDWARE_FEEDBACK_TIMEOUT_SECS} secs"
                )
            try:
                self.check_health()
                check_base_motion(self.gyro, self.quat)
            except ValueError as error:
                return self.reject_arm(str(error))
            if self.feedback.start_button_pressed or self.f3_pressed:
                return self.reject_arm("Start and F3 must be released")
            for index, limits in self._monitored_joints():
                joint_name = G1_29_JOINT_NAMES[index]
                position = self.feedback.q[index]
                velocity = self.feedback.dq[index]
                if not limits.position_min_rad <= position <= limits.position_max_rad:
                    return self.reject_arm(
                        f"{joint_name} position: measured={position:.6f} rad, "
                        f"allowed=[{limits.position_min_rad}, {limits.position_max_rad}] rad"
                    )
                if self.whole_body:
                    try:
                        check_measured_velocity(joint_name, limits, velocity)
                    except ValueError as error:
                        return self.reject_arm(str(error))
                elif abs(velocity) > self.profile.initial_velocity_tolerance_rad_secs:
                    return self.reject_arm(
                        f"{joint_name} speed: measured={velocity:.6f} rad/s, "
                        f"maximum absolute={self.profile.initial_velocity_tolerance_rad_secs} rad/s"
                    )
            if self.whole_body:
                if not self._ensure_balance_policy():
                    return self.reject_arm("startup_balance_model_unavailable")
                if not self._ensure_soft_reset_policy():
                    return self.reject_arm("soft_reset_model_unavailable")
                if not self._ensure_startup_policy():
                    return self.reject_arm("startup_balance_model_unavailable")
            # Serialize granting control with an abort arriving from another thread.
            with self.stop_lock:
                if self.abort_reason:
                    return self.reject_arm(f"already stopped: {self.abort_reason}")
                self._reset_session()
                if self.whole_body:
                    self.startup_policy.reset()
                    self.startup.policy = self.startup_policy
                self.last_submission_secs = now
                self.armed = True
            return True

    def _reset_session(self):
        """Initialize output history before workers start or under both adapter locks."""
        self.command = None
        self.command_expires_secs = None
        self.q_applied = None
        self.dq_applied = self.tau_applied = None
        self.command_time = None
        self.first_output_secs = self.last_output_secs = None
        self.last_worker_cycle_secs = None
        self.last_written_ownership_weight = 0.0
        self.ownership_taken = False
        self.hold_targets = None
        self.soft_reset_active = False
        self.automatic_soft_reset = False
        self.joint_diagnostics = JointDeltaDiagnostics()
        self.position_warnings = {}
        self.startup = StartupBalance()
        self.applied_gains = {}
        self.policy_launched = False if self.whole_body else not self.launch_gate
        self.launch_requested = False

    def reject_arm(self, reason):
        """Record the failed readiness check without changing motor settings."""
        self.arm_rejection_reason = reason
        return False

    def _upright_for_soft_reset(self, feedback):
        """Require fresh orientation within 30 degrees of world-up."""
        if feedback is None or not (
            0 <= self.clock() - feedback.received_at_secs < HARDWARE_FEEDBACK_TIMEOUT_SECS
        ):
            return False
        quat = feedback.quat
        if (len(quat) != 4 or len(feedback.gyro) != 3
                or len(feedback.q) != 29 or len(feedback.dq) != 29
                or not all(math.isfinite(v) for values in (
                    quat, feedback.gyro, feedback.q, feedback.dq) for v in values)):
            return False
        norm = sum(v * v for v in quat)
        if not 0.9 <= norm <= 1.1:
            return False
        # Dot product of the measured body-up axis with world-up, independent of yaw.
        upright = 1 - 2 * (quat[1] ** 2 + quat[2] ** 2) / norm
        return upright >= SOFT_RESET_UPRIGHT_MIN - 1e-12

    def abort(self, reason, request_damping=False):
        """Latch the submission fault and select recovery or damping."""
        # Keep stop requests independent of a blocked SDK write.
        with self.stop_lock:
            first_fault = self.abort_reason is None
            if (first_fault and reason not in DAMP_ONLY_REASONS
                    and self.whole_body and self.armed and not self.hold_failed
                    and (self.ownership_taken or self.balance_released)
                    and self.soft_reset_policy is not None
                    and self._upright_for_soft_reset(self.feedback)):
                self.automatic_soft_reset = reason != "hardware_controller_double_f3"
                self.walk_stop = True
            soft_reason = "hardware_controller_double_f3"
            soft_balance = self.walk_stop and not self.hold_failed and (
                reason == soft_reason
                or (self.abort_reason == soft_reason and reason == "policy_container_exit")
                or (self.automatic_soft_reset and (
                    first_fault or reason in (self.abort_reason, "policy_container_exit")))
            )
            if soft_balance and self.abort_reason is None:
                # The recovery writer replaces the submission and its heartbeat.
                self.q_applied = None
                self.soft_reset_active = False
                self.last_worker_cycle_secs = self.clock()
            if first_fault:
                self.abort_reason = reason
                self.startup.fail()
            self.command = None
            if self.transport is not None and not (soft_balance and self.soft_reset_active):
                self.transport.clear_body()
            if not soft_balance and (request_damping or (self.whole_body and self.armed
                                     and (self.ownership_taken or self.balance_released))):
                self.hold_failed = True
                self.walk_stop = False
                self.automatic_soft_reset = False
            if self.armed or request_damping or self.walk_stop or self.hold_failed:
                self.stop_requested.set()
        if first_fault and self.trace is not None:
            try:
                self.trace.emit_abort(reason, self.clock())
            except Exception as error:  # noqa: BLE001 - logging must not stop the motors
                print("Control trace failed: {0}".format(error), flush=True)
                self.trace = None

    def submit(self, command, expires_at_secs):
        """Copy gate-approved targets without renewing their original deadline."""
        with self.lock:
            if self.abort_reason or not self.armed:
                return
            now = self.clock()
            if (
                not math.isfinite(expires_at_secs)
                or now - self.last_submission_secs >= HARDWARE_FEEDBACK_TIMEOUT_SECS
                or not now < expires_at_secs
                or expires_at_secs - now > HARDWARE_FEEDBACK_TIMEOUT_SECS
                or (
                    self.command_expires_secs is not None
                    and now >= self.command_expires_secs
                )
            ):
                self.abort("hardware_command_expired")
                return
            self.command = deepcopy(command)
            self.command_expires_secs = expires_at_secs
            self.last_submission_secs = now

    def step(self):
        """Check and publish one command from the single output worker."""
        if self.walk_stop and not self.hold_failed and self.stop_requested.is_set():
            self.stop_once()
            return
        with self.lock:
            if not self.armed or self.abort_reason or self.stop_requested.is_set():
                return
            now = self.clock()
            if not self.feedback_fresh(now):
                self.abort("hardware_feedback_timeout")
                return
            if (
                self.last_worker_cycle_secs is not None
                and now - self.last_worker_cycle_secs
                >= self.profile.max_publish_gap_secs
            ):
                self.abort("hardware_output_loop_stall")
                return
            self.last_worker_cycle_secs = now
            if now - self.last_submission_secs >= HARDWARE_FEEDBACK_TIMEOUT_SECS:
                self.abort("hardware_gate_silent")
                return
            if self.command is None:
                return
            if now >= self.command_expires_secs:
                self.abort("hardware_command_expired")
                return
            try:
                self.check_health()
            except ValueError as error:
                self.abort(str(error), request_damping=True)
                return
            try:
                self.record_response()
                diagnostic_command = dict(self.command)
                diagnostic_command["q"] = list(self.command["q"])
                for index, limits in self._monitored_joints():
                    diagnostic_command["q"][index] = clip_position(
                        G1_29_JOINT_NAMES[index], self.command["q"][index],
                        limits.position_min_rad, limits.position_max_rad)
                self.joint_diagnostics.record(
                    diagnostic_command, self.feedback
                )
                infer_started = self.clock()
                targets = self.checked_targets(now)
                if self.trace is not None:
                    self._trace_record = self._control_snapshot(
                        now, targets, self.clock() - infer_started
                    )
                elapsed = (
                    0
                    if self.first_output_secs is None
                    else now - self.first_output_secs
                )
                weight_progress = min(elapsed / self.profile.handover_secs, 1.0)
                weight = 1.0 if self.whole_body else weight_progress**2 * (3 - 2 * weight_progress)
                if self.stop_requested.is_set():
                    return
            except Exception as error:  # noqa: BLE001
                self.abort(f"hardware_output_error:{type(error).__name__}: {error}")
                return
        self._emit_trace()
        if self.whole_body and self.armed and self.command is not None and not self.balance_released:
            if self.abort_reason or self.stop_requested.is_set():
                return
            self.release_pending = True
            try:
                self.walk_rpc_code = self.transport.release_motion()
                if self.walk_rpc_code != 0:
                    # A timeout can leave ownership uncertain. Require an
                    # acknowledged restoration before treating stock control as active.
                    try:
                        restored = self.transport.restore_motion() == 0
                    except Exception:  # noqa: BLE001
                        restored = False
                    self.balance_released = not restored
                    self.abort("hardware_release_failed")
                    return
                self.balance_released = True
                self.motion_released_at_secs = self.clock()
                with self.lock:
                    if self.abort_reason or self.stop_requested.is_set():
                        self.abort(self.abort_reason or "hardware_stop_during_release")
                        return
                    self.startup_policy.reset()
                    with self.stop_lock:
                        if not self.abort_reason and not self.stop_requested.is_set():
                            self.startup = StartupBalance(self.startup_policy)
            except Exception:  # noqa: BLE001 - An uncertain release requires restoration.
                self.balance_released = True
                self.abort("hardware_release_failed")
                return
            finally:
                self.release_pending = False
                with self.lock:
                    # ReleaseMode blocks longer than the publish gap. The next
                    # cycle starts from when the RPC returns.
                    self.last_worker_cycle_secs = self.clock()
            # Recheck fresh feedback, expiry and stops after the blocking RPC.
            # Start the entry blend now instead of waiting another output period.
            return self.step()
        # SDK writes must not hold the feedback lock or delay controller monitoring.
        try:
            if self.stop_requested.is_set():
                return
            written = (self.transport.write_body(targets, self.machine_type)
                       if self.whole_body else self.transport.write_arm(targets, weight))
            with self.lock:
                if not written:
                    self.abort("hardware_write_failed")
                    return
                self.last_written_ownership_weight = weight
                if weight == 1.0:
                    self.q_applied = {
                        index: values["q"] for index, values in targets.items()
                    }
                    self.dq_applied = {i: v["dq"] for i, v in targets.items()}
                    self.tau_applied = {i: v["tau"] for i, v in targets.items()}
                    self.applied_gains = {i: (v["kp"], v["kd"]) for i, v in targets.items()}
                    self.command_time = self.clock()
                if weight > 0:
                    with self.stop_lock:
                        self.ownership_taken = True
                if self.first_output_secs is None:
                    self.first_output_secs = now
                self.last_output_secs = now
        except Exception as error:  # noqa: BLE001
            self.abort(f"hardware_output_error:{type(error).__name__}: {error}")

    def _emit_trace(self):
        record = self._trace_record
        self._trace_record = None
        if record is None or self.trace is None:
            return
        try:
            self.trace.note_step(record)
        except Exception as error:  # noqa: BLE001 - logging must not stop the motors
            print("Control trace failed: {0}".format(error), flush=True)
            self.trace = None

    def _control_snapshot(self, now, targets, infer_secs):
        """Numbers from one checked command. Called with the feedback lock held."""
        measured = self.feedback.q
        proposed = self._policy_q if self._policy_q is not None else self.command["q"]
        speeds = self.feedback.dq
        step_period = None if self._last_step_secs is None else now - self._last_step_secs
        self._last_step_secs = now
        gap_joint, gap = None, 0.0
        rewrite_joint, rewrite = None, 0.0
        speed_joint, speed = None, 0.0
        command_delta = 0.0
        for index, _limits in self._monitored_joints():
            if index not in targets:
                continue
            name = G1_29_JOINT_NAMES[index]
            written = targets[index]["q"]
            gap_value = written - measured[index]
            rewrite_value = written - proposed[index]
            speed_value = speeds[index]
            if abs(gap_value) >= abs(gap):
                gap_joint, gap = name, gap_value
            if abs(rewrite_value) >= abs(rewrite):
                rewrite_joint, rewrite = name, rewrite_value
            if abs(speed_value) >= abs(speed):
                speed_joint, speed = name, speed_value
            if self.q_applied is not None and index in self.q_applied:
                command_delta = max(command_delta, abs(written - self.q_applied[index]))
        with self.battery_lock:
            battery_time = self.battery_time
        gyro = tuple(self.gyro)
        return {
            "now": now,
            "policy": self.startup_policy_name,
            "phase": self.startup.phase,
            "since_output_s": (
                None if self.first_output_secs is None else now - self.first_output_secs
            ),
            "sample_age_s": now - self.feedback.received_at_secs,
            "step_period_s": step_period,
            "policy_period_s": self.startup.infer_period,
            "infer_s": infer_secs,
            "gyro": [round(float(value), 5) for value in gyro],
            "gyro_abs_max": round(max(abs(float(value)) for value in gyro), 5),
            "dq_joint": speed_joint,
            "dq": round(float(speed), 5),
            "gap_joint": gap_joint,
            "gap_rad": round(float(gap), 5),
            "rewrite_joint": rewrite_joint,
            "rewrite_rad": round(float(rewrite), 5),
            "command_step_rad": round(float(command_delta), 5),
            "battery_age_s": None if battery_time is None else now - battery_time,
            "network": self.startup.entry_goal is None and self.startup.entry_started is not None,
        }

    def joint_diagnostics_snapshot(self):
        """Skip a busy output lock so diagnostics cannot stall ROS stop handling."""
        if not self.lock.acquire(blocking=False):
            return None
        try:
            result = self.joint_diagnostics.snapshot()
            for name, warning in self.position_warnings.items():
                if warning.get("reason") == "joint_position_clipped":
                    result[name]["position_clipping"] = warning.copy()
                if warning.get("velocity_reason") == "joint_velocity_clipped":
                    result[name]["velocity_clipping"] = warning.copy()
                if "torque_target_clipping" in warning:
                    result[name]["torque_target_clipping"] = warning["torque_target_clipping"].copy()
            if self.q_applied is not None:
                for index, limits in self._monitored_joints():
                    if index in self.q_applied:
                        result[G1_29_JOINT_NAMES[index]]["applied_command"] = {
                            "q": self.q_applied[index], "dq": self.dq_applied[index],
                            "tau": self.tau_applied[index],
                            "kp": self.applied_gains[index][0], "kd": self.applied_gains[index][1],
                            "written_at_secs": self.command_time,
                        }
            return result
        finally:
            self.lock.release()

    def record_response(self):
        """Record later feedback without treating response error as a fault."""
        if self.q_applied is not None and self.feedback.received_at_secs > self.command_time:
            self.joint_diagnostics.record_response(self.q_applied, self.feedback)

    def _abort_if_motion_exceeds_limits(self):
        """Enforce physical speed ceilings immediately on fresh feedback."""
        try:
            self.check_health()
            for index, limits in self._monitored_joints():
                check_measured_velocity(G1_29_JOINT_NAMES[index], limits, self.feedback.dq[index])
            check_base_motion(self.gyro, self.quat)
            if self.q_applied is not None:
                for index, limits in self._monitored_joints():
                    if index not in self.q_applied:
                        continue
                    tau_demand = torque_demand(
                        self.q_applied[index], self.dq_applied[index], self.applied_gains[index][0],
                        self.applied_gains[index][1], self.tau_applied[index], self.feedback.q[index],
                        self.feedback.dq[index],
                    )
                    check_torque(G1_29_JOINT_NAMES[index], limits, tau_demand)
        except ValueError as error:
            self.abort(str(error), request_damping=True)

    def _monitored_joints(self):
        """Legs use the same command rules as the waist and arms."""
        return (
            *zip(range(len(self.profile.legs)), self.profile.legs),
            *zip(UPPER_BODY_INDICES, self.profile.joints),
        )

    def _stance_command(self, now):
        """Ease from the measured pose to the first command, then wait for left and B."""
        if self.stance_from is None:
            self.stance_from = [float(value) for value in self.feedback.q]
            self.stance_q = [float(value) for value in self.command["q"]]
            self.stance_since = now
        elapsed = now - self.stance_since
        alpha = 1.0 if self.stance_blend_secs == 0 else min(
            1.0, max(0.0, elapsed / self.stance_blend_secs)
        )
        if alpha >= 1.0 and self.launch_requested:
            self.policy_launched = True
            if not self.launch_announced:
                self.launch_announced = True
                print("left + B accepted; launching the policy", flush=True)
            return self.command
        if alpha >= 1.0 and not self.stance_held_announced:
            self.stance_held_announced = True
            print("holding the first pose. press left + B to launch.", flush=True)
        blended = dict(self.command)
        blended["q"] = [
            (1.0 - alpha) * start + alpha * goal
            for start, goal in zip(self.stance_from, self.stance_q)
        ]
        count = len(blended["q"])
        blended["dq"] = [0.0] * count
        blended["tau"] = [0.0] * count
        return blended

    def checked_targets(self, now):
        """Check proposals with the profile gains used by the arm controller."""
        command = self.command
        entries = list(self._monitored_joints())
        if self.whole_body:
            command = dict(command, kp=[v.kp for _, v in entries],
                           kd=[v.kd for _, v in entries])
            command = self.startup.command(
                now, self.feedback.received_at_secs,
                dict(q=self.feedback.q, dq=self.feedback.dq, quat=self.quat, gyro=self.gyro),
                command,
            )
            self.policy_launched = self.startup.handoff is not None
            self._policy_q = command["q"]
            entries = [(i, replace(v, kp=float(command["kp"][i]), kd=float(command["kd"][i])))
                       for i, v in entries]
        elif self.launch_gate and not self.policy_launched:
            command = self._stance_command(now)
        targets = checked_targets(
            command,
            ((index, G1_29_JOINT_NAMES[index], limits) for index, limits in entries),
            self.feedback.q, self.feedback.dq, initial=self.last_output_secs is None,
            initial_velocity=self.profile.initial_velocity_tolerance_rad_secs,
            warnings=self.position_warnings,
            torque_target_fraction=TORQUE_TARGET_FRACTION if self.whole_body else None,
        )
        return targets if self.whole_body else {index: targets[index] for index in UPPER_BODY_INDICES}

    def stop_once(self):
        """Apply recovery, damping, or an arm hold for the latched stop."""
        # SDK calls stay off the arm-write lock so a stuck motion write cannot delay the stop.
        if self.release_pending:
            return
        with self.stop_lock:
            if not self.stop_requested.is_set():
                return
            hard = self.hold_failed
            walk = self.walk_stop and not hard
            owned = self.ownership_taken
        if hard:
            self._request_stock_damping()
            return
        if walk:
            self._publish_balance()
            return
        if not owned:
            return
        if not self._publish_pose_hold():
            with self.stop_lock:
                self.hold_failed = True
            self._request_stock_damping()

    def _publish_balance(self):
        """Run zero-velocity Holosoma recovery on all 29 joints."""
        policy = self.soft_reset_policy
        if policy is None:
            if self.balance_released or self.ownership_taken:
                self.hold_failed = True
                self._request_stock_damping()
            return
        if not self.balance_released:
            self.release_pending = True
            try:
                self.walk_rpc_code = self.transport.release_motion(announce_soft_reset=True)
            except Exception:  # noqa: BLE001
                self.walk_rpc_code = None
                return
            finally:
                self.release_pending = False
                self.last_worker_cycle_secs = self.clock()
            if self.walk_rpc_code != 0:
                return
            self.balance_released = True
        try:
            feedback = self.feedback
            if feedback is None or not self.feedback_fresh(self.clock()):
                raise ValueError("stale balance feedback")
            # A latched submission fault must not disable health checks for the
            # replacement controller. Recovery still drives the same motors.
            self.check_health()
            if self.automatic_soft_reset and not self._upright_for_soft_reset(feedback):
                raise ValueError("robot is not upright for automatic recovery")
            if not self.soft_reset_active:
                policy.reset()
                self.soft_reset_active = True
            quat, gyro = ((feedback.quat, feedback.gyro) if self.automatic_soft_reset
                          else (self.quat, self.gyro))
            q = policy.update(feedback.q, feedback.dq, quat, gyro)
            if any(len(values) != 29 for values in (q, policy.kp, policy.kd)):
                raise ValueError("invalid balance output shape")
            if not all(math.isfinite(value) for values in (q, policy.kp, policy.kd) for value in values):
                raise ValueError("nonfinite balance output")
            if not self.feedback_fresh(self.clock()) or not (
                0 <= self.clock() - feedback.received_at_secs < HARDWARE_FEEDBACK_TIMEOUT_SECS
            ):
                raise ValueError("stale balance feedback")
            # Retain the recovery policy's gains, but never bypass the robot's
            # physical position and torque envelope for a replacement controller.
            entries = tuple(
                (index, G1_29_JOINT_NAMES[index], replace(
                    limits, kp=float(policy.kp[index]), kd=float(policy.kd[index])))
                for index, limits in self._monitored_joints()
            )
            targets = checked_targets(
                {"q": q, "dq": [0.0] * 29, "tau": [0.0] * 29},
                entries, feedback.q, feedback.dq, initial=False,
                initial_velocity=self.profile.initial_velocity_tolerance_rad_secs,
                warnings=self.position_warnings,
                torque_target_fraction=TORQUE_TARGET_FRACTION,
            )
            if self.hold_failed or not self.walk_stop:
                return
            self.check_health()
            if self.automatic_soft_reset and not self._upright_for_soft_reset(self.feedback):
                raise ValueError("robot is not upright for automatic recovery")
            if not self.transport.write_body(targets, self.machine_type or 0):
                raise RuntimeError("balance write failed")
            if self.hold_failed or not self.walk_stop:
                self.transport.clear_body()
                return
            self.last_worker_cycle_secs = self.clock()
        except Exception as exc:  # noqa: BLE001 - Balance failures enter damping.
            self.abort_reason = "balance_output_error: " + str(exc)
            with self.stop_lock:
                self.hold_failed = True
                self.walk_stop = False
                self.automatic_soft_reset = False
            self.transport.clear_body()
            self._request_stock_damping()

    def _ensure_soft_reset_policy(self):
        """Prime a dedicated Holosoma recovery policy before motor ownership."""
        if self.soft_reset_policy is not None:
            return True
        try:
            policy = HolosomaLocomotionPolicy(str(holosoma_weights()))
            policy.warmup()
            self.soft_reset_policy = policy
        except Exception as exc:  # noqa: BLE001
            print("Holosoma soft-reset preparation failed: {0}. "
                  "Set HIM_HOLOSOMA_ONNX to fastsac_g1_29dof.onnx.".format(exc), flush=True)
            return False
        return True

    def _ensure_balance_policy(self):
        """Load and prime inference at startup, then retain the prepared policy."""
        if self.balance_policy is not None:
            return True
        try:
            policy = BalancePolicy(str(MODEL_PATH))
            policy.warmup()
            self.balance_policy = policy
        except Exception as exc:  # noqa: BLE001
            print(
                f"balance preparation failed at {MODEL_PATH}: {exc}. "
                "GR00T startup unavailable until restart.",
                flush=True,
            )
            return False
        return True

    def _ensure_startup_policy(self):
        """Load startup independently from the Holosoma recovery policy."""
        if self.startup_policy is not None:
            return True
        if self.startup_policy_name == "groot":
            self.startup_policy = self.balance_policy
            return self.startup_policy is not None
        try:
            policy = HolosomaLocomotionPolicy(str(holosoma_weights()))
            policy.warmup()
            self.startup_policy = policy
            print(
                "Holosoma: easing to the standing crouch over {0:.0f}s, "
                "then the standing policy.".format(policy.entry_ramp_seconds),
                flush=True,
            )
        except Exception as exc:  # noqa: BLE001
            print(
                "Holosoma startup preparation failed: {0}. "
                "Set HIM_HOLOSOMA_ONNX to fastsac_g1_29dof.onnx.".format(exc),
                flush=True,
            )
            return False
        return True

    def _publish_pose_hold(self):
        """Command the pose captured at the stop, using profile gains and full ownership."""
        with self.stop_lock:
            now = self.clock()
            snapshot = self.pose_snapshot
            snapshot_secs = self.pose_snapshot_secs
            if (
                snapshot is None
                or snapshot_secs is None
                or not 0 <= now - snapshot_secs < HARDWARE_FEEDBACK_TIMEOUT_SECS
            ):
                return False
            if self.hold_targets is None:
                if not all(math.isfinite(q) for q in snapshot):
                    return False
                self.hold_targets = {
                    index: {
                        "q": q,
                        "dq": 0.0,
                        "kp": limits.kp,
                        "kd": limits.kd,
                        "tau": 0.0,
                    }
                    for index, q, limits in zip(
                        UPPER_BODY_INDICES, snapshot, self.profile.joints
                    )
                }
            targets = self.hold_targets
        try:
            written = bool(self.transport.write_arm(targets, 1.0))
        except Exception:  # noqa: BLE001
            written = False
        if written:
            self.last_written_ownership_weight = 1.0
        return written

    def _request_stock_damping(self):
        """Enter stock whole-body damping when the pose hold can no longer be published."""
        if self.release_pending:
            return
        if self.balance_released:
            try:
                restored = self.transport.restore_motion()
            except Exception:  # noqa: BLE001
                return
            if restored != 0:
                self.damping_rpc_code = restored
                return
            self.balance_released = False
        try:
            self.damping_rpc_code = self.transport.request_damping()
        except Exception:  # noqa: BLE001
            self.damping_rpc_code = None
        try:
            self.arm_release_written = bool(self.transport.write_arm({}, 0.0))
        except Exception:  # noqa: BLE001
            self.arm_release_written = False
        if self.damping_rpc_code != 0:
            return
        try:
            code, fsm = self.transport.read_fsm()
            self.reported_fsm_id = fsm if code == 0 else None
        except Exception:  # noqa: BLE001
            self.reported_fsm_id = None
        if (
            self.damping_rpc_code == 0
            and self.reported_fsm_id == DAMPING_FSM_ID
            and self.arm_release_written
            and (self.release_damp_when_confirmed or self.whole_body)
        ):
            self._release_after_confirmed_damp()

    def _release_after_confirmed_damp(self):
        """Stop resending damp so a later stand from the remote is left alone."""
        with self.stop_lock:
            self.soft_reset_active = False
            self.automatic_soft_reset = False
            self.hold_failed = False
            self.walk_stop = False
            self.armed = False
            self.ownership_taken = False
            self.command = None
            if not self.whole_body:
                self.abort_reason = None
            self.release_damp_when_confirmed = False
            self.damp_settled = True
            self.stop_requested.clear()
        self.button_detector.reset()
        self.f3_detector.reset()

    def start_workers(self):
        """Run output deadlines and stop RPC retries independently of ROS callbacks."""
        self._ensure_balance_policy()
        self._ensure_soft_reset_policy()
        self._ensure_startup_policy()
        self.trace = ControlTrace(default_trace_path())
        self.trace.open()
        sys.setswitchinterval(INTERPRETER_SWITCH_SECS)

        def output_loop():
            # Fixed deadlines like Holosoma's rate limiter. Sleeping a full
            # period after each step stretches 20 ms by the step time.
            next_tick = time.monotonic()
            while True:
                delay, next_tick = poll_deadline(
                    next_tick + HARDWARE_PERIOD_SECS, time.monotonic(), HARDWARE_PERIOD_SECS
                )
                if self.shutdown_requested.wait(delay):
                    return
                self.step()

        def body_repeat_loop():
            # Every whole-body policy uses this bus: GR00T, Holosoma,
            # and the container policy after left and B. Inference stays in
            # output_loop at 20 ms. A late cycle waits without spinning.
            next_tick = time.monotonic()
            while not self.shutdown_requested.is_set():
                self._repeat_body_once()
                now = time.monotonic()
                delay, next_tick = whole_body_repeat_wait(next_tick + BODY_REPEAT_SECS, now)
                if self.shutdown_requested.wait(delay):
                    return

        def stop_loop():
            last_request_secs = None
            while not self.shutdown_requested.wait(HARDWARE_WATCHDOG_POLL_SECS):
                now = self.clock()
                last_progress = self.last_worker_cycle_secs
                if last_progress is None:
                    last_progress = self.last_submission_secs
                if (
                    self.armed
                    and not self.release_pending
                    and last_progress is not None
                    and now - last_progress >= self.profile.max_publish_gap_secs
                ):
                    self.abort("hardware_output_worker_stalled")
                if not self.stop_requested.is_set():
                    continue
                with self.stop_lock:
                    hard = self.hold_failed
                    walk = self.walk_stop and not hard
                    holding = self.ownership_taken and not hard and not walk
                if hard:
                    interval = (
                        1.0
                        if self.damping_rpc_code == SDK_RPC_SEND_CODE
                        else DAMPING_RETRY_SECS
                    )
                elif walk:
                    # Recovery inference runs on the same 20 ms clock as startup.
                    continue
                elif holding:
                    interval = HARDWARE_PERIOD_SECS
                else:
                    continue
                if last_request_secs is None or now - last_request_secs >= interval:
                    self.stop_once()
                    last_request_secs = self.clock()

        self.workers = [
            Thread(target=output_loop, daemon=True),
            Thread(target=stop_loop, daemon=True),
        ]
        self.workers.append(Thread(target=body_repeat_loop, daemon=True))
        if self.whole_body:
            print(
                "Whole-body rt/lowcmd refreshes every 2 ms. "
                "Policy inference stays at 20 ms.",
                flush=True,
            )
        for worker in self.workers:
            worker.start()

    def _repeat_body_once(self):
        """Publish the latched whole-body command, or nothing once it is cleared."""
        transport = self.transport
        if transport is None or not (self.whole_body or self.soft_reset_active):
            return
        machine = self.machine_type
        if machine is None:
            return
        try:
            written = transport.repeat_body(machine)
        except Exception as error:  # noqa: BLE001
            self.abort(f"hardware_output_error:{type(error).__name__}: {error}")
            return
        if written and self.trace is not None:
            self.trace.note_repeat(time.monotonic())
        if written is False:
            self.abort("hardware_write_failed")

    def close(self):
        """Damp on exit only after this process has taken arm ownership."""
        self.abort("hardware_shutdown")
        with self.stop_lock:
            soft = self.walk_stop and not self.hold_failed and not self.balance_released
            exiting_a_hold = (self.ownership_taken and not soft) or self.balance_released
            if exiting_a_hold:
                self.hold_failed = True
        if exiting_a_hold:
            self._request_stock_damping()
        deadline = time.monotonic() + SHUTDOWN_WAIT_SECS
        while exiting_a_hold and time.monotonic() < deadline:
            if (
                self.damping_rpc_code == 0
                and self.reported_fsm_id == DAMPING_FSM_ID
                and self.arm_release_written
            ):
                break
            time.sleep(HARDWARE_PERIOD_SECS)
            self._request_stock_damping()
        self.shutdown_requested.set()
        for worker in self.workers:
            worker.join(timeout=SHUTDOWN_WAIT_SECS)
        if self.trace is not None:
            self.trace.close()
        # A timed-out worker may still be using DDS, so do not close beneath it.
        if not any(worker.is_alive() for worker in self.workers):
            self.transport.close()
