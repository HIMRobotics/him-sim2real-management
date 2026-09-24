"""Send safety-approved commands to the robot through the Unitree SDK."""

import math
import time
from copy import deepcopy
from threading import Event, Lock, RLock, Thread

from .config import Config
from .controller import StartDoublePressDetector
from .feedback import G1_29_JOINT_NAMES, feedback_from_sdk
from .hardware_profile import (
    DAMPING_FSM_ID,
    DAMPING_RETRY_SECS,
    HARDWARE_FEEDBACK_TIMEOUT_SECS,
    HARDWARE_PERIOD_SECS,
    HARDWARE_WATCHDOG_POLL_SECS,
    SHUTDOWN_WAIT_SECS,
    UPPER_BODY_INDICES,
)
from .joint_diagnostics import JointDeltaDiagnostics


class G1ArmAdapter:
    """Connect approved safety-checker commands to robot motors and request damping."""

    def __init__(self, profile, clock=time.monotonic):
        self.profile = profile
        self.clock = clock
        self.transport = None
        self.lock = RLock()
        self.stop_lock = Lock()
        self.feedback = None
        self.machine_type = None
        self.abort_reason = None
        self.armed = False
        self.time_ready_mode_confirmed_secs = None
        self.arm_rejection_reason = None
        self.command = None
        self.command_expires_secs = None
        self.last_submission_secs = None
        self.previous_targets = None
        self.previous_velocities = None
        self.last_written_ownership_weight = 0.0
        self.first_output_secs = None
        self.last_output_secs = None
        self.last_worker_cycle_secs = None
        self.stop_requested = Event()
        self.shutdown_requested = Event()
        self.damping_rpc_code = None
        self.reported_fsm_id = None
        self.arm_release_written = False
        self.joint_diagnostics = JointDeltaDiagnostics()
        self.workers = []
        self.button_detector = StartDoublePressDetector(
            Config().double_press_window_secs, HARDWARE_FEEDBACK_TIMEOUT_SECS
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
                self.feedback = sample
                if stop:
                    self.abort("hardware_controller_double_start", request_damping=True)
                if (
                    self.armed
                    and self.machine_type != self.profile.expected_machine_type
                ):
                    self.abort("hardware_machine_type_changed")
            except (AttributeError, TypeError, ValueError, OverflowError):
                self.abort("invalid_hardware_feedback")

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
            if fsm_id not in self.profile.allowed_start_fsm_ids:
                return self.reject_arm(
                    f"ready mode: measured={fsm_id}, "
                    f"allowed={self.profile.allowed_start_fsm_ids}"
                )
            self.time_ready_mode_confirmed_secs = self.clock()
            return True

    def arm(self):
        """Require a recent ready-mode check and fresh stationary joint feedback."""
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
            if self.feedback.start_button_pressed:
                return self.reject_arm("Start button must be released")
            for index, limits in zip(UPPER_BODY_INDICES, self.profile.joints):
                joint_name = G1_29_JOINT_NAMES[index]
                position = self.feedback.q[index]
                velocity = self.feedback.dq[index]
                if not limits.position_min_rad <= position <= limits.position_max_rad:
                    return self.reject_arm(
                        f"{joint_name} position: measured={position:.6f} rad, "
                        f"allowed=[{limits.position_min_rad}, {limits.position_max_rad}] rad"
                    )
                if abs(velocity) > self.profile.initial_velocity_tolerance_rad_secs:
                    return self.reject_arm(
                        f"{joint_name} speed: measured={velocity:.6f} rad/s, "
                        f"maximum absolute={self.profile.initial_velocity_tolerance_rad_secs} rad/s"
                    )
            # Serialize granting control with an abort arriving from another thread.
            with self.stop_lock:
                if self.abort_reason:
                    return self.reject_arm(f"already stopped: {self.abort_reason}")
                self.last_submission_secs = now
                self.armed = True
            return True

    def reject_arm(self, reason):
        """Record the failed readiness check without changing motor settings."""
        self.arm_rejection_reason = reason
        return False

    def abort(self, reason, request_damping=False):
        """Permanently prohibit arm writes and wake the independent damping RPC worker."""
        # Keep stop requests independent of a blocked SDK write.
        with self.stop_lock:
            if self.abort_reason is None:
                self.abort_reason = reason
            self.command = None
            if self.armed or request_damping:
                self.stop_requested.set()

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
        """Check freshness and target slew before one serialized arm SDK write."""
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
                self.joint_diagnostics.record(
                    self.command, self.feedback, self.previous_targets
                )
                targets = self.checked_targets(now)
                elapsed = (
                    0
                    if self.first_output_secs is None
                    else now - self.first_output_secs
                )
                weight_progress = min(elapsed / self.profile.handover_secs, 1.0)
                weight = weight_progress**2 * (3 - 2 * weight_progress)
                if self.stop_requested.is_set():
                    return
                if not self.transport.write_arm(targets, weight):
                    self.abort("hardware_write_failed")
                    return
                self.last_written_ownership_weight = weight
                if self.first_output_secs is None:
                    self.first_output_secs = now
                self.last_output_secs = now
                self.previous_targets = {
                    index: target["q"] for index, target in targets.items()
                }
                self.previous_velocities = {
                    index: target["dq"] for index, target in targets.items()
                }
            except Exception as error:  # noqa: BLE001
                self.abort(f"hardware_output_error:{type(error).__name__}: {error}")

    def joint_diagnostics_snapshot(self):
        """Skip a busy output lock so diagnostics cannot stall ROS stop handling."""
        if not self.lock.acquire(blocking=False):
            return None
        try:
            return self.joint_diagnostics.snapshot()
        finally:
            self.lock.release()

    def checked_targets(self, now):
        """Use configured gains, zero feedforward torque and bounded position/velocity."""
        targets = {}
        dt = 0 if self.last_output_secs is None else now - self.last_output_secs
        for index, limits in zip(UPPER_BODY_INDICES, self.profile.joints):
            q = self.command["q"][index]
            dq = self.command["dq"][index]
            tau = self.command["tau"][index]
            joint = G1_29_JOINT_NAMES[index]
            measured = self.feedback.q[index]
            tracking_error = abs(q - measured)
            if not all(math.isfinite(value) for value in (q, dq, tau)):
                raise ValueError(f"{joint}: nonfinite target q={q}, dq={dq}, tau={tau}")
            if not limits.position_min_rad <= q <= limits.position_max_rad:
                raise ValueError(
                    f"{joint}: target q={q:.6f} rad outside "
                    f"[{limits.position_min_rad}, {limits.position_max_rad}]"
                )
            if abs(dq) > limits.max_velocity_rad_secs:
                raise ValueError(
                    f"{joint}: target dq={dq:.6f} rad/s exceeds "
                    f"absolute limit={limits.max_velocity_rad_secs}"
                )
            if tau != 0:
                raise ValueError(f"{joint}: feedforward tau={tau} Nm must be zero")
            if tracking_error > limits.max_tracking_error_rad:
                raise ValueError(
                    f"{joint}: tracking error={tracking_error:.6f} rad exceeds "
                    f"limit={limits.max_tracking_error_rad}, "
                    f"target={q:.6f}, measured={measured:.6f}"
                )
            if self.previous_targets is None:
                if tracking_error > self.profile.initial_position_tolerance_rad:
                    raise ValueError(
                        f"{joint}: initial position error={tracking_error:.6f} rad "
                        f"exceeds limit={self.profile.initial_position_tolerance_rad}"
                    )
                if abs(dq) > self.profile.initial_velocity_tolerance_rad_secs:
                    raise ValueError(
                        f"{joint}: initial target dq={dq:.6f} rad/s exceeds "
                        f"absolute limit={self.profile.initial_velocity_tolerance_rad_secs}"
                    )
                # First SDK write uses exactly the measured position with zero weight.
                q, dq = measured, 0.0
            else:
                if dt <= 0:
                    raise ValueError(f"output time did not advance: interval={dt} secs")
                position_change = abs(q - self.previous_targets[index])
                if position_change > limits.max_velocity_rad_secs * dt:
                    raise ValueError(
                        f"{joint}: target position change={position_change:.6f} rad "
                        f"exceeds limit={limits.max_velocity_rad_secs * dt:.6f} rad "
                        f"over interval={dt:.6f} secs"
                    )
                velocity_change = abs(dq - self.previous_velocities[index])
                if velocity_change > limits.max_acceleration_rad_secs2 * dt:
                    raise ValueError(
                        f"{joint}: target velocity change={velocity_change:.6f} rad/s "
                        f"exceeds limit={limits.max_acceleration_rad_secs2 * dt:.6f} "
                        f"over interval={dt:.6f} secs"
                    )
            targets[index] = {
                "q": q,
                "dq": dq,
                "kp": limits.kp,
                "kd": limits.kd,
                "tau": 0.0,
            }
        return targets

    def stop_once(self):
        """Request full-body damping, then disable arm ownership and inspect stock FSM."""
        # RPC calls never hold the arm-write lock, so a timeout cannot delay latching.
        try:
            self.damping_rpc_code = self.transport.request_damping()
        except Exception:  # noqa: BLE001
            self.damping_rpc_code = None
        try:
            # Empty motor targets and zero ownership weight are a release, not damping.
            self.arm_release_written = bool(self.transport.write_arm({}, 0.0))
        except Exception:  # noqa: BLE001
            self.arm_release_written = False
        try:
            code, fsm = self.transport.read_fsm()
            self.reported_fsm_id = fsm if code == 0 else None
        except Exception:  # noqa: BLE001
            self.reported_fsm_id = None

    def start_workers(self):
        """Run output deadlines and stop RPC retries independently of ROS callbacks."""

        def output_loop():
            while not self.shutdown_requested.wait(HARDWARE_PERIOD_SECS):
                self.step()

        def stop_loop():
            last_request_secs = None
            while not self.shutdown_requested.wait(HARDWARE_WATCHDOG_POLL_SECS):
                now = self.clock()
                last_progress = self.last_worker_cycle_secs
                if last_progress is None:
                    last_progress = self.last_submission_secs
                if (
                    self.armed
                    and last_progress is not None
                    and now - last_progress >= self.profile.max_publish_gap_secs
                ):
                    self.abort("hardware_output_worker_stalled")
                if self.stop_requested.is_set() and (
                    last_request_secs is None
                    or now - last_request_secs >= DAMPING_RETRY_SECS
                ):
                    self.stop_once()
                    last_request_secs = self.clock()

        self.workers = [
            Thread(target=output_loop, daemon=True),
            Thread(target=stop_loop, daemon=True),
        ]
        for worker in self.workers:
            worker.start()

    def close(self):
        """Attempt damping on orderly exit, without claiming protection against SIGKILL."""
        self.abort("hardware_shutdown")
        deadline = time.monotonic() + SHUTDOWN_WAIT_SECS
        while self.stop_requested.is_set() and time.monotonic() < deadline:
            if (
                self.damping_rpc_code == 0
                and self.reported_fsm_id == DAMPING_FSM_ID
                and self.arm_release_written
            ):
                break
            time.sleep(HARDWARE_PERIOD_SECS)
        self.shutdown_requested.set()
        for worker in self.workers:
            worker.join(timeout=SHUTDOWN_WAIT_SECS)
        # A timed-out worker may still be using DDS, so do not close beneath it.
        if not any(worker.is_alive() for worker in self.workers):
            self.transport.close()
