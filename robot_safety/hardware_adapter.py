"""Send safety-approved commands to the robot through the Unitree SDK."""

import math
import time
from copy import deepcopy
from threading import Event, Lock, RLock, Thread

from .balance_policy import LEG_KD, LEG_KP, MODEL_PATH, BalancePolicy
from .config import Config
from .controller import StartDoublePressDetector, decode_f3_pressed
from .feedback import feedback_from_sdk
from .hardware_constants import (
    ARM_INDICES,
    DAMPING_FSM_ID,
    G1_29_JOINT_NAMES,
    SDK_RPC_SEND_CODE,
    UPPER_BODY_INDICES,
)
from .hardware_profile import (
    DAMPING_RETRY_SECS,
    HARDWARE_FEEDBACK_TIMEOUT_SECS,
    HARDWARE_PERIOD_SECS,
    HARDWARE_WATCHDOG_POLL_SECS,
    SHUTDOWN_WAIT_SECS,
)
from .joint_checks import check_command, check_torque
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
        self._reset_session()
        self.last_submission_secs = None
        self.stop_requested = Event()
        self.shutdown_requested = Event()
        self.damping_rpc_code = None
        self.walk_rpc_code = None
        self.walk_stop = False
        self.balance_policy = None
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
                self.feedback = sample
                imu = getattr(message, "imu_state", None)
                if imu is not None:
                    self.quat = tuple(float(value) for value in imu.quaternion)
                    self.gyro = tuple(float(value) for value in imu.gyroscope)
                with self.stop_lock:
                    self.pose_snapshot = tuple(
                        sample.q[index] for index in UPPER_BODY_INDICES
                    )
                    self.pose_snapshot_secs = sample.received_at_secs
                    if stop:
                        self.hold_failed = True
                        self.walk_stop = False
                        self.release_damp_when_confirmed = True
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
            if self.feedback.start_button_pressed or self.f3_pressed:
                return self.reject_arm("Start and F3 must be released")
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
                self._reset_session()
                self.last_submission_secs = now
                self.armed = True
            return True

    def _reset_session(self):
        """Initialize output history before workers start or under both adapter locks."""
        self.command = None
        self.command_expires_secs = None
        self.previous_targets = self.previous_velocities = None
        self.first_output_secs = self.last_output_secs = None
        self.last_worker_cycle_secs = None
        self.last_written_ownership_weight = 0.0
        self.ownership_taken = False
        self.hold_targets = self.arm_hold_q = None
        self.joint_diagnostics = JointDeltaDiagnostics()

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
            if self.armed or request_damping or self.walk_stop or self.hold_failed:
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
                if weight > 0:
                    with self.stop_lock:
                        self.ownership_taken = True
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
            previous = None if self.previous_targets is None else (
                self.previous_targets[index], self.previous_velocities[index])
            check_command(joint, limits, q, dq, tau, measured, previous, dt,
                          self.profile.initial_position_tolerance_rad,
                          self.profile.initial_velocity_tolerance_rad_secs)
            check_torque(joint, limits, limits.kp * (q - measured)
                         + limits.kd * (dq - self.feedback.dq[index]) + tau)
            if previous is None:
                q, dq = measured, 0.0
            targets[index] = {
                "q": q,
                "dq": dq,
                "kp": limits.kp,
                "kd": limits.kd,
                "tau": 0.0,
            }
        return targets

    def stop_once(self):
        """Damp on double Start, balance on double F3, otherwise hold the arms."""
        # SDK calls stay off the arm-write lock so a stuck motion write cannot delay the stop.
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
        """Hold the GR00T standing network on the legs. Arms stay at the captured pose."""
        if self.balance_policy is None:
            return
        if not self.balance_released:
            try:
                self.walk_rpc_code = self.transport.release_motion()
            except Exception:  # noqa: BLE001
                self.walk_rpc_code = None
                return
            if self.walk_rpc_code != 0:
                return
            self.balance_released = True
        if self.feedback is None:
            return
        feedback = self.feedback
        q = feedback.q
        if self.arm_hold_q is None:
            self.arm_hold_q = tuple(q[index] for index in ARM_INDICES)
        try:
            lower = self.balance_policy.update(q, feedback.dq, self.quat, self.gyro)
            now = self.clock()
            if not 0 <= now - feedback.received_at_secs < HARDWARE_FEEDBACK_TIMEOUT_SECS:
                raise ValueError("stale balance feedback")
            # Soft-reset balancing does not use submission movement limits.
            if not self.transport.write_lower_body(lower, self.arm_hold_q, LEG_KP, LEG_KD, self.machine_type or 0):
                raise RuntimeError("balance write failed")
        except Exception as exc:  # noqa: BLE001 - Balance failures enter damping.
            self.abort_reason = "balance_output_error: " + str(exc)
            self.hold_failed = True
            self._request_stock_damping()

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
                "F3 balancing unavailable until restart.",
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
            self.release_damp_when_confirmed
            and self.damping_rpc_code == 0
            and self.reported_fsm_id == DAMPING_FSM_ID
            and self.arm_release_written
        ):
            self._release_after_confirmed_damp()

    def _release_after_confirmed_damp(self):
        """Stop resending damp so a later stand from the remote is left alone."""
        with self.stop_lock:
            self.arm_hold_q = None
            self.hold_failed = False
            self.walk_stop = False
            self.armed = False
            self.command = None
            self.abort_reason = None
            self.release_damp_when_confirmed = False
            self.damp_settled = True
            self.stop_requested.clear()
        self.button_detector.reset()
        self.f3_detector.reset()

    def start_workers(self):
        """Run output deadlines and stop RPC retries independently of ROS callbacks."""
        self._ensure_balance_policy()

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
                elif walk or holding:
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
        for worker in self.workers:
            worker.start()

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
        # A timed-out worker may still be using DDS, so do not close beneath it.
        if not any(worker.is_alive() for worker in self.workers):
            self.transport.close()
