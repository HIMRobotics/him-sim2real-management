"""Opt-in motor gate for checked Unitree joint commands and stock damping."""

import json
import os
import socket
import time

from him_safety_interfaces.msg import Abort
from std_msgs.msg import String

from . import ros_topics as topics
from .hardware_adapter import G1ArmAdapter
from .hardware_constants import NUM_JOINTS
from .hardware_profile import (
    JOINT_DIAGNOSTICS_PERIOD_SECS,
    READINESS_QUERY_ATTEMPTS,
    READINESS_RPC_TIMEOUT_SECS,
    HardwareProfile,
)
from .motor_gate import MotorCommandGate
from .ros_nodes import MotorGateNode
from .unitree_transport import UnitreeArmTransport


def _as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes")
    return bool(value)


class HardwareMotorGateNode(MotorGateNode):
    """Send approved targets using explicitly supplied robot settings."""

    def __init__(self):
        self.adapter = None
        self.session_open = False
        super().__init__()
        enabled = self.declare_parameter("enable_hardware_output", False).value
        dry_run = self.declare_parameter("dry_run", False).value
        whole_body = self.declare_parameter("whole_body_output", False).value
        launch_gate = self.declare_parameter("launch_gate", False).value
        stance_blend_secs = self.declare_parameter("stance_blend_secs", 5.0).value
        profile_path = self.declare_parameter("hardware_profile_path", "").value
        interface = self.declare_parameter("sdk_interface", "").value
        domain = self.declare_parameter("sdk_domain", 0).value
        if enabled == dry_run or not profile_path or not interface:
            raise ValueError(
                "hardware output requires explicit enable, profile path and SDK interface"
            )
        if self.config.joints != NUM_JOINTS:
            raise ValueError(
                "hardware adapter requires the identified 29-joint configuration"
            )
        profile = HardwareProfile.load(profile_path)
        self.hardware_profile_path = profile_path
        self.status_publisher = self.create_publisher(
            String, "/safety/hardware_status", self.command_qos
        )
        self.joint_diagnostics_publisher = self.create_publisher(
            String, topics.JOINT_DIAGNOSTICS, self.command_qos
        )
        self.final_diagnostics_logged = False
        self.create_timer(JOINT_DIAGNOSTICS_PERIOD_SECS, self.publish_joint_diagnostics)
        self.last_status = None
        self.last_status_publish_secs = None
        self.startup_ready = False
        self.adapter = G1ArmAdapter(
            profile, whole_body=whole_body,
            launch_gate=_as_bool(launch_gate),
            stance_blend_secs=float(stance_blend_secs),
        )
        self.adapter.transport = UnitreeArmTransport(
            interface, domain, self.adapter.receive_state, self.adapter.receive_battery,
            output_enabled=enabled,
        )
        self.output_enabled = enabled
        self.adapter.start_workers()
        self.get_logger().warn(
            f"Hardware output enabled: {enabled}. No automatic rearm."
        )

    def notify_startup_ready(self):
        """Report startup readiness only after warmup and fresh released buttons."""
        adapter = self.adapter
        if (
            self.startup_ready or adapter.balance_policy is None
            or adapter.soft_reset_policy is None or adapter.startup_policy is None
            or not adapter.feedback_fresh(time.monotonic())
            or adapter.feedback.start_button_pressed or adapter.f3_pressed
            or adapter.abort_reason or self.motor_gate.abort_reason
            or not adapter.workers or not all(worker.is_alive() for worker in adapter.workers)
        ):
            return
        address = os.environ.get("NOTIFY_SOCKET")
        if address:
            if address.startswith("@"):
                address = "\0" + address[1:]
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as notifier:
                notifier.sendto(b"READY=1\nSTATUS=Policy warmed, workers running, telemetry fresh", address)
        self.startup_ready = True
        self.get_logger().info("Safety monitor ready: policy warmed, workers running, telemetry fresh")

    def on_handover_ready(self, request, response):
        """Confirm a full-weight SDK write for this session, not physical takeover."""
        response = super().on_handover_ready(request, response)
        response.success = (
            response.success
            and self.adapter is not None
            and self.adapter.abort_reason is None
            and self.adapter.last_written_ownership_weight == 1.0
            and (not self.adapter.whole_body or self.adapter.policy_launched)
        )
        response.message = (
            "Policy handoff authorized" if response.success else "handover incomplete"
        )
        return response

    def _accept_new_session(self):
        """Drop the previous run's stop. Ready mode is checked again after this."""
        adapter = self.adapter
        if adapter is None:
            return False
        if (
            adapter.armed
            or adapter.ownership_taken
            or adapter.balance_released
            or adapter.stop_requested.is_set()
        ):
            return False
        with adapter.stop_lock:
            adapter.abort_reason = None
            adapter.arm_rejection_reason = None
            adapter.hold_failed = False
            adapter.walk_stop = False
            adapter.command = None
            adapter.stop_requested.clear()
        if hasattr(self, "host_boot_id"):
            self.motor_gate = MotorCommandGate(self.host_boot_id, self.config)
            self.abort_reason = None
            self.final_diagnostics_logged = False
            if getattr(self, "abort_publisher", None) is not None:
                self.destroy_publisher(self.abort_publisher)
                self.abort_publisher = self.create_publisher(
                    Abort, topics.ABORT, self.abort_qos
                )
        else:
            self.motor_gate.abort_reason = None
            self.motor_gate.policy_session_id = None
        self.session_open = True
        return True

    def on_ready(self, _request, response):
        """Query stock mode before the checker starts its command watchdog."""
        if self.adapter is None or not HardwareMotorGateNode._accept_new_session(self):
            response.success = False
            response.message = "Hardware gate is unavailable or still stopping"
            return response
        query_started_secs = time.monotonic()
        self.get_logger().info(
            f"Reading stock ready mode: up to {READINESS_QUERY_ATTEMPTS} attempts, "
            f"{READINESS_RPC_TIMEOUT_SECS:.1f}s SDK timeout each"
        )
        try:
            response.success = self.adapter.check_ready_mode()
        except Exception as error:  # noqa: BLE001
            self.adapter.arm_rejection_reason = (
                f"readiness query raised {type(error).__name__}: {error}"
            )
            response.success = False
        results = getattr(self.adapter.transport, "readiness_query_results", None)
        if results:
            self.get_logger().info(f"Ready-mode query attempts: {results}")
        elapsed_secs = time.monotonic() - query_started_secs
        self.get_logger().info(
            f"Ready-mode query completed in {elapsed_secs:.3f}s, success={response.success}"
        )
        if response.success:
            response.message = "ready"
            return response
        detail = self.adapter.arm_rejection_reason or "unspecified readiness failure"
        response.success = False
        response.message = f"Hardware readiness rejected: {detail}"
        return response

    def on_arm(self, request, response):
        """Reject missing sessions or an unready robot before authorizing SDK output."""
        if (
            self.motor_gate.policy_session_id is not None
            or self.motor_gate.abort_reason
            or not request.session_id
        ):
            response.success = False
            response.message = "Gate is latched, already armed or session is missing"
            return response
        try:
            ready = self.adapter.arm()
        except Exception as error:  # noqa: BLE001
            self.adapter.arm_rejection_reason = (
                f"readiness check raised {type(error).__name__}: {error}"
            )
            ready = False
        if not ready:
            detail = (
                self.adapter.arm_rejection_reason or "unspecified readiness failure"
            )
            return self._reject_request(
                response, "hardware_arm_rejected", f"Hardware arming rejected: {detail}"
            )
        return super().on_arm(request, response)

    def _reject_request(self, response, reason, message):
        """Latch a failed hardware check across the adapter, gate and ROS publisher."""
        self.get_logger().error(message)
        self.adapter.abort(reason)
        self.motor_gate.abort(reason)
        self.latch_abort(reason)
        response.success = False
        response.message = message
        return response

    def on_abort(self, message):
        # A restarted gate is created while the previous writer may still be
        # delivering its last abort. That sample is the previous run.
        if message.source != self.get_name() and not self.session_open:
            self.get_logger().warning(
                "Ignoring abort from before this session: " + (message.reason or "")
            )
            return
        if message.source != self.get_name():
            self.adapter.abort(message.reason or "external_abort")
        super().on_abort(message)

    def publish_output(self):
        """Forward only running policy output, retaining the gate's original expiry."""
        if self.adapter is None:
            return
        if self.adapter.trace is not None:
            self.adapter.trace.note_gate(time.monotonic())
        self.notify_startup_ready()
        if self.adapter.damp_settled:
            with self.adapter.stop_lock:
                self.adapter.damp_settled = False
                if not self.adapter.abort_reason and not self.adapter.stop_requested.is_set():
                    self.motor_gate = MotorCommandGate(self.host_boot_id, self.config)
                    self.abort_reason = None
                    # Recreate the writer to discard its retained stop for future tests.
                    self.destroy_publisher(self.abort_publisher)
                    self.abort_publisher = self.create_publisher(Abort, topics.ABORT, self.abort_qos)
                    self.final_diagnostics_logged = False
        if self.adapter.abort_reason:
            self.motor_gate.abort(self.adapter.abort_reason)
        command = self.motor_gate.cycle(time.monotonic())
        if self.motor_gate.abort_reason:
            self.adapter.abort(self.motor_gate.abort_reason)
            self.latch_abort(self.motor_gate.abort_reason)
        elif command["kind"] == "policy":
            self.adapter.submit(command, self.motor_gate.time_command_expires_secs)
        if self.adapter.abort_reason and not self.final_diagnostics_logged:
            self.publish_joint_diagnostics(final=True)
        # RPC acceptance and stock FSM are evidence, not proof of physical damping.
        status = json.dumps(
            {
                **(self.adapter.startup.status() if self.adapter.whole_body else {}),
                "hardware_output_enabled": self.output_enabled,
                "whole_body_output": self.adapter.whole_body,
                "startup_policy": self.adapter.startup_policy_name,
                "soft_reset_policy": "holosoma",
                "hardware_profile_path": self.hardware_profile_path,
                "motion_released": self.adapter.balance_released,
                "motion_released_at_secs": self.adapter.motion_released_at_secs,
                "armed": self.adapter.armed,
                "handover_complete": self.adapter.last_written_ownership_weight == 1.0,
                "launch_gate": self.adapter.launch_gate,
                "policy_launched": self.adapter.policy_launched,
                "left_b_count": self.adapter.left_b_count,
                "left_b_at_secs": self.adapter.left_b_at_secs,
                "arm_rejection_reason": self.adapter.arm_rejection_reason,
                "abort_reason": self.adapter.abort_reason,
                "damping_rpc_code": self.adapter.damping_rpc_code,
                "walk_stop": self.adapter.walk_stop,
                "walk_rpc_code": self.adapter.walk_rpc_code,
                "reported_fsm_id": self.adapter.reported_fsm_id,
                "arm_release_written": self.adapter.arm_release_written,
                "pose_hold": self.adapter.hold_targets is not None
                and not self.adapter.hold_failed,
                "physical_damping_verified": False,
            }
        )
        now = time.monotonic()
        changed = status != self.last_status
        due = (
            self.last_status_publish_secs is None
            or now - self.last_status_publish_secs >= 0.5
        )
        if changed or due:
            message = String()
            message.data = status
            self.status_publisher.publish(message)
            if changed:
                self.get_logger().info(status)
            self.last_status = status
            self.last_status_publish_secs = now

    def publish_joint_diagnostics(self, final=False):
        """Publish run extrema once a second, and log every joint once on abort."""
        if self.adapter is None or self.final_diagnostics_logged:
            return
        joints = self.adapter.joint_diagnostics_snapshot()
        if joints is None:
            return
        sampled = {
            name: values
            for name, values in joints.items()
            if values["command_gap_rad"]["samples"]
            or values["response_error_rad"]["samples"]
        }
        if not sampled:
            return
        payload = {
            "session_id": self.motor_gate.policy_session_id,
            "abort_reason": self.adapter.abort_reason,
            "scope": "clipped_command_gaps_and_responses_to_prior_full_weight_writes",
            "joints": joints,
        }
        message = String()
        message.data = json.dumps(payload)
        self.joint_diagnostics_publisher.publish(message)
        self.get_logger().info(f"Joint diagnostics: {message.data}")
        for name, values in sampled.items():
            clipping = values.get("torque_target_clipping", {})
            if clipping.get("active"):
                self.get_logger().warning(
                    f"Torque target limited {name}: {json.dumps(clipping)}"
                )
            if "position_clipping" in values:
                self.get_logger().warning(
                    f"Joint position clipped {name}: {json.dumps(values['position_clipping'])}"
                )
        if final:
            self.final_diagnostics_logged = True

    def destroy_node(self):
        if self.adapter is not None and self.adapter.transport is not None:
            self.publish_joint_diagnostics(final=True)
            self.adapter.close()
        return super().destroy_node()
