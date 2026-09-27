"""Opt-in motor gate that writes Unitree upper-body targets and requests body damping."""

import json
import os
import socket
import time

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
from .ros_nodes import MotorGateNode
from .unitree_transport import UnitreeArmTransport


class HardwareMotorGateNode(MotorGateNode):
    """Send approved targets using explicitly supplied robot settings."""

    def __init__(self):
        self.adapter = None
        super().__init__()
        enabled = self.declare_parameter("enable_hardware_output", False).value
        profile_path = self.declare_parameter("hardware_profile_path", "").value
        interface = self.declare_parameter("sdk_interface", "").value
        domain = self.declare_parameter("sdk_domain", 0).value
        if enabled is not True or not profile_path or not interface:
            raise ValueError(
                "hardware output requires explicit enable, profile path and SDK interface"
            )
        if self.config.joints != NUM_JOINTS:
            raise ValueError(
                "hardware adapter requires the identified 29-joint configuration"
            )
        profile = HardwareProfile.load(profile_path)
        self.status_publisher = self.create_publisher(
            String, "/safety/hardware_status", self.command_qos
        )
        self.joint_diagnostics_publisher = self.create_publisher(
            String, topics.JOINT_DIAGNOSTICS, self.command_qos
        )
        self.final_diagnostics_logged = False
        self.create_timer(JOINT_DIAGNOSTICS_PERIOD_SECS, self.publish_joint_diagnostics)
        self.last_status = None
        self.startup_ready = False
        self.adapter = G1ArmAdapter(profile)
        self.adapter.transport = UnitreeArmTransport(
            interface, domain, self.adapter.receive_state
        )
        self.adapter.start_workers()
        self.get_logger().warn(
            "Hardware output enabled. Arming may command the robot. No automatic rearm."
        )

    def notify_startup_ready(self):
        """Report startup readiness only after warmup and fresh released buttons."""
        adapter = self.adapter
        if (
            self.startup_ready or adapter.balance_policy is None
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
        )
        response.message = (
            "Full ownership weight sent" if response.success else "handover incomplete"
        )
        return response

    def on_ready(self, _request, response):
        """Query stock mode before the checker starts its command watchdog."""
        if (
            self.adapter is None
            or self.motor_gate.abort_reason
            or self.motor_gate.policy_session_id is not None
        ):
            response.success = False
            response.message = "Hardware gate is unavailable or stopped"
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
        else:
            detail = self.adapter.arm_rejection_reason
            self.get_logger().error(f"Hardware readiness rejected: {detail}")
            self.adapter.abort("hardware_readiness_rejected")
            self.motor_gate.abort("hardware_readiness_rejected")
            self.latch_abort("hardware_readiness_rejected")
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
            self.get_logger().error(f"Hardware arming rejected: {detail}")
            self.adapter.abort("hardware_arm_rejected")
            self.motor_gate.abort("hardware_arm_rejected")
            self.latch_abort("hardware_arm_rejected")
            response.success = False
            response.message = f"Hardware arming rejected: {detail}"
            return response
        return super().on_arm(request, response)

    def publish_output(self):
        """Forward only running policy output, retaining the gate's original expiry."""
        if self.adapter is None:
            return
        self.notify_startup_ready()
        if self.adapter.damp_settled:
            self.motor_gate.abort_reason = None
            self.abort_reason = None
            self.adapter.damp_settled = False
        elif self.adapter.abort_reason:
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
                "hardware_output_enabled": True,
                "armed": self.adapter.armed,
                "handover_complete": self.adapter.last_written_ownership_weight == 1.0,
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
        if status != self.last_status:
            message = String()
            message.data = status
            self.status_publisher.publish(message)
            self.get_logger().info(status)
            self.last_status = status

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
            if values["tracking_error_rad"]["samples"]
        }
        if not sampled:
            return
        payload = {
            "session_id": self.motor_gate.policy_session_id,
            "abort_reason": self.adapter.abort_reason,
            "scope": "targets_checked_before_sdk_write",
            "joints": joints,
        }
        message = String()
        message.data = json.dumps(payload)
        self.joint_diagnostics_publisher.publish(message)
        if final:
            for name, values in sampled.items():
                self.get_logger().info(f"Joint extrema {name}: {json.dumps(values)}")
            self.final_diagnostics_logged = True
        else:
            name = max(
                sampled, key=lambda key: sampled[key]["tracking_error_rad"]["max_abs"]
            )
            error = sampled[name]["tracking_error_rad"]
            self.get_logger().info(
                f"Run tracking extrema: largest={name}, "
                f"min={error['min']:.6f}, max={error['max']:.6f} rad. "
                f"All joints: {topics.JOINT_DIAGNOSTICS}"
            )

    def destroy_node(self):
        if self.adapter is not None and self.adapter.transport is not None:
            self.publish_joint_diagnostics(final=True)
            self.adapter.close()
        return super().destroy_node()
