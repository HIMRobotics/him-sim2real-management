"""ROS 2 command checking, controller-stop detection and motor-gate roles."""

import argparse
import json
import math
import signal
import time
from dataclasses import fields, replace
from pathlib import Path

import rclpy
from him_safety_interfaces.msg import (
    Abort,
    CheckedCommand,
    DryRunOutput,
    JointCommand,
    RobotFeedback,
)
from him_safety_interfaces.srv import ArmGate
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from . import ros_topics as topics
from .config import COMMAND_FIELDS, Config, ThrashConfig
from .controller import StartDoublePressDetector
from .core import Supervisor, SupervisorPhase
from .motor_gate import MotorCommandGate
from .thrash import default_weights


def command_from_ros(message):
    """Convert a typed command to the transport-independent core representation."""
    result = {name: list(getattr(message, name)) for name in COMMAND_FIELDS}
    result.update(
        session=message.session_id,
        seq=message.sequence,
        state_id=message.observation_id,
    )
    return result


def command_to_ros(command):
    """Convert selected command data into a typed diagnostic message."""
    message = JointCommand()
    for name in COMMAND_FIELDS:
        setattr(message, name, command[name])
    message.session_id = command.get("session", "")
    message.sequence = command.get("seq", 0)
    message.observation_id = command.get("state_id", 0)
    return message


class SafetyNode(Node):
    """Shared topic QoS and abort broadcasting, not a common execution loop."""

    def __init__(self, name):
        super().__init__(name)
        joints = self.declare_parameter("joints", Config().joints).value
        self.config = Config(joints=joints)
        self.host_boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.feedback_qos = QoSProfile(
            depth=self.config.runtime_queue_capacity,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.abort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.abort_publisher = self.create_publisher(
            Abort, topics.ABORT, self.abort_qos
        )
        self.abort_reason = None
        self.create_timer(topics.ABORT_REPEAT_SECS, self.repeat_abort)

    def latch_abort(self, reason):
        """Publish immediately on first fault and periodically thereafter."""
        if self.abort_reason is None:
            self.abort_reason = reason[:128]
            self.get_logger().error(f"Abort latched: {self.abort_reason}")
            self.repeat_abort()

    def repeat_abort(self):
        if self.abort_reason is not None:
            message = Abort()
            message.source = self.get_name()
            message.reason = self.abort_reason
            self.abort_publisher.publish(message)

    def feedback_is_fresh(self, message, cur_time_secs):
        """Reject stale feedback or timestamps from a different host boot."""
        return (
            message.host_boot_id == self.host_boot_id
            and math.isfinite(message.received_at_secs)
            and 0
            <= cur_time_secs - message.received_at_secs
            < self.config.robot_timeout
        )


class SafetyCheckerNode(SafetyNode):
    """Relay each accepted precheck frame once, or latch and broadcast abort."""

    def __init__(self):
        super().__init__("safety_checker")
        settings = {}
        for field in fields(ThrashConfig):
            default = field.default
            if field.name == "velocity_scales":
                default = [1.0] * self.config.joints
            elif field.name == "joint_weights":
                default = list(default_weights(self.config.joints))
            value = self.declare_parameter(f"thrash.{field.name}", default).value
            settings[field.name] = tuple(value) if isinstance(default, list) else value
        self.config = replace(self.config, thrash=ThrashConfig(**settings))
        self.supervisor = Supervisor(self.config)
        self.thrash_event_logged = False
        self.postcheck_publisher = self.create_publisher(
            CheckedCommand, topics.POSTCHECK_COMMAND, self.command_qos
        )
        self.policy_observation_publisher = self.create_publisher(
            RobotFeedback, topics.POLICY_OBSERVATION, self.feedback_qos
        )
        self.create_subscription(
            RobotFeedback, topics.ROBOT_FEEDBACK, self.on_feedback, self.feedback_qos
        )
        self.create_subscription(
            JointCommand, topics.PRECHECK_COMMAND, self.on_precheck, self.command_qos
        )
        self.create_subscription(Abort, topics.ABORT, self.on_abort, self.abort_qos)
        self.create_service(Trigger, topics.CHECKER_ARM, self.on_arm)
        self.create_timer(self.config.period, self.on_timer)

    def on_feedback(self, message):
        cur_time_secs = time.monotonic()
        if not self.feedback_is_fresh(message, cur_time_secs):
            self.supervisor.input_fault("invalid_feedback_clock_or_age", cur_time_secs)
        else:
            self.supervisor.state(
                {"q": list(message.q), "dq": list(message.dq)},
                message.start_button_pressed,
                cur_time_secs,
                received_time_secs=message.received_at_secs,
            )
            if self.supervisor.supervisor_phase != SupervisorPhase.STOP_LATCHED:
                message.observation_id = self.supervisor.latest_observation_id
                self.policy_observation_publisher.publish(message)
        self.check_fault()

    def on_precheck(self, message):
        cur_time_secs = time.monotonic()
        if self.abort_reason is not None:
            return
        if self.supervisor.accept(command_from_ros(message), cur_time_secs):
            checked = CheckedCommand()
            checked.command = message
            checked.host_boot_id = self.host_boot_id
            checked.checked_at_secs = cur_time_secs
            observation_received_at_secs = (
                self.supervisor.observation_received_times_secs[message.observation_id]
            )
            checked.expires_at_secs = min(
                cur_time_secs + self.config.action_timeout,
                observation_received_at_secs + self.config.observation_timeout,
            )
            self.postcheck_publisher.publish(checked)
        self.check_fault()

    def on_abort(self, message):
        if message.source != self.get_name():
            self.supervisor.stop(message.reason or "external_abort", time.monotonic())
            self.check_fault()

    def on_arm(self, _request, response):
        result = self.supervisor.control("arm", time.monotonic())
        response.success = "error" not in result
        response.message = result.get("error", result.get("session") or "")
        self.check_fault()
        return response

    def on_timer(self):
        # Never republish cached commands to postcheck as a heartbeat.
        self.supervisor.cycle(time.monotonic())
        self.check_fault()

    def check_fault(self):
        if self.supervisor.thrash_event is None:
            self.thrash_event_logged = False
        elif not self.thrash_event_logged:
            self.get_logger().warning(
                f"Thrash detection ({self.config.thrash.mode}): "
                f"{json.dumps(self.supervisor.thrash_event)}"
            )
            self.thrash_event_logged = True
        if self.supervisor.supervisor_phase == SupervisorPhase.STOP_LATCHED:
            self.latch_abort(self.supervisor.stop_reason)


class MotorGateNode(SafetyNode):
    """Independently watchdog postcheck and publish only diagnostic output."""

    def __init__(self):
        super().__init__("motor_command_gate")
        self.motor_gate = MotorCommandGate(self.host_boot_id, self.config)
        self.dry_run_output_publisher = self.create_publisher(
            DryRunOutput, topics.DRY_RUN_OUTPUT, self.command_qos
        )
        self.create_subscription(
            CheckedCommand,
            topics.POSTCHECK_COMMAND,
            self.on_postcheck,
            self.command_qos,
        )
        self.create_subscription(Abort, topics.ABORT, self.on_abort, self.abort_qos)
        self.create_service(Trigger, topics.GATE_READY, self.on_ready)
        self.create_service(ArmGate, topics.GATE_ARM, self.on_arm)
        self.create_service(ArmGate, topics.HANDOVER_READY, self.on_handover_ready)
        self.create_timer(self.config.period, self.publish_output)

    def on_handover_ready(self, request, response):
        """A diagnostic gate has no physical ownership ramp to wait for."""
        response.success = (
            bool(request.session_id)
            and request.session_id == self.motor_gate.policy_session_id
            and self.motor_gate.abort_reason is None
            and self.motor_gate.supervisor_phase == SupervisorPhase.RUNNING
        )
        response.message = "ready" if response.success else "not ready"
        return response

    def on_ready(self, _request, response):
        """Confirm the diagnostic gate is unused without opening a session."""
        response.success = (
            self.motor_gate.abort_reason is None
            and self.motor_gate.policy_session_id is None
        )
        response.message = (
            "ready" if response.success else "gate already armed or stopped"
        )
        return response

    def on_postcheck(self, message):
        self.motor_gate.receive(
            command_from_ros(message.command),
            message.host_boot_id,
            message.checked_at_secs,
            message.expires_at_secs,
            time.monotonic(),
        )
        if self.motor_gate.abort_reason:
            self.publish_output()

    def on_abort(self, message):
        if message.source != self.get_name():
            self.motor_gate.abort(message.reason or "external_abort")
            self.publish_output()

    def on_arm(self, request, response):
        response.success = self.motor_gate.arm(request.session_id, time.monotonic())
        response.message = (
            "armed"
            if response.success
            else "gate inhibited by latch or invalid session"
        )
        return response

    def publish_output(self):
        command = self.motor_gate.cycle(time.monotonic())
        if self.motor_gate.abort_reason:
            self.latch_abort(self.motor_gate.abort_reason)
        message = DryRunOutput()
        message.damping = command["kind"] == "damping"
        message.reason = (
            self.motor_gate.abort_reason or self.motor_gate.supervisor_phase.value
        )
        message.command = command_to_ros(command)
        self.dry_run_output_publisher.publish(message)


class ControllerStopNode(SafetyNode):
    """Broadcast Start double-press abort without depending on the checker."""

    def __init__(self):
        super().__init__("controller_stop")
        self.double_press_detector = StartDoublePressDetector(
            self.config.double_press_window_secs, self.config.robot_timeout
        )
        self.time_last_feedback_secs = time.monotonic()
        self.received_controller_feedback = False
        self.startup_timeout_secs = self.declare_parameter(
            "startup_timeout_secs", self.config.robot_timeout
        ).value
        if (
            not math.isfinite(self.startup_timeout_secs)
            or self.startup_timeout_secs <= 0
        ):
            raise ValueError("startup timeout must be finite and positive")
        self.create_service(Trigger, topics.CONTROLLER_READY, self.on_ready)
        self.create_subscription(
            RobotFeedback, topics.ROBOT_FEEDBACK, self.on_feedback, self.feedback_qos
        )
        self.create_timer(self.config.period, self.check_freshness)

    def on_feedback(self, message):
        cur_time_secs = time.monotonic()
        if not self.feedback_is_fresh(message, cur_time_secs):
            self.double_press_detector.reset()
            self.latch_abort("invalid_controller_feedback")
            return
        try:
            stop = self.double_press_detector.update(
                message.start_button_pressed, message.received_at_secs
            )
        except ValueError:
            self.latch_abort("invalid_controller_timestamp")
            return
        self.time_last_feedback_secs = message.received_at_secs
        self.received_controller_feedback = True
        if stop:
            self.latch_abort("controller_double_start")

    def check_freshness(self):
        timeout = (
            self.config.robot_timeout
            if self.received_controller_feedback
            else self.startup_timeout_secs
        )
        if time.monotonic() - self.time_last_feedback_secs >= timeout:
            self.double_press_detector.reset()
            self.latch_abort("controller_feedback_timeout")

    def on_ready(self, _request, response):
        """Confirm fresh released-Start input before the dry-run producer arms."""
        self.check_freshness()
        response.success = (
            self.abort_reason is None
            and self.received_controller_feedback
            and self.double_press_detector.previous_start_pressed is False
        )
        response.message = (
            "ready" if response.success else "Controller input is not ready"
        )
        return response


def main():
    """Launch one role per process so a frozen checker cannot freeze the gate."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "role",
        choices=(
            "checker",
            "motor_gate",
            "controller_stop",
            "feedback",
            "hardware_gate",
        ),
    )
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)

    def _stop_on_term(_signum, _frame):
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, _stop_on_term)
    from .feedback_node import RobotFeedbackNode
    from .hardware_node import HardwareMotorGateNode

    node = {
        "checker": SafetyCheckerNode,
        "motor_gate": MotorGateNode,
        "controller_stop": ControllerStopNode,
        "feedback": RobotFeedbackNode,
        "hardware_gate": HardwareMotorGateNode,
    }[args.role]()
    try:
        if args.role == "feedback":
            node.start_sdk()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Terminal SIGINT and launch shutdown can both arrive during cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
