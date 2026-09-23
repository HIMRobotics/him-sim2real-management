"""Propose a slow arm raise to precheck and stop publishing on any abort."""

import signal
import time
from enum import Enum

import rclpy
from him_safety_interfaces.msg import Abort, JointCommand, RobotFeedback
from him_safety_interfaces.srv import ArmGate
from std_srvs.srv import Trigger

from robot_safety import ros_topics as topics
from robot_safety.feedback import G1_29_JOINT_NAMES
from robot_safety.ros_nodes import SafetyNode, command_to_ros

from .arm_raise import ArmRaiseSettings, SlowArmRaise

ARM_RAISE_START = "/safety/arm_raise/start"


class ArmRaisePhase(Enum):
    """One-shot test lifecycle, with no automatic rearm or reset."""

    WAITING = "waiting"
    CHECKING_CONTROLLER = "checking_controller"
    ARMING_CHECKER = "arming_checker"
    ARMING_GATE = "arming_gate"
    RUNNING = "running"
    STOPPED = "stopped"


class ArmRaiseNode(SafetyNode):
    """Publish diagnostic joint proposals after an explicit start service call."""

    def __init__(self):
        super().__init__("dry_run_arm_raise")
        if self.config.joints != len(G1_29_JOINT_NAMES):
            raise ValueError("arm raise requires 29 joints")
        defaults = ArmRaiseSettings()
        self.settings = ArmRaiseSettings(
            angle_rad=self.declare_parameter(
                "raise_angle_rad", defaults.angle_rad
            ).value,
            duration_secs=self.declare_parameter(
                "raise_duration_secs", defaults.duration_secs
            ).value,
        )
        self.phase = ArmRaisePhase.WAITING
        self.latest_observation = None
        self.policy_session_id = None
        self.command_sequence = 0
        self.trajectory = None
        self.time_started_secs = None
        self.time_last_command_secs = None
        self.pending_arm_request = None
        self.time_arm_requested_secs = None
        self.command_publisher = self.create_publisher(
            JointCommand, topics.PRECHECK_COMMAND, self.command_qos
        )
        self.checker_arm_client = self.create_client(Trigger, topics.CHECKER_ARM)
        self.gate_arm_client = self.create_client(ArmGate, topics.GATE_ARM)
        self.controller_ready_client = self.create_client(
            Trigger, topics.CONTROLLER_READY
        )
        self.readiness_reported = False
        self.create_subscription(
            RobotFeedback,
            topics.POLICY_OBSERVATION,
            self.on_observation,
            self.feedback_qos,
        )
        self.create_subscription(Abort, topics.ABORT, self.on_abort, self.abort_qos)
        self.create_service(Trigger, ARM_RAISE_START, self.on_start)
        self.create_timer(self.config.period, self.publish_command)

    def on_observation(self, message):
        """Retain the latest checker-assigned observation for command freshness."""
        self.latest_observation = message

    def on_abort(self, message):
        self.stop_test(message.reason or "external_abort")

    def stop_test(self, reason):
        """Latch abort, cancel pending arming and prohibit subsequent proposals."""
        self.phase = ArmRaisePhase.STOPPED
        if self.pending_arm_request is not None:
            self.pending_arm_request.cancel()
            self.pending_arm_request = None
        self.latch_abort(reason)

    def observation_is_ready(self):
        message = self.latest_observation
        return (
            message is not None
            and self.feedback_is_fresh(message, time.monotonic())
            and message.observation_id > 0
            and not message.start_button_pressed
        )

    def on_start(self, _request, response):
        """Begin asynchronous arming once peers and released-Start feedback exist."""
        if (
            self.phase != ArmRaisePhase.WAITING
            or not self.observation_is_ready()
            or not self.checker_arm_client.service_is_ready()
            or not self.gate_arm_client.service_is_ready()
            or not self.controller_ready_client.service_is_ready()
            or self.command_publisher.get_subscription_count() == 0
        ):
            response.success = False
            response.message = (
                "Need fresh released-Start feedback, peers and an unused test node"
            )
            return response
        self.phase = ArmRaisePhase.CHECKING_CONTROLLER
        self.time_arm_requested_secs = time.monotonic()
        self.pending_arm_request = self.controller_ready_client.call_async(
            Trigger.Request()
        )
        response.success = True
        response.message = "Dry-run arming requested. Any failure latches abort."
        return response

    def advance_arming(self, cur_time_secs):
        """Arm checker then gate without blocking feedback or abort callbacks."""
        if cur_time_secs - self.time_arm_requested_secs >= self.config.action_timeout:
            self.stop_test("arm_raise_arming_timeout")
            return
        if not self.pending_arm_request.done():
            return
        try:
            response = self.pending_arm_request.result()
            if not response.success:
                self.stop_test("arm_raise_arming_rejected")
                return
            if self.phase == ArmRaisePhase.CHECKING_CONTROLLER:
                self.pending_arm_request = self.checker_arm_client.call_async(
                    Trigger.Request()
                )
                self.phase = ArmRaisePhase.ARMING_CHECKER
            elif self.phase == ArmRaisePhase.ARMING_CHECKER:
                self.policy_session_id = response.message
                request = ArmGate.Request()
                request.session_id = self.policy_session_id
                self.pending_arm_request = self.gate_arm_client.call_async(request)
                self.phase = ArmRaisePhase.ARMING_GATE
            else:
                self.trajectory = SlowArmRaise(self.latest_observation.q, self.settings)
                self.time_started_secs = cur_time_secs
                self.time_last_command_secs = cur_time_secs
                self.phase = ArmRaisePhase.RUNNING
                self.pending_arm_request = None
                self.get_logger().info(
                    "Dry-run arm raise started. Double Start requests abort."
                )
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Arm raise initialization failed: {error}")
            # RPC or malformed feedback failures must never leave a partial test armed.
            self.stop_test("arm_raise_start_failed")

    def publish_command(self):
        """Send fresh proposals until abort, completion, stale state or a loop stall."""
        if self.phase == ArmRaisePhase.WAITING:
            ready = (
                self.observation_is_ready()
                and self.checker_arm_client.service_is_ready()
                and self.gate_arm_client.service_is_ready()
                and self.controller_ready_client.service_is_ready()
                and self.command_publisher.get_subscription_count() > 0
            )
            if ready and not self.readiness_reported:
                self.get_logger().info(
                    "Feedback and services discovered. Ready for explicit start request. "
                    "Controller readiness will be checked before arming."
                )
            self.readiness_reported = ready
            return
        if self.phase == ArmRaisePhase.STOPPED:
            return
        message = self.latest_observation
        # Start may be held during a double press, so only require release at arming.
        if message is None or not self.feedback_is_fresh(message, time.monotonic()):
            self.stop_test("arm_raise_feedback_timeout")
            return
        cur_time_secs = time.monotonic()
        if self.phase in (
            ArmRaisePhase.CHECKING_CONTROLLER,
            ArmRaisePhase.ARMING_CHECKER,
            ArmRaisePhase.ARMING_GATE,
        ):
            self.advance_arming(cur_time_secs)
            if self.phase != ArmRaisePhase.RUNNING:
                return
        if (
            cur_time_secs - self.time_last_command_secs
            >= self.settings.max_publish_gap_secs
        ):
            # Stop instead of jumping forward along the trajectory after a delayed timer.
            self.stop_test("arm_raise_publish_gap")
            return
        elapsed_secs = cur_time_secs - self.time_started_secs
        if elapsed_secs >= self.settings.duration_secs:
            self.stop_test("arm_raise_complete")
            return
        self.command_sequence += 1
        command = self.trajectory.command_at(elapsed_secs)
        command.update(
            session=self.policy_session_id,
            seq=self.command_sequence,
            state_id=message.observation_id,
        )
        self.command_publisher.publish(command_to_ros(command))
        self.time_last_command_secs = cur_time_secs


def main():
    """Run the manual test producer without adding a production safety-node role."""
    rclpy.init()
    node = ArmRaiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
