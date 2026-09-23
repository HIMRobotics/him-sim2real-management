"""Isolated ROS graph tests using generated types and fake motor output."""

import time
import unittest

try:
    import rclpy
    from him_safety_interfaces.msg import JointCommand, RobotFeedback
    from him_safety_interfaces.srv import ArmGate
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger

    from robot_safety.core import SupervisorPhase
    from robot_safety.ros_nodes import MotorGateNode, SafetyCheckerNode
except ImportError as error:
    raise unittest.SkipTest(
        "Build and source the ROS 2 packages to run graph tests"
    ) from error


class SafetyGraphTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.checker = SafetyCheckerNode()
        self.motor = MotorGateNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.checker)
        self.executor.add_node(self.motor)
        self.wait_for(
            lambda: (
                self.checker.postcheck_publisher.get_subscription_count() > 0
                and self.checker.abort_publisher.get_subscription_count() >= 2
            )
        )

    def tearDown(self):
        self.executor.shutdown()
        self.checker.destroy_node()
        self.motor.destroy_node()
        rclpy.shutdown()

    def wait_for(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.005)
        self.assertTrue(predicate())

    def arm(self):
        feedback = RobotFeedback()
        feedback.host_boot_id = self.checker.host_boot_id
        feedback.received_at_secs = time.monotonic()
        feedback.q = [0.0] * 29
        feedback.dq = [0.0] * 29
        self.checker.on_feedback(feedback)
        response = self.checker.on_arm(Trigger.Request(), Trigger.Response())
        self.assertTrue(response.success)
        request = ArmGate.Request()
        request.session_id = response.message
        self.assertTrue(self.motor.on_arm(request, ArmGate.Response()).success)
        command = JointCommand()
        command.session_id = response.message
        command.sequence = 1
        command.observation_id = self.checker.supervisor.latest_observation_id
        command.q = [0.0] * 29
        command.dq = [0.0] * 29
        command.kp = [1.0] * 29
        command.kd = [1.0] * 29
        command.tau = [0.0] * 29
        return command

    def test_valid_relay_then_invalid_joint_aborts_gate(self):
        command = self.arm()
        self.checker.on_precheck(command)
        self.wait_for(
            lambda: self.motor.motor_gate.supervisor_phase == SupervisorPhase.RUNNING
        )
        command.sequence = 2
        command.kp[0] = -1.0
        self.checker.on_precheck(command)
        self.wait_for(
            lambda: (
                self.motor.motor_gate.supervisor_phase == SupervisorPhase.STOP_LATCHED
            )
        )
        self.assertEqual(
            self.motor.motor_gate.cycle(time.monotonic())["kind"], "damping"
        )
        self.assertEqual(self.checker.abort_reason, "invalid_action")

    def test_silent_checker_causes_independent_gate_damping(self):
        self.checker.on_precheck(self.arm())
        self.wait_for(
            lambda: self.motor.motor_gate.supervisor_phase == SupervisorPhase.RUNNING
        )
        self.executor.remove_node(self.checker)
        self.wait_for(
            lambda: (
                self.motor.motor_gate.supervisor_phase == SupervisorPhase.STOP_LATCHED
            )
        )
        self.assertIn(
            self.motor.motor_gate.abort_reason,
            ("checked_command_expired", "postcheck_timeout"),
        )


class FeedbackBridgeTests(unittest.TestCase):
    def setUp(self):
        from robot_safety.feedback_node import RobotFeedbackNode

        rclpy.init(args=["--ros-args", "-p", "sdk_interface:=test_unused"])
        self.bridge = RobotFeedbackNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge)

    def tearDown(self):
        self.executor.shutdown()
        self.bridge.destroy_node()
        rclpy.shutdown()

    def test_receipt_time_and_button_state_delivered(self):
        from robot_safety import ros_topics
        from robot_safety.feedback import FeedbackSample

        received = []
        subscription = self.bridge.create_subscription(
            RobotFeedback,
            ros_topics.ROBOT_FEEDBACK,
            received.append,
            self.bridge.feedback_qos,
        )
        deadline = time.monotonic() + 3.0
        while self.bridge.feedback_publisher.get_subscription_count() == 0:
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.005)
        timestamp = time.monotonic()
        self.bridge.inbox.samples.put_nowait(
            FeedbackSample(timestamp, (0.1,) * 29, (0.0,) * 29, True)
        )
        self.bridge.publish_feedback()
        while not received:
            self.assertLess(time.monotonic(), deadline)
            self.executor.spin_once(timeout_sec=0.005)
        self.assertEqual(received[0].received_at_secs, timestamp)
        self.assertTrue(received[0].start_button_pressed)
        self.assertEqual(received[0].observation_id, 0)
        self.bridge.destroy_subscription(subscription)

    def test_initial_discovery_has_separate_deadline(self):
        from unittest.mock import patch

        from robot_safety.feedback import SDK_DISCOVERY_TIMEOUT_SECS

        with patch(
            "robot_safety.feedback_node.time.monotonic",
            return_value=self.bridge.time_started_secs + 0.3,
        ):
            self.bridge.publish_feedback()
            self.assertIsNone(self.bridge.abort_reason)
        with patch(
            "robot_safety.feedback_node.time.monotonic",
            return_value=self.bridge.time_started_secs + SDK_DISCOVERY_TIMEOUT_SECS,
        ):
            self.bridge.publish_feedback()
            self.assertEqual(self.bridge.abort_reason, "sdk_discovery_timeout")

    def test_missing_feedback_after_discovery_uses_short_deadline(self):
        self.bridge.time_last_feedback_secs = time.monotonic() - 0.3
        self.bridge.publish_feedback()
        self.assertEqual(self.bridge.abort_reason, "sdk_feedback_timeout")

    def test_stale_sample_latches_and_blocks_recovery(self):
        from robot_safety.feedback import FeedbackSample

        self.bridge.inbox.samples.put_nowait(
            FeedbackSample(time.monotonic() - 1.0, (0.0,) * 29, (0.0,) * 29, False)
        )
        self.bridge.publish_feedback()
        self.assertEqual(self.bridge.abort_reason, "stale_or_reordered_sdk_feedback")
        self.bridge.inbox.samples.put_nowait(
            FeedbackSample(time.monotonic(), (0.0,) * 29, (0.0,) * 29, False)
        )
        self.bridge.publish_feedback()
        self.assertEqual(self.bridge.inbox.samples.qsize(), 1)


if __name__ == "__main__":
    unittest.main()
