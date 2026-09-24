"""Isolated ROS graph tests using generated types and fake motor output."""

import time
import unittest

from tests.ros2.environment import require_test_domain

setUpModule = require_test_domain

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


class ArmRaiseGraphTests(unittest.TestCase):
    test_policy = "slow_raise"

    def test_service_start_controller_double_press_and_all_joint_damping(self):
        from robot_safety import ros_topics
        from robot_safety.ros_nodes import ControllerStopNode
        from tests.manual.arm_raise_node import ArmRaiseNode, ArmRaisePhase

        rclpy.init(args=["--ros-args", "-p", f"test_policy:={self.test_policy}"])
        checker = SafetyCheckerNode()
        gate = MotorGateNode()
        producer = ArmRaiseNode()
        from dataclasses import replace

        # This test measures message flow, not OS scheduling. Stall handling has
        # its own deterministic test using the production 20 ms threshold.
        producer.settings = replace(producer.settings, max_publish_gap_secs=0.2)
        controller = ControllerStopNode()
        executor = SingleThreadedExecutor()
        nodes = (checker, gate, producer, controller)
        for node in nodes:
            executor.add_node(node)
        publisher = producer.create_publisher(
            RobotFeedback, ros_topics.ROBOT_FEEDBACK, producer.feedback_qos
        )

        def spin_until(predicate, pressed=False):
            deadline = time.monotonic() + 3.0
            while not predicate() and time.monotonic() < deadline:
                feedback = RobotFeedback()
                feedback.host_boot_id = producer.host_boot_id
                feedback.received_at_secs = time.monotonic()
                feedback.q = [0.2] * 29
                feedback.dq = [0.0] * 29
                feedback.start_button_pressed = pressed
                publisher.publish(feedback)
                executor.spin_once(timeout_sec=0.001)
            self.assertTrue(predicate())

        try:
            # Discover peers before spinning the controller's startup watchdog.
            deadline = time.monotonic() + 3.0
            while not (
                publisher.get_subscription_count() >= 2
                and producer.command_publisher.get_subscription_count() > 0
                and producer.checker_arm_client.service_is_ready()
                and producer.gate_arm_client.service_is_ready()
                and producer.abort_publisher.get_subscription_count() >= 3
            ):
                self.assertLess(time.monotonic(), deadline)
                time.sleep(0.005)
            controller.time_last_feedback_secs = time.monotonic()
            spin_until(lambda: producer.observation_is_ready())
            start_client = producer.create_client(Trigger, "/safety/arm_raise/start")
            self.assertTrue(start_client.wait_for_service(timeout_sec=2.0))
            start_future = start_client.call_async(Trigger.Request())
            spin_until(lambda: start_future.done())
            self.assertTrue(start_future.result().success)
            spin_until(
                lambda: (
                    gate.motor_gate.supervisor_phase == SupervisorPhase.RUNNING
                    and producer.phase == ArmRaisePhase.RUNNING
                )
            )
            self.assertEqual(producer.phase, ArmRaisePhase.RUNNING)
            spin_until(
                lambda: controller.double_press_detector.previous_start_pressed is True,
                pressed=True,
            )
            spin_until(
                lambda: controller.double_press_detector.previous_start_pressed is False
            )
            spin_until(lambda: controller.abort_reason is not None, pressed=True)
            spin_until(
                lambda: (
                    producer.phase == ArmRaisePhase.STOPPED
                    and gate.motor_gate.supervisor_phase == SupervisorPhase.STOP_LATCHED
                )
            )
            self.assertEqual(controller.abort_reason, "controller_double_start")
            output = gate.motor_gate.cycle(time.monotonic())
            self.assertEqual(output["kind"], "damping")
            for field in ("q", "dq", "kp", "tau"):
                self.assertEqual(output[field], [0.0] * 29)
            self.assertEqual(output["kd"], [gate.config.damping] * 29)
            sequence_at_stop = producer.command_sequence
            producer.publish_command()
            self.assertEqual(producer.command_sequence, sequence_at_stop)
            self.assertFalse(
                producer.on_start(Trigger.Request(), Trigger.Response()).success
            )
        finally:
            executor.shutdown()
            for node in reversed(nodes):
                node.destroy_node()
            rclpy.shutdown()


class UnitreeArmGraphTests(ArmRaiseGraphTests):
    test_policy = "unitree_arm7"


class ArmRaiseStopTests(unittest.TestCase):
    def test_slow_readiness_wait_does_not_open_checker_session(self):
        from unittest.mock import Mock

        from rclpy.task import Future

        from tests.manual.arm_raise_node import ArmRaiseNode, ArmRaisePhase

        rclpy.init()
        node = ArmRaiseNode()
        try:
            node.phase = ArmRaisePhase.CHECKING_GATE
            node.time_arm_requested_secs = 10.0
            node.pending_arm_request = Future()
            node.checker_arm_client = Mock()
            node.advance_arming(11.5)
            self.assertEqual(node.phase, ArmRaisePhase.CHECKING_GATE)
            node.checker_arm_client.call_async.assert_not_called()
            self.assertEqual(node.command_sequence, 0)
            node.advance_arming(15.0)
            self.assertEqual(node.phase, ArmRaisePhase.STOPPED)
            self.assertEqual(node.abort_reason, "arm_raise_arming_timeout")
            node.checker_arm_client.call_async.assert_not_called()
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_stall_completion_and_stale_feedback_stop_without_publishing(self):
        from unittest.mock import patch

        from tests.manual.arm_raise import ArmRaiseSettings, SlowArmRaise
        from tests.manual.arm_raise_node import ArmRaiseNode, ArmRaisePhase

        for condition in ("stall", "complete", "stale"):
            with self.subTest(condition=condition):
                rclpy.init()
                node = ArmRaiseNode()
                try:
                    now = time.monotonic()
                    observation = RobotFeedback()
                    observation.host_boot_id = node.host_boot_id
                    observation.received_at_secs = now - (
                        0.3 if condition == "stale" else 0
                    )
                    observation.observation_id = 1
                    observation.q = [0.0] * 29
                    node.latest_observation = observation
                    node.phase = ArmRaisePhase.RUNNING
                    node.trajectory = SlowArmRaise(observation.q, ArmRaiseSettings())
                    node.time_started_secs = now - (
                        11 if condition == "complete" else 0
                    )
                    node.time_last_command_secs = now - (
                        0.03 if condition == "stall" else 0
                    )
                    with patch(
                        "tests.manual.arm_raise_node.time.monotonic", return_value=now
                    ):
                        node.publish_command()
                    expected = {
                        "stall": "arm_raise_publish_gap",
                        "complete": "arm_raise_complete",
                        "stale": "arm_raise_feedback_timeout",
                    }
                    self.assertEqual(node.abort_reason, expected[condition])
                    self.assertEqual(node.command_sequence, 0)
                    self.assertEqual(node.phase, ArmRaisePhase.STOPPED)
                finally:
                    node.destroy_node()
                    rclpy.shutdown()


class ControllerReadinessTests(unittest.TestCase):
    def test_discovery_grace_requires_feedback_and_does_not_extend_active_deadline(
        self,
    ):
        from unittest.mock import patch

        from robot_safety.ros_nodes import ControllerStopNode

        rclpy.init(args=["--ros-args", "-p", "startup_timeout_secs:=5.0"])
        node = ControllerStopNode()
        try:
            initial_time = node.time_last_feedback_secs
            with patch(
                "robot_safety.ros_nodes.time.monotonic", return_value=initial_time + 1
            ):
                response = node.on_ready(Trigger.Request(), Trigger.Response())
                self.assertFalse(response.success)
                self.assertIsNone(node.abort_reason)
                feedback = RobotFeedback()
                feedback.host_boot_id = node.host_boot_id
                feedback.received_at_secs = initial_time + 1
                node.on_feedback(feedback)
                self.assertTrue(
                    node.on_ready(Trigger.Request(), Trigger.Response()).success
                )
                feedback.start_button_pressed = True
                node.on_feedback(feedback)
                self.assertFalse(
                    node.on_ready(Trigger.Request(), Trigger.Response()).success
                )
            with patch(
                "robot_safety.ros_nodes.time.monotonic", return_value=initial_time + 1.3
            ):
                self.assertFalse(
                    node.on_ready(Trigger.Request(), Trigger.Response()).success
                )
                self.assertEqual(node.abort_reason, "controller_feedback_timeout")
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
