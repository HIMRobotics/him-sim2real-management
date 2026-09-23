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


if __name__ == "__main__":
    unittest.main()
