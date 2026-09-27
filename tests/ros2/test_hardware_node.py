"""Verify the ROS gate-to-SDK boundary using a fake transport only."""

import time
import unittest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import Mock, patch

from tests.ros2.environment import require_test_domain

setUpModule = require_test_domain

try:
    import rclpy
    from him_safety_interfaces.msg import Abort, CheckedCommand
    from him_safety_interfaces.srv import ArmGate
    from std_srvs.srv import Trigger

    from robot_safety.hardware_node import HardwareMotorGateNode
    from robot_safety.ros_nodes import command_to_ros
except ImportError as error:
    raise unittest.SkipTest(
        "Build and source ROS packages for adapter tests"
    ) from error

from tests.test_robot_safety_hardware import FakeTransport, load_profile, profile_data


class HardwareNodeTests(unittest.TestCase):
    def test_readiness_timeout_rejects_without_damping(self):
        from robot_safety.hardware_adapter import G1ArmAdapter

        adapter = G1ArmAdapter(load_profile(profile_data()))
        transport = FakeTransport()
        transport.read_ready_fsm = lambda: (3104, None)
        adapter.transport = transport
        node = SimpleNamespace(
            adapter=adapter,
            motor_gate=SimpleNamespace(
                abort_reason=None, policy_session_id=None, abort=Mock()
            ),
            get_logger=Mock(return_value=Mock()),
            latch_abort=Mock(),
        )
        response = HardwareMotorGateNode.on_ready(
            node, Trigger.Request(), Trigger.Response()
        )
        self.assertFalse(response.success)
        self.assertIn("rpc_code=3104", response.message)
        self.assertFalse(adapter.armed)
        self.assertFalse(adapter.stop_requested.is_set())
        adapter.step()
        self.assertEqual(transport.writes, [])
        self.assertEqual(transport.stop_calls, 0)
        node.latch_abort.assert_called_once_with("hardware_readiness_rejected")

    def test_arm_checked_command_and_abort_reach_fake_hardware(self):
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                "enable_hardware_output:=true",
                "-p",
                "hardware_profile_path:=unused_test_profile",
                "-p",
                "sdk_interface:=unused_test_interface",
            ]
        )
        transport = FakeTransport()
        transport.close = lambda: None
        node = None
        try:
            with ExitStack() as patches:
                patches.enter_context(
                    patch(
                        "robot_safety.hardware_node.HardwareProfile.load",
                        return_value=load_profile(profile_data()),
                    )
                )
                patches.enter_context(
                    patch(
                        "robot_safety.hardware_node.UnitreeArmTransport",
                        return_value=transport,
                    )
                )
                patches.enter_context(
                    patch("robot_safety.hardware_node.G1ArmAdapter.start_workers")
                )
                node = HardwareMotorGateNode()
            node.adapter.receive_state(
                SimpleNamespace(
                    mode_machine=5,
                    wireless_remote=[0] * 40,
                    motor_state=[SimpleNamespace(q=0.0, dq=0.0) for _ in range(35)],
                )
            )
            self.assertTrue(
                node.on_ready(Trigger.Request(), Trigger.Response()).success
            )
            request = ArmGate.Request()
            request.session_id = "hardware-test"
            self.assertTrue(node.on_arm(request, ArmGate.Response()).success)
            command = {field: [0.0] * 29 for field in ("q", "dq", "kp", "kd", "tau")}
            command.update(session="hardware-test", seq=1, state_id=1)
            checked = CheckedCommand()
            checked.command = command_to_ros(command)
            checked.host_boot_id = node.host_boot_id
            checked.checked_at_secs = time.monotonic()
            checked.expires_at_secs = checked.checked_at_secs + 0.1
            node.on_postcheck(checked)
            node.publish_output()
            node.adapter.step()
            self.assertEqual(len(transport.writes), 1)
            self.assertEqual(transport.writes[0][1], 0.0)
            abort = Abort()
            abort.source = "controller_stop"
            abort.reason = "controller_double_start"
            node.on_abort(abort)
            node.adapter.stop_once()
            self.assertEqual(transport.stop_calls, 0)
            self.assertEqual(node.adapter.abort_reason, "controller_double_start")
            self.assertEqual(transport.writes[-1][1], 0.0)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
