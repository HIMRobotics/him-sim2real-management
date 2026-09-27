"""Verify the ROS gate-to-SDK boundary using a fake transport only."""

import time
import os
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
    def test_startup_ready_requires_warmup_telemetry_and_live_workers(self):
        adapter = SimpleNamespace(
            balance_policy=None, feedback_fresh=Mock(return_value=True),
            feedback=SimpleNamespace(start_button_pressed=False), f3_pressed=False,
            abort_reason=None, workers=[Mock(is_alive=Mock(return_value=True))],
        )
        node = SimpleNamespace(
            adapter=adapter, startup_ready=False,
            motor_gate=SimpleNamespace(abort_reason=None), get_logger=Mock(),
        )
        with patch.dict(os.environ, {"NOTIFY_SOCKET": "@test-safety-notify"}), patch(
            "robot_safety.hardware_node.socket.socket"
        ) as socket:
            notify = lambda: HardwareMotorGateNode.notify_startup_ready(node)
            notify()
            socket.assert_not_called()
            adapter.balance_policy = object()
            adapter.feedback_fresh.return_value = False
            notify()
            socket.assert_not_called()
            adapter.feedback_fresh.return_value = True
            adapter.feedback.start_button_pressed = True
            notify()
            socket.assert_not_called()
            adapter.feedback.start_button_pressed = False
            adapter.workers[0].is_alive.return_value = False
            notify()
            socket.assert_not_called()
            adapter.workers[0].is_alive.return_value = True
            adapter.f3_pressed = True
            notify()
            socket.assert_not_called()
            adapter.f3_pressed = False
            adapter.abort_reason = "fault"
            notify()
            socket.assert_not_called()
            adapter.abort_reason = None
            notify()
            notify()
            sent = socket.return_value.__enter__.return_value.sendto
            sent.assert_called_once()
            self.assertIn(b"READY=1", sent.call_args[0][0])
            self.assertEqual(sent.call_args[0][1], "\0test-safety-notify")
            self.assertTrue(node.startup_ready)

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
            observer = rclpy.create_node("abort_history_observer")
            received = []
            from robot_safety import ros_topics as topics
            from robot_safety.core import SupervisorPhase

            def observe_for(seconds):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    rclpy.spin_once(observer, timeout_sec=0.02)

            try:
                subscription = observer.create_subscription(
                    Abort, topics.ABORT, received.append, node.abort_qos
                )
                observe_for(0.5)
                self.assertTrue(received)
                self.assertEqual(received[-1].reason, "controller_double_start")
                observer.destroy_subscription(subscription)

                node.adapter.release_damp_when_confirmed = True
                node.adapter._request_stock_damping()
                node.publish_output()
                self.assertIsNone(node.abort_reason)
                self.assertIsNone(node.motor_gate.policy_session_id)
                self.assertEqual(node.motor_gate.supervisor_phase, SupervisorPhase.INHIBITED)
                self.assertFalse(node.adapter.armed)
                received.clear()
                observer.create_subscription(
                    Abort, topics.ABORT, received.append, node.abort_qos
                )
                observe_for(0.5)
                self.assertEqual(received, [])

                node.adapter.damp_settled = True
                abort.reason = "new_stop"
                node.on_abort(abort)
                self.assertEqual(node.motor_gate.abort_reason, "new_stop")
                self.assertEqual(node.abort_reason, "new_stop")
            finally:
                observer.destroy_node()
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
