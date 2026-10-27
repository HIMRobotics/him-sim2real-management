"""Exercise duplicate fault forwarding with fake hardware and no ROS nodes."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from tests.ros2.environment import require_test_domain
from tests.test_recovery_diagnostics import recovery_rig

pytest.importorskip("rclpy")
pytest.importorskip("him_safety_interfaces.msg")

from robot_safety.hardware_node import HardwareMotorGateNode
from robot_safety.motor_gate import MotorCommandGate


def setup_module():
    require_test_domain()


def node_for(adapter):
    return SimpleNamespace(
        adapter=adapter, motor_gate=MotorCommandGate("offline-test"),
        _forwarded_gate_fault=None,
        notify_startup_ready=Mock(), latch_abort=Mock(), final_diagnostics_logged=True,
        publish_joint_diagnostics=Mock(),
        output_enabled=True, hardware_profile_path="offline-test",
        last_status=None, last_status_publish_secs=None,
        status_publisher=Mock(), get_logger=Mock(),
    )


def publish(node):
    HardwareMotorGateNode.publish_output(node)


def test_repeated_fault_is_forwarded_only_once():
    rig = recovery_rig()
    node = node_for(rig.adapter)
    node.motor_gate.abort("thrash_detected")
    with patch.object(rig.adapter, "abort", wraps=rig.adapter.abort) as abort:
        publish(node)
        publish(node)
        abort.assert_called_once_with("thrash_detected")


def test_new_fault_is_forwarded_after_an_existing_fault():
    rig = recovery_rig()
    rig.adapter.abort("thrash_detected")
    node = node_for(rig.adapter)
    node.motor_gate.abort("hardware_controller_double_start")
    with patch.object(rig.adapter, "abort", wraps=rig.adapter.abort) as abort:
        publish(node)
        publish(node)
        abort.assert_called_once_with("hardware_controller_double_start")


def test_same_fault_is_forwarded_again_after_new_session():
    rig = recovery_rig()
    node = node_for(rig.adapter)
    node.host_boot_id = "offline-test"
    node.config = node.motor_gate.config
    node.motor_gate.abort("thrash_detected")
    with patch.object(rig.adapter, "abort", wraps=rig.adapter.abort) as abort:
        publish(node)
        publish(node)
        abort.assert_called_once_with("thrash_detected")

        # Represent completed hardware release before accepting another session.
        rig.adapter.armed = False
        rig.adapter.ownership_taken = False
        rig.adapter.balance_released = False
        rig.adapter.stop_requested.clear()
        assert HardwareMotorGateNode._accept_new_session(node)

        node.motor_gate.abort("thrash_detected")
        publish(node)
        publish(node)
        assert abort.call_count == 2
        abort.assert_called_with("thrash_detected")
