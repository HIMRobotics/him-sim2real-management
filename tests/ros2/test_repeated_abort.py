"""Exercise duplicate fault forwarding with fake hardware and no ROS nodes."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from tests.ros2.environment import require_test_domain
from tests.test_recovery_diagnostics import recovery_rig

pytest.importorskip("rclpy")
pytest.importorskip("him_safety_interfaces.msg")

from robot_safety.hardware_node import HardwareMotorGateNode
from robot_safety.hardware_profile import HARDWARE_FEEDBACK_TIMEOUT_SECS
from robot_safety.motor_gate import MotorCommandGate


def setup_module():
    require_test_domain()


def node_for(adapter):
    return SimpleNamespace(
        adapter=adapter, motor_gate=MotorCommandGate("offline-test"),
        notify_startup_ready=Mock(), latch_abort=Mock(), final_diagnostics_logged=True,
        output_enabled=True, hardware_profile_path="offline-test",
        last_status=None, last_status_publish_secs=None,
        status_publisher=Mock(), get_logger=Mock(),
    )


def publish(node):
    HardwareMotorGateNode.publish_output(node)


def test_gate_fault_is_forwarded_once_and_recovery_keeps_checking_health():
    rig = recovery_rig()
    adapter = rig.adapter
    node = node_for(adapter)
    node.motor_gate.abort("thrash_detected")
    with patch.object(adapter, "abort", wraps=adapter.abort) as abort, \
            patch.object(adapter, "check_health", wraps=adapter.check_health) as health:
        publish(node)
        abort.assert_called_once_with("thrash_detected")
        assert health.call_count == 1
        adapter.step()
        assert adapter.soft_reset_active
        assert health.call_count == 4
        calls = health.call_count
        rig.transport.clear_body = Mock(wraps=rig.transport.clear_body)
        for _ in range(200):
            publish(node)
        assert abort.call_count == 1
        assert health.call_count == calls
        rig.transport.clear_body.assert_not_called()
        rig.now += .01
        rig.state()
        assert health.call_count == calls + 1
        adapter.step()
        assert health.call_count == calls + 4
    assert adapter.automatic_soft_reset
    assert len(rig.transport.body_commands) == 2
    assert node.latch_abort.call_count == 201


def test_fault_already_latched_by_feedback_is_not_forwarded_again():
    rig = recovery_rig()
    rig.adapter.abort("thrash_detected")
    node = node_for(rig.adapter)
    with patch.object(rig.adapter, "abort", wraps=rig.adapter.abort) as abort:
        publish(node)
        abort.assert_not_called()
    assert node.motor_gate.abort_reason == "thrash_detected"


def test_different_gate_fault_still_disables_recovery():
    rig = recovery_rig()
    rig.adapter.abort("thrash_detected")
    rig.adapter.step()
    node = node_for(rig.adapter)
    node.motor_gate.abort("hardware_controller_double_start")
    with patch.object(rig.adapter, "abort", wraps=rig.adapter.abort) as abort:
        publish(node)
        abort.assert_called_once_with("hardware_controller_double_start")
    assert rig.adapter.hold_failed
    assert not rig.adapter.walk_stop
    assert not rig.adapter.automatic_soft_reset
    assert rig.transport.body_cleared


def test_missing_feedback_still_stops_recovery_without_timer_reassertion():
    rig = recovery_rig()
    node = node_for(rig.adapter)
    node.motor_gate.abort("thrash_detected")
    publish(node)
    rig.adapter.step()
    count = len(rig.transport.body_commands)
    rig.now += HARDWARE_FEEDBACK_TIMEOUT_SECS + .01
    publish(node)
    rig.adapter.step()
    assert len(rig.transport.body_commands) == count
    assert not rig.adapter.walk_stop
    assert rig.transport.stop_calls > 0


def test_new_bad_posture_feedback_still_aborts_recovery():
    rig = recovery_rig()
    node = node_for(rig.adapter)
    node.motor_gate.abort("thrash_detected")
    publish(node)
    rig.adapter.step()
    message = SimpleNamespace(
        motor_state=[SimpleNamespace(q=0.0, dq=0.0, temperature=(25, 25), tau_est=0.0)
                     for _ in range(35)],
        mode_machine=5, wireless_remote=[0] * 40,
        imu_state=SimpleNamespace(quaternion=(0.0, 1.0, 0.0, 0.0), gyroscope=(0.0,) * 3),
    )
    with patch.object(rig.adapter, "abort", wraps=rig.adapter.abort) as abort:
        rig.adapter.receive_state(message)
        abort.assert_called_once_with("hardware_not_standing", request_damping=True)
    assert rig.adapter.hold_failed
    assert not rig.adapter.walk_stop
    assert rig.transport.body_cleared
