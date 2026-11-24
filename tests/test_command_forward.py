"""output_on_command at the hardware gate: forward a new postcheck without the gate's own cycle.

Imports hardware_node with stand-in ROS modules when ROS is not installed, and removes
them again afterwards, so the ROS tests keep their own skip behaviour.
"""

import sys
import types
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from robot_safety.config import Config
from robot_safety.core import SupervisorPhase
from robot_safety.motor_gate import MotorCommandGate


def _ros_stand_ins():
    class Node:
        def __init__(self, *args, **kwargs):
            pass

    def module(name, **attrs):
        mod = types.ModuleType(name)
        for key, value in attrs.items():
            setattr(mod, key, value)
        return mod

    names = ("Abort", "CheckedCommand", "DryRunOutput", "JointCommand", "RobotFeedback")
    return {
        "rclpy": module("rclpy"),
        "rclpy.node": module("rclpy.node", Node=Node),
        "rclpy.qos": module("rclpy.qos", DurabilityPolicy=Mock(), QoSProfile=Mock(),
                            ReliabilityPolicy=Mock()),
        "him_safety_interfaces": module("him_safety_interfaces"),
        "him_safety_interfaces.msg": module("him_safety_interfaces.msg",
                                            **{name: type(name, (), {}) for name in names}),
        "him_safety_interfaces.srv": module("him_safety_interfaces.srv", ArmGate=type("ArmGate", (), {})),
        "std_msgs": module("std_msgs"),
        "std_msgs.msg": module("std_msgs.msg", String=type("String", (), {})),
        "std_srvs": module("std_srvs"),
        "std_srvs.srv": module("std_srvs.srv", Trigger=type("Trigger", (), {})),
    }


@pytest.fixture
def hardware_node():
    try:
        import rclpy  # noqa: F401
        stand_ins = {}
    except ImportError:
        stand_ins = _ros_stand_ins()
    import robot_safety

    names = ("hardware_node", "ros_nodes")
    saved = {name: getattr(robot_safety, name) for name in names if hasattr(robot_safety, name)}
    with patch.dict(sys.modules, stand_ins):
        for name in names:
            if stand_ins:
                sys.modules.pop("robot_safety." + name, None)
        import robot_safety.hardware_node as module
        try:
            yield module
        finally:
            if stand_ins:
                for name in names:
                    sys.modules.pop("robot_safety." + name, None)
                    if name in saved:
                        setattr(robot_safety, name, saved[name])
                    elif hasattr(robot_safety, name):
                        delattr(robot_safety, name)


def make_node(module, *, enabled=True):
    node = module.HardwareMotorGateNode.__new__(module.HardwareMotorGateNode)
    node.adapter = Mock(output_on_command=enabled, abort_reason=None, damp_settled=False)
    node.motor_gate = MotorCommandGate("boot", Config())
    node.publish_output = Mock()
    assert node.motor_gate.arm("session", 100.0)
    return node


def postcheck(seq, now, session="session"):
    zeros = [0.0] * 29
    command = SimpleNamespace(q=[0.1] * 29, dq=zeros, kp=zeros, kd=zeros, tau=zeros,
                              session_id=session, sequence=seq, observation_id=1)
    return SimpleNamespace(command=command, host_boot_id="boot",
                           checked_at_secs=now - 0.001, expires_at_secs=now + 0.1)


def deliver(module, node, message, now):
    with patch.object(module.time, "monotonic", return_value=now), \
            patch("robot_safety.ros_nodes.time.monotonic", return_value=now):
        module.HardwareMotorGateNode.on_postcheck(node, message)


def test_new_postcheck_is_forwarded_without_the_gate_cycle(hardware_node):
    node = make_node(hardware_node)
    deliver(hardware_node, node, postcheck(1, 100.01), 100.01)
    node.adapter.submit.assert_called_once()
    command, expires = node.adapter.submit.call_args.args
    assert command["kind"] == "policy" and command["seq"] == 1 and command["q"] == [0.1] * 29
    assert expires == pytest.approx(100.11)
    # The gate's loop-progress stamp and status stay on the 5 ms timer.
    assert node.motor_gate.time_last_cycle_secs is None
    node.publish_output.assert_not_called()
    assert node.motor_gate.supervisor_phase == SupervisorPhase.RUNNING


def test_option_off_leaves_forwarding_to_the_timer(hardware_node):
    node = make_node(hardware_node, enabled=False)
    deliver(hardware_node, node, postcheck(1, 100.01), 100.01)
    node.adapter.submit.assert_not_called()
    node.publish_output.assert_not_called()


def test_invalid_postcheck_latches_through_the_existing_path(hardware_node):
    node = make_node(hardware_node)
    deliver(hardware_node, node, postcheck(1, 100.01, session="other"), 100.01)
    assert node.motor_gate.abort_reason == "invalid_postcheck"
    node.adapter.submit.assert_not_called()
    node.publish_output.assert_called_once()     # MotorGateNode.on_postcheck, as before


@pytest.mark.parametrize("field", ["abort_reason", "damp_settled"])
def test_stopped_adapter_is_not_forwarded(hardware_node, field):
    node = make_node(hardware_node)
    setattr(node.adapter, field, "hardware_test_stop" if field == "abort_reason" else True)
    deliver(hardware_node, node, postcheck(1, 100.01), 100.01)
    node.adapter.submit.assert_not_called()


def test_gate_deadline_fault_is_not_forwarded(hardware_node):
    node = make_node(hardware_node)
    deliver(hardware_node, node, postcheck(1, 100.5), 100.5)   # 0.5 s after arming: postcheck timeout
    assert node.motor_gate.abort_reason == "postcheck_timeout"
    node.adapter.submit.assert_not_called()
