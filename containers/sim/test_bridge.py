"""ROS callback tests without opening network sockets."""

import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]


def load_bridge():
    names = (
        "rclpy",
        "rclpy.node",
        "rclpy.qos",
        "him_safety_interfaces",
        "him_safety_interfaces.msg",
        "std_msgs",
        "std_msgs.msg",
        "std_srvs",
        "std_srvs.srv",
    )
    modules = {name: types.ModuleType(name) for name in names}
    modules["rclpy.node"].Node = object
    for name in ("DurabilityPolicy", "QoSProfile", "ReliabilityPolicy"):
        setattr(modules["rclpy.qos"], name, Mock())
    for name in ("Abort", "JointCommand", "RobotFeedback"):
        setattr(modules["him_safety_interfaces.msg"], name, types.SimpleNamespace)
    modules["std_msgs.msg"].String = types.SimpleNamespace
    modules["std_srvs.srv"].Trigger = object
    spec = importlib.util.spec_from_file_location(
        "sim_bridge_test", ROOT / "containers/sim/bridge.py"
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules), patch.object(sys, "path", list(sys.path)):
        spec.loader.exec_module(module)
    return module


class BridgeTests(unittest.TestCase):
    def test_control_publishes_state_after_physics_step(self):
        bridge = load_bridge()
        node = bridge.SimBridge.__new__(bridge.SimBridge)
        node.finished = False
        node.run_started = 1.0
        node.seconds = 10.0
        node.accepted = True
        node.clock = lambda: 0.04
        node.plant = Mock()
        node.plant.call.side_effect = [
            {"ok": True, "reason": None},
            {"ok": True, "sim_time": 0.02},
        ]
        node.publish_state = Mock()
        node.lowstate = Mock()
        with patch.object(bridge.time, "monotonic", return_value=1.04):
            node.on_control()
        calls = [c.args[0]["op"] for c in node.plant.call.call_args_list]
        self.assertEqual(calls, ["step", "feedback"])
        self.assertEqual(node.publish_state.call_args.args[0]["sim_time"], 0.02)

    def test_extended_state_is_published_with_command_ids(self):
        bridge = load_bridge()
        node = bridge.SimBridge.__new__(bridge.SimBridge)
        node.clock = lambda: 0.0
        node.feedback_pub, node.observation_pub, node.sim_state_pub = (
            Mock(),
            Mock(),
            Mock(),
        )
        state = {
            "q": [0.0] * 29,
            "dq": [0.0] * 29,
            "observation_id": 4,
            "session_id": "sim-test",
            "position": [1, 0, 0.79],
            "base_velocity": [0] * 6,
        }
        node.publish_state(state)
        message = node.sim_state_pub.publish.call_args.args[0]
        self.assertEqual(json.loads(message.data), state)

    def test_namespace_guard_uses_actual_interfaces(self):
        bridge = load_bridge()
        with (
            patch.dict(bridge.os.environ, {"G1_SIM_NETNS": "1"}),
            patch.object(bridge.socket, "if_nameindex", return_value=[(1, "lo")]),
        ):
            self.assertTrue(bridge.require_isolated_netns())
        with (
            patch.dict(bridge.os.environ, {"G1_SIM_NETNS": "1"}),
            patch.object(
                bridge.socket, "if_nameindex", return_value=[(1, "lo"), (2, "wlan0")]
            ),
        ):
            self.assertFalse(bridge.require_isolated_netns())

    def test_unchecked_completion_has_success_exit_code(self):
        import tempfile

        bridge = load_bridge()
        node = bridge.SimBridge.__new__(bridge.SimBridge)
        node.finished = False
        node.lowstate = Mock(ok=False)
        node.plant = Mock()
        node.plant.call.return_value = {"status": "completed", "commands": 3}
        with tempfile.TemporaryDirectory() as tmp:
            node.report_path = Path(tmp) / "report.json"
            node.finish()
            self.assertEqual(node.exit_code, 0)
            self.assertEqual(
                json.loads(node.report_path.read_text())["execution"], "docker_ros"
            )


if __name__ == "__main__":
    unittest.main()
