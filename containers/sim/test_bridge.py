"""ROS callback tests without opening network sockets."""

import importlib.util
import json
import subprocess
import sys
import time
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
    for name in ("Abort", "CheckedCommand", "JointCommand", "RobotFeedback"):
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
        node.motor_gate = None
        node.checker = None
        node.created = 1.0
        node.run_started = 1.0
        node.seconds = 10.0
        node.accepted = True
        node.circle = False
        node.clock = lambda: 0.04
        node.plant = Mock()
        node.plant.call.return_value = {"ok": True, "reason": None, "sim_time": 0.02}
        node.publish_state = Mock()
        node.lowstate = Mock()
        with patch.object(bridge.time, "monotonic", return_value=1.04):
            node.on_control()
        calls = [c.args[0]["op"] for c in node.plant.call.call_args_list]
        self.assertEqual(calls, ["step"])
        self.assertTrue(node.plant.call.call_args.args[0]["feedback"])
        self.assertEqual(node.publish_state.call_args.args[0]["sim_time"], 0.02)

    def test_plant_deadline_handles_missing_and_partial_lines(self):
        bridge = load_bridge()
        for output in ("", '{"ok":'):
            with self.subTest(output=output):
                proc = subprocess.Popen(
                    [
                        sys.executable,
                        "-u",
                        "-c",
                        (
                            "import sys, time; sys.stdin.readline(); "
                            f"sys.stdout.write({output!r}); sys.stdout.flush(); time.sleep(10)"
                        ),
                    ],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    bufsize=0,
                )
                with patch.object(bridge.subprocess, "Popen", return_value=proc):
                    client = bridge.PlantClient(sys.executable, timeout=0.2)
                try:
                    started = time.monotonic()
                    with self.assertRaises(TimeoutError):
                        client.call({"op": "arm"})
                    self.assertLess(time.monotonic() - started, 2)
                    with self.assertRaisesRegex(RuntimeError, "unavailable"):
                        client.call({"op": "arm"})
                finally:
                    client.close()
                self.assertIsNotNone(proc.poll())

    def test_plant_client_exchanges_complete_json_lines(self):
        bridge = load_bridge()
        proc = subprocess.Popen(
            [
                sys.executable,
                "-u",
                "-c",
                ("import sys; [print(line.strip(), flush=True) for line in sys.stdin]"),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            bufsize=0,
        )
        with patch.object(bridge.subprocess, "Popen", return_value=proc):
            client = bridge.PlantClient(sys.executable, timeout=2)
        try:
            for request in ({"op": "arm"}, {"op": "command", "payload": "x" * 100000}):
                self.assertEqual(client.call(request), request)
        finally:
            client.close()

    def test_transport_failure_writes_error_without_another_plant_call(self):
        import tempfile

        bridge = load_bridge()
        node = bridge.SimBridge.__new__(bridge.SimBridge)
        node.finished = False
        node.plant = Mock()
        node.latch = Mock()
        with tempfile.TemporaryDirectory() as tmp:
            node.report_path = Path(tmp) / "report.json"
            node.finish(error=TimeoutError("plant response timed out"))
            report = json.loads(node.report_path.read_text())
        node.plant.call.assert_not_called()
        node.latch.assert_called_once_with("plant_error")
        self.assertEqual(report["status"], "error")
        self.assertEqual(node.exit_code, 2)

    def test_extended_state_is_published_with_command_ids(self):
        bridge = load_bridge()
        node = bridge.SimBridge.__new__(bridge.SimBridge)
        node.clock = lambda: 0.0
        node.checker = None
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
        node.checker = None
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
