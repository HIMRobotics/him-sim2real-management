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

from containers.sim import netns

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
    def test_new_fault_reaches_recovery_after_submission_abort(self):
        bridge = load_bridge()
        node = bridge.SimBridge.__new__(bridge.SimBridge)
        node.abort_reason = "action_timeout"
        node.abort_at = 1.0
        node.plant = Mock(failed=False)
        node.motor_gate = Mock()
        node.latch("action_timeout")
        node.plant.call.assert_not_called()
        node.latch("hardware_controller_double_start")
        node.plant.call.assert_called_once_with({
            "op": "abort", "reason": "hardware_controller_double_start", "detail": None,
        })
        self.assertEqual(node.abort_reason, "action_timeout")
        self.assertEqual(node.abort_at, 1.0)

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
        node.last_tick, node.stalls = None, []
        with patch.object(bridge.time, "monotonic", return_value=1.04):
            node.on_control()
        calls = [c.args[0]["op"] for c in node.plant.call.call_args_list]
        self.assertEqual(calls, ["step"])
        self.assertTrue(node.plant.call.call_args.args[0]["feedback"])
        self.assertEqual(node.publish_state.call_args.args[0]["sim_time"], 0.02)

    def test_control_records_gaps_where_the_host_stopped_the_simulator(self):
        bridge = load_bridge()
        node = bridge.SimBridge.__new__(bridge.SimBridge)
        node.finished = False
        node.motor_gate = None
        node.checker = None
        node.created = node.run_started = 1.0
        node.seconds = 10.0
        node.accepted = node.circle = False
        node.clock = lambda: 0.0
        node.plant = Mock()
        node.plant.call.return_value = {"ok": True}
        node.publish_state = node.lowstate = Mock()
        node.last_tick, node.stalls = None, []
        for now in (1.00, 1.02, 1.30, 1.32):
            with patch.object(bridge.time, "monotonic", return_value=now):
                node.on_control()
        self.assertEqual(node.stalls, [(1.02, 1.30)])

    def test_lockstep_steps_once_per_reply_on_the_simulated_clock(self):
        bridge = load_bridge()
        node = bridge.SimBridge.__new__(bridge.SimBridge)
        node.finished = False
        node.motor_gate = None
        node.checker = None
        node.abort_reason = None
        node.lockstep = True
        node.created = node.sim_clock = node.run_started = 100.0
        node.seconds = 10.0
        node.accepted = True
        node.circle = False
        node.plant = Mock()
        node.plant.call.side_effect = lambda request: {
            "ok": True, "observation_id": node.plant.call.call_count}
        node.publish_state = Mock()
        node.lowstate = Mock()
        node.latch = Mock()
        with patch.object(bridge.time, "monotonic", return_value=500.0):
            node.on_control()
            node.on_control()
        self.assertEqual(node.plant.call.call_count, 1)
        request = node.plant.call.call_args.args[0]
        self.assertEqual((request["op"], request["time"]), ("step", request["feedback_time"]))
        self.assertAlmostEqual(request["time"], 0.02)
        self.assertEqual(node.awaiting, 1)
        with patch.object(bridge.time, "monotonic", return_value=900.0):
            node.replied()
        self.assertEqual(node.plant.call.call_count, 2)
        with patch.object(bridge.time, "monotonic", return_value=900.0):
            node.replied()
        self.assertEqual(node.plant.call.call_count, 2)
        node.awaiting = 2
        self.assertAlmostEqual(node.clock(), 0.04)
        with patch.object(bridge.time, "monotonic", return_value=900.0 + bridge.LOCKSTEP_REPLY_SECONDS):
            node.on_control()
        self.assertEqual((node.plant.call.call_count, node.missed_replies), (3, 1))
        self.assertAlmostEqual(node.clock(), 0.06)
        node.latch.assert_not_called()

    def test_clean_end_keeps_simulating_the_handoff_without_the_policy(self):
        bridge = load_bridge()
        node = bridge.SimBridge.__new__(bridge.SimBridge)
        node.finished = False
        node.motor_gate = Mock(abort_reason=None)
        node.checker = None
        node.abort_reason = None
        node.lockstep = True
        node.created = node.sim_clock = node.run_started = 100.0
        node.seconds = 0.01
        node.accepted = True
        node.circle = False
        node.plant = Mock()
        node.plant.call.side_effect = lambda request: (
            {"ok": True, "handoff": True} if request["op"] == "end"
            else {"ok": True, "observation_id": 1})
        node.publish_state = Mock()
        node.lowstate = Mock()
        node.finish = Mock(side_effect=lambda: setattr(node, "finished", True))
        with patch.object(bridge, "HANDOFF_SECONDS", 0.03):
            node.on_control()
            node.on_precheck(Mock())
            ops = [c.args[0]["op"] for c in node.plant.call.call_args_list]
            self.assertEqual(ops, ["end", "step"])
            self.assertIsNotNone(node.ended_at)
            node.publish_state.assert_not_called()
            self.assertIsNone(node.awaiting)
            node.on_control()
            node.finish.assert_not_called()
            node.on_control()
        node.finish.assert_called_once()
        node.motor_gate.cycle.assert_called_once()

    def test_stop_during_a_host_stall_is_flagged_inconclusive(self):
        import tempfile

        bridge = load_bridge()
        for stalls, flagged in (([(9.8, 10.4)], True), ([(2.0, 2.6)], False)):
            node = bridge.SimBridge.__new__(bridge.SimBridge)
            node.finished = False
            node.lowstate = Mock(ok=False)
            node.checker = None
            node.plant = Mock()
            node.plant.call.return_value = {"status": "violation", "reason": "observation_timeout"}
            node.stalls, node.abort_at = stalls, 10.4
            with tempfile.TemporaryDirectory() as tmp:
                node.report_path = Path(tmp) / "report.json"
                node.finish()
                report = json.loads(node.report_path.read_text())
            self.assertEqual(node.exit_code, 1)
            self.assertEqual(report["status"], "violation")
            if flagged:
                self.assertEqual(report["host_stall"], {"longest_gap_secs": 0.6})
            else:
                self.assertNotIn("host_stall", report)

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

    def test_plant_startup_call_gets_a_longer_deadline(self):
        bridge = load_bridge()
        proc = subprocess.Popen(
            [
                sys.executable,
                "-u",
                "-c",
                "import sys, time; line = sys.stdin.readline(); time.sleep(0.5); print(line.strip(), flush=True)",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            bufsize=0,
        )
        with patch.object(bridge.subprocess, "Popen", return_value=proc):
            client = bridge.PlantClient(sys.executable, timeout=0.2)
        try:
            self.assertEqual(client.call({"op": "arm"}, timeout=5), {"op": "arm"})
        finally:
            client.close()

    def test_plant_startup_failure_writes_error_report(self):
        import tempfile

        bridge = load_bridge()
        plant = Mock()
        plant.call.side_effect = TimeoutError("mujoco plant response timed out")
        lowstate = types.ModuleType("lowstate")
        lowstate.SimLowState = Mock()
        with tempfile.TemporaryDirectory() as tmp:
            report_path = Path(tmp) / "report.json"
            env = {"G1_SIM_REPORT": str(report_path), "G1_SIM_EXAMPLE": "walker"}
            with patch.object(bridge, "require_isolated_netns", return_value=True), \
                    patch.object(bridge, "PlantClient", return_value=plant), \
                    patch.dict(sys.modules, {"lowstate": lowstate}), \
                    patch.dict(bridge.os.environ, env):
                code = bridge.main()
            report = json.loads(report_path.read_text())
        self.assertEqual(code, 2)
        self.assertEqual(plant.call.call_args.kwargs["timeout"], bridge.PLANT_STARTUP_SECONDS)
        self.assertEqual(report["status"], "error")
        self.assertEqual(report["reason"], "plant_error")
        self.assertIn("timed out", report["detail"])
        self.assertEqual(report["policy"], "walker")

    def test_transport_failure_writes_error_without_another_plant_call(self):
        import tempfile

        bridge = load_bridge()
        node = bridge.SimBridge.__new__(bridge.SimBridge)
        node.finished = False
        node.plant = Mock()
        node.latch = Mock()
        node.stalls, node.abort_at = [], None
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
        node.feedback_pub, node.observation_pub, node.sim_state_pub, node.policy_state_pub = (
            Mock(),
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
        published = json.loads(message.data)
        self.assertEqual({key: published[key] for key in state}, state)
        self.assertFalse(published["control_ready"])
        self.assertGreater(published["received_at_secs"], 0)
        node.policy_state_pub.publish.assert_called_once_with(message)
        node.publish_state(dict(state, control_ready=True))
        ready = json.loads(node.policy_state_pub.publish.call_args.args[0].data)
        self.assertTrue(ready["control_ready"])

    def test_namespace_guard_uses_actual_interfaces(self):
        bridge = load_bridge()
        with (
            patch.dict(bridge.os.environ, {"G1_SIM_NETNS": "1"}),
            patch.object(netns.socket, "if_nameindex", return_value=[(1, "lo")]),
        ):
            self.assertTrue(bridge.require_isolated_netns())
        with (
            patch.dict(bridge.os.environ, {"G1_SIM_NETNS": "1"}),
            patch.object(
                netns.socket, "if_nameindex", return_value=[(1, "lo"), (2, "wlan0")]
            ),
        ):
            self.assertFalse(bridge.require_isolated_netns())

    def test_namespace_guard_ignores_only_idle_fallback_tunnels(self):
        interfaces = [(1, "lo"), (2, "gre0"), (3, "sit0"), (4, "tunl0")]
        with (
            patch.object(netns.socket, "if_nameindex", return_value=interfaces),
            patch.object(netns, "interface_up", return_value=False),
        ):
            self.assertTrue(netns.loopback_only())
        with patch.object(netns.socket, "if_nameindex", return_value=interfaces):
            with patch.object(netns, "interface_up", return_value=True):
                self.assertFalse(netns.loopback_only())
            with patch.object(netns.Path, "read_text", side_effect=OSError):
                self.assertFalse(netns.loopback_only())
        with (
            patch.object(netns.socket, "if_nameindex", return_value=[(1, "lo"), (2, "eth0")]),
            patch.object(netns, "interface_up", return_value=False),
        ):
            self.assertFalse(netns.loopback_only())

    def test_unchecked_completion_has_success_exit_code(self):
        import tempfile

        bridge = load_bridge()
        node = bridge.SimBridge.__new__(bridge.SimBridge)
        node.finished = False
        node.lowstate = Mock(ok=False)
        node.checker = None
        node.plant = Mock()
        node.plant.call.return_value = {"status": "completed", "commands": 3}
        node.stalls, node.abort_at = [], None
        with tempfile.TemporaryDirectory() as tmp:
            node.report_path = Path(tmp) / "report.json"
            node.finish()
            self.assertEqual(node.exit_code, 0)
            self.assertEqual(
                json.loads(node.report_path.read_text())["execution"], "docker_ros"
            )


if __name__ == "__main__":
    unittest.main()
