"""Control-trace lines stay sparse and still record a stop."""

import json
import tempfile
import unittest
from pathlib import Path

from robot_safety.control_trace import ControlTrace
from robot_safety.hardware_adapter import G1ArmAdapter
from tests.test_robot_safety_hardware import FakeTransport, load_profile, profile_data


class ControlTraceTests(unittest.TestCase):
    def test_steps_are_sampled_and_an_abort_is_kept(self):
        with tempfile.TemporaryDirectory() as directory:
            trace = ControlTrace(Path(directory) / "trace.jsonl")
            trace.open()
            trace.note_repeat(1.0)
            trace.note_repeat(1.003)
            trace.note_gate(1.2)
            trace.note_step({"now": 1.0, "phase": "entering_balance", "gap_rad": 0.1})
            trace.note_step({"now": 1.05, "phase": "entering_balance", "gap_rad": 0.2})
            trace.note_gate(1.4)
            trace.emit_abort("motor_gate_loop_timeout", 1.4)
            trace.close()
            lines = [json.loads(line) for line in trace.path.read_text().splitlines()]
        self.assertEqual([line["gap_rad"] for line in lines], [0.1, 0.2])
        self.assertEqual(lines[0]["repeat_count"], 2)
        self.assertAlmostEqual(lines[0]["repeat_gap_s"], 0.003)
        self.assertEqual(lines[1]["abort"], "motor_gate_loop_timeout")
        self.assertAlmostEqual(lines[1]["gate_gap_s"], 0.2)

    def test_hardware_step_writes_the_checked_command(self):
        with tempfile.TemporaryDirectory() as directory:
            now = 10.0
            adapter = G1ArmAdapter(load_profile(profile_data()), lambda: now)
            adapter.transport = FakeTransport()
            adapter.receive_battery(type("B", (), {"soc": 80})())
            adapter.trace = ControlTrace(Path(directory) / "trace.jsonl")
            adapter.trace.open()
            adapter.receive_state(type("S", (), {
                "motor_state": [type("M", (), {"q": 0.0, "dq": 0.0, "temperature": (25, 25), "tau_est": 0.0})() for _ in range(35)],
                "mode_machine": 5,
                "imu_state": type("I", (), {"quaternion": (1.0, 0.0, 0.0, 0.0), "gyroscope": (0.2, 0.0, 0.0)})(),
                "wireless_remote": [0] * 40,
            })())
            self.assertTrue(adapter.check_ready_mode())
            self.assertTrue(adapter.arm())
            adapter.submit({name: [0.0] * 29 for name in ("q", "dq", "kp", "kd", "tau")}, now + 0.1)
            adapter.step()
            adapter.abort("battery_feedback_timeout")
            text = adapter.trace.path.read_text().strip().splitlines()
        self.assertEqual(len(text), 2)
        step = json.loads(text[0])
        self.assertEqual(step["phase"], "waiting")
        self.assertEqual(step["gyro"], [0.2, 0.0, 0.0])
        self.assertIn("gap_joint", step)
        self.assertEqual(json.loads(text[1])["abort"], "battery_feedback_timeout")


if __name__ == "__main__":
    unittest.main()
