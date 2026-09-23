"""Independent motor gate tests without ROS or hardware."""

import unittest

from robot_safety.config import Config
from robot_safety.motor_gate import MotorCommandGate


class MotorGateTests(unittest.TestCase):
    def setUp(self):
        self.gate = MotorCommandGate("host", Config(joints=1))
        self.command = {
            "session": "run",
            "seq": 0,
            "state_id": 1,
            "q": [0.1],
            "dq": [0.0],
            "kp": [1.0],
            "kd": [1.0],
            "tau": [0.0],
        }

    def receive(self, now=0.0, checked=0.0, expires=0.2, host="host"):
        return self.gate.receive(self.command, host, checked, expires, now)

    def test_inhibited_until_explicit_arm(self):
        self.assertFalse(self.receive())
        self.assertEqual(self.gate.cycle(0.0)["kind"], "damping")
        self.assertTrue(self.gate.arm("run", 0.0))
        self.assertTrue(self.receive())
        self.assertEqual(self.gate.cycle(0.0)["kind"], "policy")

    def test_frozen_checker_expires_without_abort_message(self):
        self.gate.arm("run", 0.0)
        self.receive(expires=0.15)
        self.assertEqual(self.gate.cycle(0.149)["kind"], "policy")
        self.assertEqual(self.gate.cycle(0.15)["kind"], "damping")
        self.assertEqual(self.gate.abort_reason, "checked_command_expired")

    def test_abort_blocks_all_future_frames_and_rearm(self):
        self.gate.arm("run", 0.0)
        self.receive()
        self.gate.abort("bad_joint")
        self.command["seq"] = 1
        self.assertFalse(self.receive(now=0.01))
        self.assertFalse(self.gate.arm("next_run", 0.01))
        self.assertEqual(self.gate.cycle(0.01)["kind"], "damping")

    def test_replayed_command_blows_fuse(self):
        self.gate.arm("run", 0.0)
        self.receive()
        self.assertFalse(self.receive(now=0.01))
        self.assertEqual(self.gate.abort_reason, "invalid_postcheck")

    def test_invalid_envelopes_and_joint_frames(self):
        for changed in (
            {"host": "different_boot"},
            {"expires": 0.5},
            {"checked": 0.1},
            {"expires": float("nan")},
            {"expires": 0.0},
        ):
            with self.subTest(changed=changed):
                self.setUp()
                self.gate.arm("run", 0.0)
                self.assertFalse(self.receive(**changed))
                self.assertEqual(self.gate.cycle(0.01)["kind"], "damping")
        self.setUp()
        self.gate.arm("run", 0.0)
        self.command["kp"] = [-1.0]
        self.assertFalse(self.receive())

    def test_late_replacement_cannot_hide_timeout(self):
        self.gate.arm("run", 0.0)
        self.receive()
        self.command["seq"] = 1
        self.assertFalse(self.receive(now=0.2, checked=0.2, expires=0.4))
        self.assertEqual(self.gate.abort_reason, "postcheck_timeout")


if __name__ == "__main__":
    unittest.main()
