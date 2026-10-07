"""Controller gesture and supervisor integration tests without hardware."""

import unittest

from robot_safety.config import Config
from robot_safety.controller import (
    LeftAndBDetector,
    StartDoublePressDetector,
    decode_b_pressed,
    decode_left_pressed,
    decode_start_pressed,
)
from robot_safety.core import Supervisor


class ControllerTests(unittest.TestCase):
    def detector(self, stale_timeout_secs=2.0):
        return StartDoublePressDetector(0.5, stale_timeout_secs)

    def test_sdk_start_bit_and_payload_validation(self):
        payload = bytearray(40)
        payload[2] = 8
        self.assertFalse(decode_start_pressed(payload))
        payload[2] = 4
        self.assertTrue(decode_start_pressed(payload))
        for invalid in (b"short", [256] * 40):
            with self.assertRaises(ValueError):
                decode_start_pressed(invalid)

    def test_left_and_b_use_the_sdk_joystick_bits(self):
        payload = bytearray(40)
        payload[3] = 1 << 7
        self.assertTrue(decode_left_pressed(payload))
        self.assertFalse(decode_b_pressed(payload))
        payload[3] = 1 << 1
        self.assertTrue(decode_b_pressed(payload))
        self.assertFalse(decode_left_pressed(payload))
        payload[3] = (1 << 7) | (1 << 1)
        self.assertTrue(decode_left_pressed(payload) and decode_b_pressed(payload))
        self.assertFalse(decode_start_pressed(payload))

    def test_left_and_b_requires_a_rising_combo(self):
        detector = LeftAndBDetector()
        self.assertFalse(detector.update(True, True))
        self.assertFalse(detector.update(True, True))
        self.assertFalse(detector.update(False, True))
        self.assertFalse(detector.update(True, False))
        self.assertTrue(detector.update(True, True))
        self.assertFalse(detector.update(True, True))

    def test_double_press_boundaries(self):
        for second_press, expected in (
            (0.599999, True),
            (0.6, True),
            (0.600001, False),
        ):
            with self.subTest(second_press=second_press):
                detector = self.detector()
                self.assertFalse(detector.update(False, 0.0))
                self.assertFalse(detector.update(True, 0.1))
                self.assertFalse(detector.update(False, 0.2))
                self.assertEqual(detector.update(True, second_press), expected)

    def test_hold_repeated_samples_and_triple_press(self):
        detector = self.detector()
        samples = [
            (False, 0),
            (True, 0.1),
            (True, 0.15),
            (False, 0.2),
            (True, 0.3),
            (True, 0.35),
            (False, 0.4),
            (True, 0.5),
        ]
        self.assertEqual(
            [detector.update(*sample) for sample in samples],
            [False, False, False, False, True, False, False, False],
        )

    def test_startup_held_and_reset_require_release(self):
        detector = self.detector()
        for repeat in range(2):
            detector.reset()
            self.assertFalse(detector.update(True, 0))
            self.assertFalse(detector.update(True, 0.1))
            self.assertFalse(detector.update(False, 0.2))
            self.assertFalse(detector.update(True, 0.3))
            self.assertFalse(detector.update(False, 0.4))
            self.assertTrue(detector.update(True, 0.5))

    def test_expired_pair_restarts_on_late_press(self):
        detector = self.detector()
        for pressed, timestamp in (
            (False, 0),
            (True, 0.1),
            (False, 0.2),
            (True, 0.7),
            (False, 0.8),
        ):
            self.assertFalse(detector.update(pressed, timestamp))
        self.assertTrue(detector.update(True, 0.9))

    def test_stale_gap_and_invalid_time_discard_history(self):
        detector = self.detector(stale_timeout_secs=0.2)
        detector.update(False, 0)
        detector.update(True, 0.05)
        detector.update(False, 0.1)
        self.assertFalse(detector.update(True, 0.4))
        self.assertFalse(detector.update(False, 0.45))
        self.assertFalse(detector.update(True, 0.5))
        with self.assertRaises(ValueError):
            detector.update(False, 0.49)
        self.assertFalse(detector.update(True, 0.6))

    def test_physical_hold_and_stop_event_are_independent(self):
        supervisor = Supervisor(Config(joints=1))
        observation = {"q": [0.0], "dq": [0.0]}
        supervisor.state(observation, False, 0.0)
        supervisor.state(observation, True, 0.01)
        self.assertEqual(supervisor.supervisor_phase, "INHIBITED")
        self.assertIn("error", supervisor.control("arm", 0.02))
        supervisor.state(observation, False, 0.03, stop_requested=True)
        self.assertEqual(supervisor.supervisor_phase, "STOP_LATCHED")
        self.assertFalse(supervisor.status(0.03)["start_button_pressed"])
        self.assertNotIn("error", supervisor.control("reset", 0.04))
        self.assertEqual(supervisor.supervisor_phase, "INHIBITED")

    def test_double_press_latches_in_all_phases(self):
        for phase in ("INHIBITED", "ARMED", "RUNNING", "STOP_LATCHED"):
            with self.subTest(phase=phase):
                supervisor = Supervisor(Config(joints=1))
                observation = {"q": [0.0], "dq": [0.0]}
                supervisor.state(observation, False, 0.0)
                if phase in ("ARMED", "RUNNING"):
                    supervisor.control("arm", 0.0)
                if phase == "RUNNING":
                    supervisor.accept(
                        {
                            "session": supervisor.policy_session_id,
                            "seq": 0,
                            "state_id": 1,
                            "q": [0.0],
                            "dq": [0.0],
                            "kp": [1.0],
                            "kd": [1.0],
                            "tau": [0.0],
                        },
                        0.0,
                    )
                if phase == "STOP_LATCHED":
                    supervisor.stop("operator_stop", 0.0)
                for pressed, timestamp in ((True, 0.01), (False, 0.02), (True, 0.03)):
                    supervisor.state(observation, pressed, timestamp)
                self.assertEqual(supervisor.supervisor_phase, "STOP_LATCHED")
                self.assertEqual(supervisor.cycle(0.03)["kind"], "damping")
                self.assertIn("error", supervisor.control("reset", 0.04))
                supervisor.state(observation, False, 0.05)
                self.assertNotIn("error", supervisor.control("reset", 0.06))
                supervisor.state(observation, False, 0.07)
                supervisor.state(observation, True, 0.08)
                self.assertEqual(supervisor.supervisor_phase, "INHIBITED")


if __name__ == "__main__":
    unittest.main()
