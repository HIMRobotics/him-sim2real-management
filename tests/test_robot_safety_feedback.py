"""Feedback mapping, button edges and loss detection without SDK or ROS."""

import unittest
from types import SimpleNamespace

from robot_safety.feedback import FeedbackInbox, feedback_from_sdk


def sdk_state(pressed=False):
    remote = [0] * 40
    remote[2] = 4 if pressed else 0
    return SimpleNamespace(
        motor_state=[SimpleNamespace(q=float(i), dq=-float(i)) for i in range(35)],
        wireless_remote=remote,
    )


class FeedbackTests(unittest.TestCase):
    def test_mapping_copies_only_body_joints(self):
        message = sdk_state(True)
        sample = feedback_from_sdk(message, 10.0)
        message.motor_state[15].q = 999.0
        self.assertEqual(len(sample.q), 29)
        self.assertEqual(sample.q[15], 15.0)
        self.assertEqual(sample.dq[28], -28.0)
        self.assertTrue(sample.start_button_pressed)
        self.assertEqual(sample.received_at_secs, 10.0)

    def test_queue_preserves_press_release_press(self):
        inbox = FeedbackInbox(4)
        for index, pressed in enumerate((False, True, False, True)):
            inbox.receive(sdk_state(pressed), float(index))
        self.assertEqual(
            [inbox.take().start_button_pressed for _ in range(4)],
            [False, True, False, True],
        )
        self.assertIsNone(inbox.take())

    def test_overflow_latches_even_after_queue_drained(self):
        inbox = FeedbackInbox(1)
        inbox.receive(sdk_state(), 1.0)
        inbox.receive(sdk_state(True), 2.0)
        self.assertTrue(inbox.input_fault.is_set())
        inbox.take()
        inbox.receive(sdk_state(), 3.0)
        self.assertIsNone(inbox.take())

    def test_bad_telemetry_latches_fault(self):
        for fault in ("short", "nan", "remote", "clock"):
            with self.subTest(fault=fault):
                inbox = FeedbackInbox(2)
                message = sdk_state()
                if fault == "short":
                    message.motor_state = message.motor_state[:28]
                elif fault == "nan":
                    message.motor_state[5].dq = float("nan")
                elif fault == "remote":
                    message.wireless_remote = []
                inbox.receive(message, float("nan") if fault == "clock" else 1.0)
                self.assertTrue(inbox.input_fault.is_set())
                self.assertIsNone(inbox.take())
