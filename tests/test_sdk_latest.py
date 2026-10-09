"""Newest-sample reads replace the 1 kHz lowstate callback."""

import threading
import time
import unittest
from types import SimpleNamespace

from robot_safety.sdk_latest import LatestSdkPoller, LatestSdkReader, poll_deadline


class FakeReader:
    def __init__(self, batches):
        self.batches = list(batches)
        self.calls = []

    def take(self, N=1):
        self.calls.append(N)
        return self.batches.pop(0) if self.batches else []


class LatestSdkReaderTests(unittest.TestCase):
    def test_returns_the_newest_sample_or_none(self):
        first, second = SimpleNamespace(tick=1), SimpleNamespace(tick=2)
        reader = LatestSdkReader("rt/lowstate", None, depth=2,
                                 reader=FakeReader([[first, second], []]))
        self.assertIs(reader.take_latest(), second)
        self.assertIsNone(reader.take_latest())
        self.assertEqual(reader.reader.calls, [2, 2])

    def test_invalid_samples_are_skipped(self):
        try:
            from cyclonedds.internal import InvalidSample
        except ImportError:
            self.skipTest("cyclonedds is only installed for the robot Python")
        valid = SimpleNamespace(tick=1)
        invalid = InvalidSample.__new__(InvalidSample)
        reader = LatestSdkReader("rt/lowstate", None, depth=2,
                                 reader=FakeReader([[valid, invalid]]))
        self.assertIs(reader.take_latest(), valid)

    def test_late_deadline_waits_a_full_period(self):
        self.assertEqual(poll_deadline(1.0, 0.996, 0.005)[1], 1.0)
        delay, following = poll_deadline(1.0, 1.003, 0.005)
        self.assertEqual(delay, 0.005)
        self.assertAlmostEqual(following, 1.008)

    def test_poller_hands_over_samples_until_stopped(self):
        seen = []
        done = threading.Event()

        class Reader:
            topic = "rt/lowstate"

            def __init__(self):
                self.count = 0

            def take_latest(self):
                self.count += 1
                if self.count > 3:
                    done.set()
                    return None
                return self.count

        poller = LatestSdkPoller(Reader(), seen.append, 0.001)
        poller.start()
        self.assertTrue(done.wait(1.0))
        poller.stop()
        time.sleep(0.01)
        self.assertEqual(seen, [1, 2, 3])
        self.assertFalse(poller.thread.is_alive())


if __name__ == "__main__":
    unittest.main()
