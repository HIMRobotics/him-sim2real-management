"""Dry-run runtime scheduling, queue and stop-priority tests."""

import threading
import unittest

from robot_safety.config import Config
from robot_safety.runtime import DryRunRuntime


class FakeClock:
    """Controllable monotonic clock for deterministic queue tests."""

    def __init__(self):
        self.time_secs = 0.0

    def __call__(self):
        return self.time_secs


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.runtime = DryRunRuntime(Config(joints=1), clock=self.clock)

    def observation(self, pressed=False):
        return self.runtime.submit_observation({"q": [0.0], "dq": [0.0]}, pressed)

    def test_startup_and_snapshot_isolation(self):
        self.observation()
        status = self.runtime.step()
        self.assertEqual(status["phase"], "INHIBITED")
        self.assertFalse(status["motor_publisher"])
        self.assertEqual(status["completed_cycles"], 1)
        status["output"]["kd"][0] = 999
        self.assertEqual(self.runtime.status()["output"]["kd"], [1.0])

    def test_double_press_receipt_times_and_stop_precedence(self):
        self.observation()
        self.runtime.step()
        reset = self.runtime.request_control("reset")
        arm = self.runtime.request_control("arm")
        for timestamp, pressed in (
            (0.01, True),
            (0.02, False),
            (0.03, True),
            (0.04, False),
        ):
            self.clock.time_secs = timestamp
            self.observation(pressed)
        self.clock.time_secs = 0.05
        status = self.runtime.step()
        self.assertEqual(status["phase"], "STOP_LATCHED")
        self.assertEqual(status["fault_reason"], "controller_stop")
        self.assertEqual(status["output"]["kind"], "damping")
        self.assertAlmostEqual(status["robot_state_age"], 0.01)
        self.assertIn("error", reset.result(0))
        self.assertIn("error", arm.result(0))
        self.clock.time_secs = 0.06
        reset = self.runtime.request_control("reset")
        self.runtime.step()
        self.assertEqual(reset.result(0)["phase"], "INHIBITED")

    def test_duplicate_stop_blocks_reset_even_for_existing_latch(self):
        self.observation()
        self.runtime.request_stop()
        self.runtime.step()
        self.clock.time_secs = 0.01
        reset = self.runtime.request_control("reset")
        self.runtime.request_stop()
        self.runtime.step()
        self.assertIn("error", reset.result(0))

    def test_stale_queued_observation_cannot_refresh_robot_state(self):
        self.observation()
        self.clock.time_secs = 0.2
        status = self.runtime.step()
        self.assertEqual(status["fault_reason"], "stale_robot_input")
        self.assertIsNone(status["robot_state_age"])

    def test_overflow_discards_edges_and_requires_fresh_input(self):
        self.runtime = DryRunRuntime(
            Config(joints=1, runtime_queue_capacity=1), clock=self.clock
        )
        self.assertTrue(self.observation())
        self.assertFalse(self.observation(True))
        status = self.runtime.step()
        self.assertEqual(status["fault_reason"], "robot_input_overflow")
        self.assertIsNone(status["state"])
        self.clock.time_secs = 0.01
        reset = self.runtime.request_control("reset")
        self.runtime.step()
        self.assertIn("error", reset.result(0))
        self.clock.time_secs = 0.02
        self.observation()
        reset = self.runtime.request_control("reset")
        self.runtime.step()
        self.assertEqual(reset.result(0)["phase"], "INHIBITED")

    def test_full_control_queue_cannot_block_stop(self):
        self.runtime = DryRunRuntime(
            Config(joints=1, runtime_queue_capacity=1), clock=self.clock
        )
        self.observation()
        arm = self.runtime.request_control("arm")
        rejected = self.runtime.request_control("reset")
        self.assertIn("error", rejected.result(0))
        self.runtime.request_stop()
        self.runtime.step()
        self.assertIn("error", arm.result(0))
        self.assertEqual(self.runtime.status()["phase"], "STOP_LATCHED")

    def test_stalled_loop_latches_and_reports_progress(self):
        self.observation()
        self.runtime.step()
        self.clock.time_secs = 0.2
        self.runtime.step()
        status = self.runtime.status()
        self.assertEqual(status["fault_reason"], "loop_progress")
        self.assertEqual(status["completed_cycles"], 2)
        self.assertEqual(status["time_last_completed_cycle_secs"], 0.2)

    def test_invalid_input_is_a_fault(self):
        self.assertFalse(self.runtime.submit_observation({"q": [float("nan")]}, False))
        self.assertEqual(self.runtime.step()["fault_reason"], "invalid_robot_input")

    def test_scheduler_period_and_shutdown(self):
        clock = self.clock
        waits = []

        class ScheduledShutdown:
            def is_set(self):
                return len(waits) == 3

            def wait(self, delay):
                waits.append(delay)
                clock.time_secs += delay

        self.runtime.run(ScheduledShutdown())
        for delay in waits:
            self.assertAlmostEqual(delay, 0.005)
        self.assertEqual(self.runtime.status()["completed_cycles"], 4)
        self.assertEqual(self.runtime.status()["phase"], "STOP_LATCHED")

    def test_background_loop_can_be_stopped(self):
        runtime = DryRunRuntime(Config(joints=1))
        shutdown = threading.Event()
        thread = threading.Thread(target=runtime.run, args=(shutdown,))
        thread.start()
        try:
            reply = runtime.request_control("status")
            self.assertTrue(reply.result(2)["dry_run"])
        finally:
            shutdown.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(runtime.status()["phase"], "STOP_LATCHED")


if __name__ == "__main__":
    unittest.main()
