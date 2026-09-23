"""Check arm trajectory endpoints, velocity and unchanged body joints."""

import math
import unittest

from tests.manual.arm_raise import ArmRaiseSettings, SlowArmRaise


class ArmRaiseTests(unittest.TestCase):
    def setUp(self):
        self.start = [0.2] * 29
        self.trajectory = SlowArmRaise(self.start, ArmRaiseSettings())

    def test_endpoints_start_at_measured_pose_and_finish_with_zero_speed(self):
        first = self.trajectory.command_at(0.0)
        last = self.trajectory.command_at(10.0)
        self.assertEqual(first["q"], self.start)
        self.assertEqual(first["dq"], [0.0] * 29)
        self.assertAlmostEqual(last["q"][16], 0.35)
        self.assertEqual(last["dq"], [0.0] * 29)
        for index in range(29):
            if index != 16:
                self.assertEqual(last["q"][index], self.start[index])

    def test_slow_monotone_raise_and_velocity_matches_position_derivative(self):
        previous = self.start[16]
        for step in range(1, 100):
            elapsed = step / 10
            command = self.trajectory.command_at(elapsed)
            self.assertGreaterEqual(command["q"][16], previous)
            self.assertLessEqual(command["dq"][16], 0.028125 + 1e-12)
            delta = 1e-5
            derivative = (
                self.trajectory.command_at(elapsed + delta)["q"][16]
                - self.trajectory.command_at(elapsed - delta)["q"][16]
            ) / (2 * delta)
            self.assertAlmostEqual(command["dq"][16], derivative, places=8)
            previous = command["q"][16]

    def test_invalid_configuration_and_measurements_rejected(self):
        for value in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                ArmRaiseSettings(duration_secs=value)
        with self.assertRaises(ValueError):
            SlowArmRaise([0.0] * 28, ArmRaiseSettings())
        with self.assertRaises(ValueError):
            self.trajectory.command_at(-0.1)

    def test_parameter_overrides_cannot_make_the_test_faster_or_larger(self):
        with self.assertRaises(ValueError):
            ArmRaiseSettings(duration_secs=1.0)
        with self.assertRaises(ValueError):
            ArmRaiseSettings(angle_rad=0.16)
        ArmRaiseSettings(angle_rad=0.1, duration_secs=20.0)

    def test_acceleration_and_command_steps_remain_small(self):
        duration = self.trajectory.settings.duration_secs
        angle = self.trajectory.settings.angle_rad
        max_acceleration = angle * (10 / math.sqrt(3)) / duration**2
        max_speed = angle * 1.875 / duration
        delta_secs = 1e-4
        for step in range(1, 2000):
            elapsed = step * 0.005
            before = self.trajectory.command_at(elapsed - delta_secs)["dq"][16]
            after = self.trajectory.command_at(elapsed + delta_secs)["dq"][16]
            self.assertLessEqual(
                abs((after - before) / (2 * delta_secs)), max_acceleration + 1e-9
            )
            for gap_secs in (0.005, 0.02):
                displacement = (
                    self.trajectory.command_at(elapsed + gap_secs)["q"][16]
                    - self.trajectory.command_at(elapsed)["q"][16]
                )
                self.assertLessEqual(displacement, max_speed * gap_secs + 1e-12)
        for endpoint in (0.0, duration):
            before_time = max(endpoint - delta_secs, 0.0)
            after_time = min(endpoint + delta_secs, duration)
            endpoint_acceleration = (
                self.trajectory.command_at(after_time)["dq"][16]
                - self.trajectory.command_at(before_time)["dq"][16]
            ) / delta_secs
            self.assertAlmostEqual(endpoint_acceleration, 0.0, places=6)
