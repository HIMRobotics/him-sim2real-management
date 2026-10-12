"""Measured trial completion without inference, ROS, or motor output."""

import math
import unittest

from containers.examples.trial import MovementTrial


class TrialTests(unittest.TestCase):
    def trial(self, phase="supported_straight"):
        return MovementTrial({"phase": phase, "qualified": True, "ramp_secs": 1.,
                              "braking_distance_m": 0.1, "settle_secs": 1., "stationary_speed_m_s": 0.02})

    def state(self, x=0., y=0.):
        return {"position": [x, y, 0.8], "base_velocity": [0.] * 6, "quat": [1., 0., 0., 0.]}

    def test_straight_uses_measured_progress_then_stays_standing(self):
        trial = self.trial()
        trial.command(self.state(), 0.)
        trial.command(self.state(1.9), 19.)
        trial.command(self.state(2.), 20.)
        self.assertEqual(trial.command(self.state(2.), 21.), [0., 0., 0.])
        self.assertTrue(trial.result["success"])
        self.assertEqual(trial.command(self.state(2.), 50.), [0., 0., 0.])

    def test_ramp_clock_starts_at_handoff(self):
        trial = self.trial()
        sample = dict(self.state(), policy_handoff_at_secs=10.0)
        self.assertAlmostEqual(trial.command(sample, 10.5)[0], 0.05)
        self.assertEqual(trial.started_at, 10.0)

    def test_no_measured_motion_times_out(self):
        trial = self.trial()
        trial.command(self.state(), 0.)
        with self.assertRaisesRegex(RuntimeError, "trial_timeout"):
            trial.command(self.state(), 40.)
        self.assertFalse(trial.result["success"])

    def test_steering_uses_start_frame_and_ramps_to_zero(self):
        settings = dict(self.trial().settings, speed_m_s=0.45, heading_gain=2., lateral_gain=1.)
        trial = MovementTrial(settings)
        state = self.state()
        state["quat"] = [math.sqrt(0.5), 0., 0., math.sqrt(0.5)]
        trial.command(state, 0.)
        state["position"] = [-0.1, 1., 0.8]
        self.assertAlmostEqual(trial.command(state, 1.)[0], 0.45)
        self.assertAlmostEqual(trial.command(state, 1.)[2], -0.1)
        state["quat"] = [0., 0., 0., 1.]
        self.assertEqual(trial.command(state, 2.)[2], -0.3)
        state["position"] = [0., 2., 0.8]
        trial.command(state, 3.)
        self.assertEqual(trial.command(state, 4.), [0., 0., 0.])

    def test_circle_requires_a_full_measured_lap(self):
        trial = self.trial("circle")
        trial.command(self.state(), 0.)
        self.assertIsNone(trial.result)
        for i in range(1, 65):
            angle = 2 * math.pi * i / 64
            trial.command(self.state(math.sin(angle), 1 - math.cos(angle)), float(i))
        trial.command(self.state(), 66.)
        trial.command(self.state(), 67.)
        self.assertTrue(trial.result["success"])
        self.assertGreaterEqual(trial.result["angular_progress_rad"], 2 * math.pi)

    def test_lateral_excursion_cannot_be_hidden_by_good_endpoint(self):
        trial = self.trial()
        trial.command(self.state(), 0.)
        trial.command(self.state(1., 0.3), 10.)
        trial.command(self.state(2.), 20.)
        trial.command(self.state(2.), 21.)
        trial.command(self.state(2.), 22.)
        self.assertFalse(trial.result["success"])
