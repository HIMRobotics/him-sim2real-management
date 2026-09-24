"""Compare proposed arm targets with the installed Unitree example's formulas."""

import math
import unittest

from tests.manual.unitree_arm_sequence import UnitreeArmSequence, UnitreeArmSettings


class UnitreeSequenceTests(unittest.TestCase):
    def test_matches_example_at_each_motion_stage(self):
        sequence = UnitreeArmSequence()
        measured = [0.2] * 29
        for elapsed in (0.0, 1.5, 3.0, 6.0, 8.98, 9.0, 13.5, 17.98):
            with self.subTest(elapsed=elapsed):
                command = sequence.command_at(elapsed, measured)
                target = [0.0] * 29
                if elapsed < 3:
                    ratio = elapsed / 3
                elif elapsed < 9:
                    ratio = (elapsed - 3) / 6
                    target[16] = target[18] = target[25] = math.pi / 2
                    target[23] = -math.pi / 2
                else:
                    ratio = (elapsed - 9) / 9
                for index in range(12, 29):
                    self.assertAlmostEqual(
                        command["q"][index],
                        (1 - ratio) * measured[index] + ratio * target[index],
                    )
                    self.assertEqual(command["kp"][index], 60.0)
                    self.assertEqual(command["kd"][index], 1.5)
                self.assertEqual(command["q"][:12], measured[:12])
                self.assertEqual(command["kp"][:12], [0.0] * 12)
                self.assertEqual(command["dq"], [0.0] * 29)
                self.assertEqual(command["tau"], [0.0] * 29)

    def test_uses_updated_feedback_instead_of_replaying_initial_pose(self):
        sequence = UnitreeArmSequence()
        self.assertAlmostEqual(sequence.command_at(1.5, [0.4] * 29)["q"][16], 0.2)
        self.assertAlmostEqual(sequence.command_at(1.5, [0.2] * 29)["q"][16], 0.1)

    def test_update_interval_is_shorter_than_stall_threshold(self):
        settings = UnitreeArmSettings()
        self.assertEqual(settings.command_period_secs, 0.02)
        self.assertGreater(settings.max_publish_gap_secs, settings.command_period_secs)
        self.assertLess(settings.max_publish_gap_secs, 0.2)
