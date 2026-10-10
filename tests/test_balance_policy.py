import unittest

import numpy as np

from robot_safety.balance_policy import (
    NUM_ACTIONS,
    NUM_JOINTS,
    SINGLE_OBS_DIM,
    gravity_orientation,
    single_observation,
    resolve_model_path,
)


class BalancePolicyTests(unittest.TestCase):
    def test_identity_quaternion_points_gravity_down(self):
        gravity = gravity_orientation([1.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(gravity, [0.0, 0.0, -1.0], atol=1e-6)

    def test_observation_matches_gear_layout(self):
        q = np.zeros(NUM_JOINTS, dtype=np.float32)
        dq = np.zeros(NUM_JOINTS, dtype=np.float32)
        action = np.zeros(NUM_ACTIONS, dtype=np.float32)
        obs = single_observation(q, dq, [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], action)
        self.assertEqual(obs.shape, (SINGLE_OBS_DIM,))
        self.assertAlmostEqual(float(obs[3]), 0.74, places=5)
        np.testing.assert_allclose(obs[10:13], [0.0, 0.0, -1.0], atol=1e-6)
        self.assertAlmostEqual(float(obs[13]), 0.1)
        self.assertAlmostEqual(float(obs[16]), -0.3)

    def test_network_output_width(self):
        model = resolve_model_path()
        self.assertTrue(model.is_file(), f"balance weights missing at {model}")
        from robot_safety.balance_policy import BalancePolicy

        policy = BalancePolicy(str(model))
        policy.warmup()
        np.testing.assert_array_equal(policy.action, np.zeros(NUM_ACTIONS))
        for frame in policy.history:
            np.testing.assert_array_equal(frame, np.zeros(SINGLE_OBS_DIM))
        targets = policy.update(
            np.zeros(NUM_JOINTS),
            np.zeros(NUM_JOINTS),
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        self.assertEqual(targets.shape, (NUM_ACTIONS,))
        self.assertTrue(np.all(np.isfinite(targets)))


if __name__ == "__main__":
    unittest.main()
