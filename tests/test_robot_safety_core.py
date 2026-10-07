"""Deterministic tests without SDK imports or robot communication."""

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_safety.config import JointCommandLimits, ThrashConfig
from robot_safety.core import Config, Supervisor
from robot_safety.thrash import ThrashDetector


class SupervisorTests(unittest.TestCase):
    def test_custom_command_limits_are_enforced(self):
        limits = replace(JointCommandLimits(), position_rad=(-0.1, 0.1))
        supervisor = self.make(command_limits=limits)
        message = self.arm(supervisor)
        self.assertTrue(supervisor.accept(message, 0.01))
        message["seq"] = 1
        message["q"][0] = 0.100001
        self.assertFalse(supervisor.accept(message, 0.02))
        self.assertEqual(supervisor.stop_reason, "invalid_action")
        detail = supervisor.status(0.02)["fault_detail"]
        self.assertEqual(detail["joint_index"], 0)
        self.assertEqual(detail["field"], "q")
        self.assertEqual(detail["value"], 0.100001)
        self.assertEqual(detail["bounds"], [-0.1, 0.1])
        supervisor.stop("later_fault", 0.03, {"joint_index": 1})
        self.assertEqual(supervisor.status(0.03)["fault_detail"], detail)
        supervisor.control("reset", 0.04)
        self.assertIsNone(supervisor.stop_detail)

    def test_invalid_limit_configuration_is_rejected(self):
        for bounds in ((1.0, -1.0), (0.0, float("inf")), (False, 1.0), [0.0, 1.0]):
            with self.subTest(bounds=bounds), self.assertRaises(ValueError):
                JointCommandLimits(position_rad=bounds)
        with self.assertRaises(ValueError):
            JointCommandLimits(stiffness_nm_per_rad=(-1.0, 1.0))
        with self.assertRaises(ValueError):
            Config(observation_history_size=0)

    def test_configured_observation_limits_and_history(self):
        supervisor = self.make(
            observation_history_size=1, robot_position_bounds_rad=(-1.0, 1.0)
        )
        message = self.arm(supervisor)
        supervisor.state({"q": [0.0, 0.0], "dq": [0.0, 0.0]}, False, 0.01)
        self.assertFalse(supervisor.accept(message, 0.02))
        self.assertEqual(len(supervisor.observation_received_times_secs), 1)

        supervisor = self.make(robot_position_bounds_rad=(-1.0, 1.0))
        supervisor.state({"q": [2.0, 0.0], "dq": [0.0, 0.0]}, False, 0.01)
        self.assertEqual(supervisor.stop_reason, "invalid_robot_state")

    def make(self, **overrides):
        config = replace(
            Config(
                joints=2,
                action_timeout=10.0,
                observation_timeout=10.0,
                robot_timeout=10.0,
                loop_timeout=10.0,
            ),
            **overrides,
        )
        supervisor = Supervisor(config)
        supervisor.state({"q": [0.0, 0.0], "dq": [0.0, 0.0]}, False, 0.0)
        supervisor.cycle(0.0)
        return supervisor

    def make_policy_command(self, supervisor, seq=0):
        return {
            "session": supervisor.policy_session_id,
            "seq": seq,
            "state_id": supervisor.latest_observation_id,
            "q": [0.1, -0.1],
            "dq": [0.0, 0.0],
            "kp": [5.0, 5.0],
            "kd": [1.0, 1.0],
            "tau": [0.0, 0.0],
        }

    def arm(self, supervisor):
        self.assertNotIn("error", supervisor.control("arm", 0.0))
        return self.make_policy_command(supervisor)

    def test_startup_and_restart_are_inhibited(self):
        first, restarted = self.make(), self.make()
        self.assertEqual(first.supervisor_phase, "INHIBITED")
        self.assertEqual(restarted.supervisor_phase, "INHIBITED")
        self.assertIsNone(restarted.policy_session_id)
        self.assertNotEqual(
            first.supervisor_instance_id, restarted.supervisor_instance_id
        )
        status = first.status(0.0)
        self.assertTrue(status["dry_run"])
        self.assertFalse(status["motor_publisher"])
        self.assertFalse(status["can_enforce_motor_damping"])
        self.assertIsNone(status["motor_application_latency"])
        self.assertIsNone(status["controller_radio_live"])

    def test_arm_alone_never_selects_policy(self):
        supervisor = self.make()
        self.arm(supervisor)
        self.assertEqual(supervisor.cycle(0.01)["kind"], "damping")
        self.assertEqual(supervisor.supervisor_phase, "ARMED")

    def test_watchdog_boundaries(self):
        cases = (
            ("action_timeout", "action_timeout", False),
            ("robot_timeout", "robot_state_timeout", False),
            ("observation_timeout", "observation_timeout", True),
            ("loop_timeout", "loop_progress", False),
        )
        for setting, reason, running in cases:
            for cur_time_secs in (0.199999, 0.2, 0.200001):
                with self.subTest(setting=setting, cur_time_secs=cur_time_secs):
                    supervisor = self.make(**{setting: 0.2})
                    message = self.arm(supervisor)
                    if running:
                        self.assertTrue(supervisor.accept(message, 0.0))
                    output = supervisor.cycle(cur_time_secs)
                    if cur_time_secs < 0.2:
                        self.assertNotEqual(supervisor.supervisor_phase, "STOP_LATCHED")
                    else:
                        self.assertEqual(supervisor.supervisor_phase, "STOP_LATCHED")
                        self.assertEqual(supervisor.stop_reason, reason)
                        self.assertEqual(output["kind"], "damping")

    def test_late_state_cannot_hide_timeout(self):
        supervisor = self.make(robot_timeout=0.2)
        self.arm(supervisor)
        supervisor.state({"q": [0.0, 0.0], "dq": [0.0, 0.0]}, False, 0.2)
        self.assertEqual(supervisor.stop_reason, "robot_state_timeout")

    def test_late_action_cannot_hide_timeout(self):
        supervisor = self.make(action_timeout=0.2)
        message = self.arm(supervisor)
        self.assertFalse(supervisor.accept(message, 0.2))
        self.assertEqual(supervisor.stop_reason, "action_timeout")

    def test_stop_in_every_phase_and_held_reset(self):
        for phase in ("INHIBITED", "ARMED", "RUNNING", "STOP_LATCHED"):
            with self.subTest(phase=phase):
                supervisor = self.make()
                if phase in ("ARMED", "RUNNING"):
                    message = self.arm(supervisor)
                    if phase == "RUNNING":
                        supervisor.accept(message, 0.0)
                elif phase == "STOP_LATCHED":
                    supervisor.stop("operator_stop", 0.0)
                supervisor.state(
                    {"q": [0.0, 0.0], "dq": [0.0, 0.0]}, True, 0.01, stop_requested=True
                )
                self.assertEqual(supervisor.supervisor_phase, "STOP_LATCHED")
                self.assertIsNone(supervisor.policy_session_id)
                self.assertIsNone(supervisor.accepted_policy_command)
                self.assertIn("error", supervisor.control("reset", 0.02))
                self.assertIn("error", supervisor.control("arm", 0.02))
                output = supervisor.cycle(0.02)
                self.assertEqual(output["kind"], "damping")
                for key in ("kp", "dq", "tau"):
                    self.assertEqual(output[key], [0.0, 0.0])

    def test_same_cycle_stop_wins_over_reset_and_arm(self):
        supervisor = self.make()
        supervisor.control("stop", 0.01)
        self.assertIn("error", supervisor.control("reset", 0.01))
        self.assertIn("error", supervisor.control("arm", 0.01))
        self.assertEqual(supervisor.supervisor_phase, "STOP_LATCHED")

    def test_reset_requires_separate_arm_and_revokes_session(self):
        supervisor = self.make()
        old = self.arm(supervisor)
        supervisor.accept(old, 0.0)
        supervisor.control("stop", 0.01)
        supervisor.control("reset", 0.02)
        self.assertEqual(supervisor.supervisor_phase, "INHIBITED")
        self.assertIsNone(supervisor.policy_session_id)
        self.assertFalse(supervisor.accept(old, 0.03))
        supervisor.control("arm", 0.04)
        self.assertNotEqual(supervisor.policy_session_id, old["session"])
        self.assertFalse(supervisor.accept(old, 0.05))
        self.assertEqual(supervisor.cycle(0.05)["kind"], "damping")

    def test_invalid_actions_latch_without_refreshing_deadline(self):
        mutations = (
            {"seq": -1},
            {"seq": True},
            {"session": "unknown"},
            {"state_id": 999},
            {"state_id": []},
            {"q": [float("nan"), 0.0]},
            {"dq": [float("inf"), 0.0]},
            {"kp": [-1.0, 0.0]},
            {"kd": [51.0, 0.0]},
            {"tau": [201.0, 0.0]},
            {"q": [0.0]},
        )
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                supervisor = self.make()
                message = self.arm(supervisor)
                message.update(mutation)
                self.assertFalse(supervisor.accept(message, 0.01))
                self.assertEqual(supervisor.time_action_watchdog_refreshed_secs, 0.0)
                self.assertEqual(supervisor.supervisor_phase, "STOP_LATCHED")

    def test_duplicate_sequence_and_stale_observation(self):
        supervisor = self.make()
        message = self.arm(supervisor)
        self.assertTrue(supervisor.accept(message, 0.01))
        self.assertFalse(supervisor.accept(message, 0.02))
        self.assertEqual(supervisor.time_action_watchdog_refreshed_secs, 0.01)

        supervisor = self.make(observation_timeout=0.2)
        message = self.arm(supervisor)
        self.assertFalse(supervisor.accept(message, 0.2))
        self.assertEqual(supervisor.supervisor_phase, "STOP_LATCHED")

    def test_latest_valid_action_selected_and_latch_persists(self):
        supervisor = self.make()
        message = self.arm(supervisor)
        supervisor.accept(message, 0.01)
        newer = self.make_policy_command(supervisor, seq=1)
        newer["q"] = [0.2, -0.2]
        supervisor.accept(newer, 0.02)
        self.assertEqual(supervisor.cycle(0.02)["q"], newer["q"])
        supervisor.stop("policy_disconnected", 0.03)
        for seq in range(2, 100):
            newer["seq"] = seq
            self.assertFalse(supervisor.accept(newer, 0.04))
            self.assertEqual(supervisor.cycle(0.04)["kind"], "damping")

    def test_status_cannot_mutate_internal_state(self):
        supervisor = self.make()
        status = supervisor.status(0.0)
        status["state"]["q"][0] = 99.0
        status["output"]["kd"][0] = 99.0
        self.assertEqual(supervisor.latest_robot_observation["q"][0], 0.0)
        self.assertEqual(supervisor.last_selected_command["kd"][0], 1.0)


class ThrashTests(unittest.TestCase):
    def detector(self, joints=1, **settings):
        return ThrashDetector(ThrashConfig(**settings), joints, 0.2)

    def sine(self, detector, amplitude, frequency, duration=3.0, dt=0.005):
        detections = []
        for i in range(round(duration / dt) + 1):
            now = i * dt
            velocity = amplitude * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * now)
            reason = detector.update([velocity] * detector.joints, now)
            if reason:
                detections.append((now, reason))
        return detections

    def test_regular_single_joint_instability_still_triggers(self):
        detector = self.detector()
        detections = self.sine(detector, 0.12, 8)
        self.assertTrue(detections)
        self.assertEqual(detections[0][1], "thrashing_severe")
        self.assertLess(detections[0][0], 1.0)
        self.assertLess(detector.metrics["irregularity_cv"][0], 0.01)

    def test_local_and_distributed_detection(self):
        for regions, reason in (
            ((0, 0), "thrashing_local"),
            ((0, 1, 2), "thrashing_distributed"),
            ((0, 1), None),
        ):
            with self.subTest(regions=regions):
                detections = self.sine(self.detector(len(regions), regions=regions), 0.07, 8)
                self.assertEqual(detections[0][1] if detections else None, reason)

    def test_regular_motion_inside_configured_envelope(self):
        detector = self.detector(29, velocity_scales=(2.0,) * 29)
        self.assertFalse(self.sine(detector, 0.4, 2))

    def test_scaling_changes_the_envelope(self):
        self.assertFalse(self.sine(self.detector(velocity_scales=(10.0,)), 0.12, 8))
        self.assertTrue(self.sine(self.detector(velocity_scales=(1.0,)), 0.12, 8))

    def test_irregular_motion_below_filtered_threshold(self):
        periods = [0.3, 0.7, 0.4, 0.6, 0.25, 0.75]
        for joints, speed, reason in ((2, 3, "thrashing_local"), (1, 4, "thrashing_severe")):
            detector = self.detector(joints, regions=(0,) * joints, activity_rms=1.5)
            detected = False
            for i in range(601):
                now = i * 0.005
                phase = now
                for period in periods:
                    if phase < period:
                        break
                    phase -= period
                velocity = speed * math.sin(2 * math.pi * phase / period)
                result = detector.update([velocity] * joints, now)
                if result:
                    self.assertEqual(result, reason)
                    self.assertLess(max(detector.metrics["oscillatory_rms"]), 1.5)
                    self.assertGreater(max(detector.metrics["irregularity_cv"]), 0.35)
                    detected = True
                    break
            self.assertTrue(detected)

    def test_asymmetric_repeated_strokes_are_not_irregular(self):
        detector = self.detector(2, regions=(0, 0), slow_cutoff_hz=10.0)
        for i in range(1201):
            now = i * 0.005
            phase = now % 0.5
            velocity = (
                2 * math.sin(math.pi * phase / 0.15) if phase < 0.15
                else -2 * 0.15 / 0.35 * math.sin(math.pi * (phase - 0.15) / 0.35)
            )
            self.assertIsNone(detector.update([velocity] * 2, now))
        self.assertLess(max(detector.metrics["irregularity_cv"]), 0.01)

    def test_noise_single_impulse_and_short_burst_do_not_trigger(self):
        signals = (
            lambda t: 0.03 * math.sin(130 * t),
            lambda t: 15.0 if 0.1 <= t < 0.13 else 0.0,
            lambda t: 8 * math.sin(2 * math.pi * 8 * t) if t < 0.15 else 0.0,
        )
        for signal in signals:
            detector = self.detector(29)
            for i in range(401):
                now = i * 0.005
                self.assertIsNone(detector.update([signal(now)] * 29, now))

    def test_evidence_decays_and_gaps_clear_history(self):
        detector = self.detector()
        self.assertTrue(self.sine(detector, 0.12, 8))
        for i in range(1, 601):
            detector.update([0.0], 3.0 + i * 0.005)
        self.assertEqual(detector.evidence, [0.0])
        self.assertEqual(detector.severe_evidence, [0.0])
        self.assertFalse(detector.motion_cycles[0].cycles)
        self.assertIsNone(detector.update([5.0], 7.0))
        self.assertEqual(detector.metrics, {})
        self.assertEqual(detector.evidence, [0.0])

    def test_duplicate_samples_do_not_accumulate_evidence(self):
        detector = self.detector()
        self.sine(detector, 0.12, 8, duration=0.5)
        before = list(detector.evidence)
        for i in range(200):
            detector.update([(-1) ** i * 10.0], 0.5)
        self.assertEqual(detector.evidence, before)

    def test_sample_rate_variation(self):
        for dt in (0.002, 0.005, 0.01):
            with self.subTest(dt=dt):
                self.assertTrue(self.sine(self.detector(), 0.12, 8, dt=dt))

    def test_invalid_settings(self):
        for settings in (
            {"mode": "auto"}, {"min_cycles": 1}, {"min_cycles": 7},
            {"local_joints": 1.5}, {"averaging_secs": float("nan")},
            {"persistence_secs": 0.0}, {"fast_cutoff_hz": 3.0},
            {"severe_rms": 0.5}, {"velocity_scales": (0.0,)},
            {"regions": (-1,)},
        ):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                ThrashConfig(**settings)
        with self.assertRaises(ValueError):
            Config(joints=2, thrash=ThrashConfig(velocity_scales=(1.0,)))

    def test_monitor_enforce_off_and_session_reset(self):
        for mode in ("monitor", "enforce", "off"):
            with self.subTest(mode=mode):
                supervisor = Supervisor(Config(
                    joints=1, thrash=ThrashConfig(mode=mode),
                ))
                supervisor.state({"q": [0.0], "dq": [0.0]}, False, 0.0)
                self.assertNotIn("error", supervisor.control("arm", 0.0))
                for i in range(1, 301):
                    now = i * 0.005
                    phase = 2 * math.pi * 8 * now
                    supervisor.state({
                        "q": [0.12 * math.sin(phase)],
                        "dq": [0.12 * 2 * math.pi * 8 * math.cos(phase)],
                    }, False, now)
                    command = {
                        "session": supervisor.policy_session_id, "seq": i,
                        "state_id": supervisor.latest_observation_id,
                        "q": [0.0], "dq": [0.0], "kp": [1.0], "kd": [1.0], "tau": [0.0],
                    }
                    accepted = supervisor.accept(command, now)
                    output = supervisor.cycle(now)
                    if supervisor.stop_reason:
                        self.assertFalse(accepted)
                        self.assertEqual(output["kind"], "damping")
                        break
                    self.assertTrue(accepted)
                    self.assertEqual(output["kind"], "policy")
                if mode == "enforce":
                    self.assertEqual(supervisor.stop_reason, "thrashing_severe")
                    self.assertFalse(supervisor.accept(command, now + 0.001))
                    self.assertNotIn("error", supervisor.control("reset", now + 0.001))
                    self.assertIsNone(supervisor.thrash_event)
                    self.assertEqual(supervisor.thrash_detector.metrics, {})
                    self.assertNotIn("error", supervisor.control("arm", now + 0.002))
                elif mode == "monitor":
                    self.assertIsNone(supervisor.stop_reason)
                    self.assertEqual(supervisor.thrash_event["reason"], "thrashing_severe")
                    snapshot = supervisor.status(now)
                    snapshot["thrash_event"]["joint_indices"].clear()
                    self.assertEqual(supervisor.thrash_event["joint_indices"], [0])
                else:
                    self.assertIsNone(supervisor.stop_reason)
                    self.assertIsNone(supervisor.thrash_event)


if __name__ == "__main__":
    unittest.main()
