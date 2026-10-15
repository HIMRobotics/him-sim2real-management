"""Exercise hardware output with fake SDK calls, never a robot connection."""

import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock, patch

from robot_safety.hardware_adapter import G1HardwareAdapter, whole_body_repeat_wait
from robot_safety.hardware_constants import G1_29_JOINT_NAMES, UPPER_BODY_INDICES
from robot_safety.hardware_profile import HardwareProfile
from robot_safety.unitree_transport import UnitreeArmTransport


def profile_data():
    # Artificial test bounds and gains. These are not a deployable hardware profile.
    return {
        "expected_machine_type": 5,
        "allowed_start_fsm_ids": [4],
        "handover_secs": 2.0,
        "max_publish_gap_secs": 0.02,
        "initial_velocity_tolerance_rad_secs": 0.01,
        "joints": {
            G1_29_JOINT_NAMES[i]: {
                "position_min_rad": -1.0,
                "position_max_rad": 1.0,
                "kp": 2.0,
                "kd": 0.5,
                "max_velocity_rad_secs": 0.1,
                "max_torque_nm": 100.0,
            }
            for i in range(29)
        },
    }


def load_profile(data):
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "profile.json"
        path.write_text(json.dumps(data))
        return HardwareProfile.load(path)


class FakeTransport:
    def __init__(self):
        self.writes = []
        self.body_commands = []
        self.stop_calls = 0
        self.release_calls = 0
        self.restore_calls = 0
        self.mode = 4
        self.write_success = True
        self.rpc_success = True

    def read_ready_fsm(self):
        return 0, self.mode

    def read_fsm(self):
        return 0, self.mode

    def request_damping(self):
        self.stop_calls += 1
        if not self.rpc_success:
            raise TimeoutError()
        self.mode = 1
        return 0

    def write_arm(self, targets, weight):
        self.writes.append((targets, weight))
        return self.write_success

    def release_motion(self, *, announce_soft_reset=False):
        self.release_calls += 1
        return 0

    def restore_motion(self):
        self.restore_calls += 1
        return 0

    def write_body(self, targets, machine):
        self.body_commands.append((targets, machine))
        return self.write_success

    def clear_body(self):
        self.body_cleared = True


class HardwareAdapterTests(unittest.TestCase):
    def test_whole_body_checks_then_releases_writes_and_latches(self):
        self.adapter.whole_body = True
        self.transport.write_body = Mock(return_value=True)
        self.assertTrue(self.adapter.arm())
        self.command["q"] = [0.1] * 29
        self.command["tau"] = [0.01] * 29
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertEqual(self.transport.release_calls, 1)
        self.transport.write_body.assert_called_once()
        self.adapter.step()
        targets, machine = self.transport.write_body.call_args[0]
        self.assertEqual((len(targets), machine), (29, 5))
        self.assertEqual(targets[0]["kp"], 150.)
        self.assertEqual(targets[0]["tau"], 0.0)
        self.assertEqual(len(self.adapter.q_applied), 29)
        recorded = self.adapter.joint_diagnostics_snapshot()[G1_29_JOINT_NAMES[0]]["applied_command"]
        self.assertEqual(recorded, dict(targets[0], written_at_secs=self.now))
        self.adapter.abort("hardware_write_failed")
        with patch.object(self.transport, "restore_motion", return_value=7002):
            self.adapter.stop_once()
        self.assertTrue(self.adapter.balance_released)
        self.assertEqual(self.transport.stop_calls, 0)
        self.assertFalse(self.adapter.arm())
        self.adapter.stop_once()
        self.assertEqual(self.transport.restore_calls, 1)
        self.assertEqual(self.transport.stop_calls, 1)
        self.adapter._release_after_confirmed_damp()
        self.assertEqual(self.adapter.abort_reason, "hardware_write_failed")
        self.assertFalse(self.adapter.arm())
        count = self.transport.write_body.call_count
        self.adapter.step()
        self.assertEqual(self.transport.write_body.call_count, count)

    def test_slow_motion_release_is_not_an_output_stall(self):
        self.adapter.whole_body = True
        self.transport.write_body = Mock(return_value=True)

        def slow_release():
            self.now += 0.05
            self.transport.release_calls += 1
            return 0

        self.transport.release_motion = slow_release
        self.assertTrue(self.adapter.arm())
        self.adapter.submit(self.command, self.now + 0.15)
        self.adapter.step()
        self.assertIsNone(self.adapter.abort_reason)
        self.assertTrue(self.adapter.balance_released)
        self.state()
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertIsNone(self.adapter.abort_reason)
        self.assertEqual(self.transport.write_body.call_count, 2)

    def test_whole_body_confirmed_damp_does_not_repeat(self):
        self.adapter.whole_body = True
        self.transport.write_body = Mock(return_value=True)
        self.assertTrue(self.adapter.arm())
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.adapter.step()
        self.adapter.abort("hardware_output_worker_stalled")
        self.adapter._request_stock_damping()
        self.assertEqual(self.adapter.reported_fsm_id, 1)
        self.assertFalse(self.adapter.stop_requested.is_set())
        self.assertFalse(self.adapter.ownership_taken)
        self.assertEqual(self.adapter.abort_reason, "hardware_output_worker_stalled")
        calls = self.transport.stop_calls
        self.adapter.stop_once()
        self.assertEqual(self.transport.stop_calls, calls)

    def test_failed_release_restores_stock_control_without_damping(self):
        self.adapter.whole_body = True
        self.transport.write_body = Mock(return_value=True)
        self.transport.release_motion = lambda: 3104
        self.assertTrue(self.adapter.arm())
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertEqual(self.adapter.abort_reason, "hardware_release_failed")
        self.assertEqual(self.transport.restore_calls, 1)
        self.assertEqual(self.transport.stop_calls, 0)
        self.assertFalse(self.adapter.balance_released)
        self.adapter.stop_once()
        self.assertEqual(self.transport.stop_calls, 0)

    def test_startup_requires_stability_then_new_left_and_b(self):
        import numpy as np

        self.adapter.whole_body = True
        self.adapter.balance_policy = Mock(update=Mock(return_value=np.zeros(15)))
        self.transport.write_body = Mock(return_value=True)
        self.assertTrue(self.adapter.arm())
        self.command["q"] = [0.4] * 29

        def step():
            self.adapter.submit(self.command, self.now + 0.15)
            self.adapter.step()

        def buttons(pressed):
            remote = [0] * 40
            remote[3] = ((1 << 7) | (1 << 1)) if pressed else 0
            self.adapter.receive_state(SimpleNamespace(
                motor_state=[SimpleNamespace(q=0.0, dq=0.0, temperature=(25, 25), tau_est=0.0) for _ in range(35)],
                mode_machine=5,
                imu_state=SimpleNamespace(quaternion=(1., 0., 0., 0.), gyroscope=(0., 0., 0.)),
                wireless_remote=remote,
            ))

        step()
        step()
        buttons(True)
        self.assertFalse(self.adapter.policy_launched)
        for _ in range(220):
            self.now += 0.01
            self.adapter.receive_battery(SimpleNamespace(soc=100))
            buttons(True)
            step()
        self.assertTrue(self.adapter.startup.ready)
        self.assertFalse(self.adapter.policy_launched)
        self.assertEqual(self.transport.write_body.call_args[0][0][0]["q"], 0.)
        buttons(False)
        buttons(True)
        step()
        self.assertTrue(self.adapter.policy_launched)
        self.assertEqual(self.transport.write_body.call_args[0][0][0]["q"], 0.)
        for _ in range(110):
            self.now += 0.01
            self.adapter.receive_battery(SimpleNamespace(soc=100))
            buttons(False)
            step()
        self.assertAlmostEqual(self.transport.write_body.call_args[0][0][0]["q"], 0.4)
        self.assertIsNone(self.adapter.abort_reason)

    def test_bad_whole_body_command_never_releases_stock_control(self):
        self.adapter.whole_body = True
        self.assertTrue(self.adapter.arm())
        self.command["q"][0] = float("nan")
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertEqual(self.transport.release_calls, 0)
        self.assertIsNotNone(self.adapter.abort_reason)

    def setUp(self):
        self.now = 10.0
        self.adapter = G1HardwareAdapter(load_profile(profile_data()), lambda: self.now)
        self.adapter.soft_reset_policy = Mock(
            update=Mock(return_value=[0.0] * 29), kp=[40.0] * 29, kd=[1.0] * 29,
        )
        self.transport = FakeTransport()
        self.adapter.transport = self.transport
        self.adapter.receive_battery(SimpleNamespace(soc=100))
        self.state()
        self.assertTrue(self.adapter.check_ready_mode())
        self.command = {name: [0.0] * 29 for name in ("q", "dq", "kp", "kd", "tau")}

    def state(self, pressed=False, f3=False, machine=5):
        remote = [0] * 40
        remote[2] = (4 if pressed else 0) | (128 if f3 else 0)
        self.adapter.receive_state(
            SimpleNamespace(
                motor_state=[SimpleNamespace(q=0.0, dq=0.0, temperature=(25, 25), tau_est=0.0) for _ in range(35)],
                mode_machine=machine,
                imu_state=SimpleNamespace(quaternion=(1., 0., 0., 0.), gyroscope=(0., 0., 0.)),
                wireless_remote=remote,
            )
        )

    def run_first(self):
        self.assertTrue(self.adapter.check_ready_mode())
        self.assertTrue(self.adapter.arm())
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()

    def test_holosoma_can_arm_from_damping(self):
        self.transport.mode = 1
        self.assertFalse(self.adapter.check_ready_mode())
        self.adapter.startup_policy_name = "holosoma"
        self.assertTrue(self.adapter.check_ready_mode())

    def test_holosoma_rejects_when_the_mode_service_does_not_answer(self):
        self.transport.read_ready_fsm = lambda: (3102, None)
        self.assertFalse(self.adapter.check_ready_mode())
        self.adapter.startup_policy_name = "holosoma"
        self.assertFalse(self.adapter.check_ready_mode())
        self.assertFalse(self.adapter.arm())
        self.assertEqual(self.transport.writes, [])

    def test_only_confirmed_ready_mode_can_arm(self):
        self.transport.mode = 500
        self.assertFalse(self.adapter.check_ready_mode())
        self.assertFalse(self.adapter.arm())
        self.transport.mode = 4
        self.assertTrue(self.adapter.check_ready_mode())
        self.assertTrue(self.adapter.arm())
        self.assertEqual(self.transport.writes, [])

    def test_readiness_failure_reports_measurement_and_limit(self):
        cases = (
            ("mode", "ready mode: measured=500"),
            ("machine", "machine type: measured=6"),
            ("feedback", "feedback age:"),
            ("button", "Start and F3 must be released"),
            ("position", "left_shoulder_roll position: measured=2.000000 rad"),
            ("speed", "left_shoulder_roll speed: measured=0.020000 rad/s"),
            ("rpc", "ready-mode query failed: rpc_code=3104"),
        )
        for case, expected in cases:
            with self.subTest(case=case):
                self.setUp()
                if case == "mode":
                    self.transport.mode = 500
                elif case == "machine":
                    self.state(machine=6)
                elif case == "feedback":
                    self.now += 0.3
                elif case == "button":
                    self.state(pressed=True)
                elif case in ("position", "speed"):
                    field = "q" if case == "position" else "dq"
                    values = list(getattr(self.adapter.feedback, field))
                    values[16] = 2.0 if case == "position" else 0.02
                    self.adapter.feedback = replace(
                        self.adapter.feedback, **{field: tuple(values)}
                    )
                else:
                    self.transport.read_ready_fsm = lambda: (3104, None)
                if case in ("mode", "rpc"):
                    self.assertFalse(self.adapter.check_ready_mode())
                else:
                    self.assertTrue(self.adapter.check_ready_mode())
                    self.assertFalse(self.adapter.arm())
                self.assertIn(expected, self.adapter.arm_rejection_reason)
                if case == "speed":
                    self.assertIn(
                        "maximum absolute=0.01 rad/s", self.adapter.arm_rejection_reason
                    )
                self.assertEqual(self.transport.writes, [])
                # Rejecting startup does not take control of the stock robot.
                self.adapter.abort("hardware_arm_rejected")
                self.assertFalse(self.adapter.stop_requested.is_set())

    def test_no_writes_or_damping_before_explicit_arm(self):
        self.adapter.step()
        self.adapter.abort("unarmed_abort")
        self.assertEqual(self.transport.writes, [])
        self.assertFalse(self.adapter.stop_requested.is_set())

    def test_ready_confirmation_expires_without_commanding_robot(self):
        self.now += 0.21
        self.state()
        self.assertFalse(self.adapter.arm())
        self.assertIn("older than", self.adapter.arm_rejection_reason)
        self.adapter.abort("hardware_arm_rejected")
        self.adapter.step()
        self.assertFalse(self.adapter.stop_requested.is_set())
        self.assertEqual(self.transport.writes, [])

    def test_abort_during_readiness_query_cannot_grant_control(self):
        def interrupted_query():
            self.adapter.abort("controller_feedback_timeout")
            return 0, 4

        self.transport.read_ready_fsm = interrupted_query
        self.assertFalse(self.adapter.check_ready_mode())
        self.assertFalse(self.adapter.arm())
        self.assertFalse(self.adapter.armed)
        self.assertEqual(self.transport.writes, [])

    def test_double_start_before_arming_still_requests_damping(self):
        self.state(pressed=True)
        self.now += 0.05
        self.state()
        self.now += 0.05
        self.state(pressed=True)
        self.assertTrue(self.adapter.stop_requested.is_set())
        self.adapter.stop_once()
        self.assertEqual(self.transport.stop_calls, 1)
        self.assertEqual(self.transport.mode, 1)
        self.assertFalse(self.adapter.arm())

    def press_f3_twice(self):
        self.state(f3=True)
        self.now += 0.05
        self.state()
        self.now += 0.05
        self.state(f3=True)

    def test_balance_preparation_is_cached_before_f3(self):
        policy = Mock(kp=[40.0] * 29, kd=[1.0] * 29)
        policy.update.return_value = [0.0] * 29
        self.adapter.soft_reset_policy = None
        with patch("robot_safety.hardware_adapter.holosoma_weights", return_value="model.onnx"), patch("robot_safety.hardware_adapter.HolosomaLocomotionPolicy", return_value=policy) as load:
            self.assertTrue(self.adapter._ensure_soft_reset_policy())
            self.assertTrue(self.adapter._ensure_soft_reset_policy())
            self.press_f3_twice()
            self.adapter.stop_once()
            self.now += 0.02
            self.adapter.stop_once()
        load.assert_called_once()
        policy.warmup.assert_called_once()
        self.assertEqual(len(self.transport.body_commands), 2)

    def test_soft_reset_balance_preserves_gains_within_physical_limits(self):
        data = profile_data()
        data["joints"]["left_hip_pitch"]["max_torque_nm"] = 0.1
        self.adapter.profile = load_profile(data)
        policy = Mock(kp=[40.0] * 29, kd=[1.0] * 29)
        policy.update.return_value = [0.0] * 29
        self.adapter.soft_reset_policy = policy
        self.press_f3_twice()
        self.adapter.stop_once()
        self.now += 0.02
        policy.update.return_value = [0.2] + [0.0] * 28
        self.adapter.stop_once()
        self.assertEqual(len(self.transport.body_commands), 2)
        target = self.transport.body_commands[-1][0][0]
        self.assertEqual(target["kp"], 40.0)
        self.assertAlmostEqual(target["q"], 0.9 * 0.1 / 40)
        self.assertEqual(self.transport.stop_calls, 0)

    def test_soft_reset_refresh_reuses_the_last_policy_output(self):
        self.transport.repeat_body = Mock(return_value=True)
        self.adapter._repeat_body_once()
        self.transport.repeat_body.assert_not_called()
        self.press_f3_twice()
        self.adapter.step()
        self.adapter.soft_reset_policy.update.assert_called_once()
        for _ in range(10):
            self.adapter._repeat_body_once()
        self.assertEqual(self.transport.repeat_body.call_count, 10)
        self.adapter.soft_reset_policy.update.assert_called_once()
        self.assertEqual(len(self.transport.body_commands), 1)

    def test_balance_inference_failure_requests_damping(self):
        policy = Mock(kp=[40.0] * 29, kd=[1.0] * 29)
        policy.update.side_effect = ValueError("inference failed")
        self.adapter.soft_reset_policy = policy
        self.press_f3_twice()
        self.adapter.stop_once()
        self.assertEqual(self.transport.body_commands, [])
        self.assertEqual(self.transport.stop_calls, 1)

    def test_failed_warmup_keeps_stock_control_and_does_not_reload_on_f3(self):
        policy = Mock(kp=[40.0] * 29, kd=[1.0] * 29)
        policy.warmup.side_effect = ValueError("warmup failed")
        self.adapter.soft_reset_policy = None
        with patch("robot_safety.hardware_adapter.holosoma_weights", return_value="model.onnx"), patch("robot_safety.hardware_adapter.HolosomaLocomotionPolicy", return_value=policy) as load:
            self.assertFalse(self.adapter._ensure_soft_reset_policy())
            self.press_f3_twice()
            self.adapter.stop_once()
        load.assert_called_once()
        self.assertIsNone(self.adapter.soft_reset_policy)
        self.assertEqual(self.transport.release_calls, 0)
        self.assertEqual(self.transport.body_commands, [])

    def test_double_f3_runs_the_balance_network(self):
        self.take_ownership()
        self.adapter.soft_reset_policy = Mock(
            update=Mock(side_effect=lambda q, dq, quat, gyro: list(q)),
            kp=list(range(1, 30)), kd=[0.5] * 29,
        )
        self.press_f3_twice()
        self.assertEqual(self.adapter.abort_reason, "hardware_controller_double_f3")
        self.adapter.stop_once()
        self.assertEqual(self.transport.release_calls, 1)
        self.assertEqual(self.transport.stop_calls, 0)
        targets, machine = self.transport.body_commands[-1]
        self.assertEqual(len(targets), 29)
        self.assertEqual([targets[i]["kp"] for i in range(29)], list(range(1, 30)))
        self.assertTrue(all(target["kd"] == 0.5 for target in targets.values()))
        self.assertEqual(machine, 5)
        self.state(pressed=True)
        self.now += 0.05
        self.state()
        self.now += 0.05
        self.state(pressed=True)
        self.adapter.stop_once()
        self.assertEqual(self.transport.restore_calls, 1)
        self.assertEqual(self.transport.stop_calls, 1)
        self.assertEqual(self.transport.mode, 1)
        self.assertFalse(self.adapter.stop_requested.is_set())
        self.adapter.stop_once()
        self.assertEqual(self.transport.stop_calls, 1)
        self.state()
        self.now += 0.05
        self.state(pressed=True)
        self.now += 0.05
        self.state()
        self.now += 0.05
        self.state(pressed=True)
        self.adapter.stop_once()
        self.assertEqual(self.transport.stop_calls, 2)

        # The next soft reset initializes inference from the new measured state.
        self.assertFalse(self.adapter.soft_reset_active)
        self.state()
        self.press_f3_twice()
        self.adapter.receive_state(
            SimpleNamespace(
                motor_state=[SimpleNamespace(q=0.4, dq=0.0, temperature=(25, 25), tau_est=0.0) for _ in range(35)],
                mode_machine=5,
                imu_state=SimpleNamespace(quaternion=(1., 0., 0., 0.), gyroscope=(0., 0., 0.)),
                wireless_remote=[0] * 40,
            )
        )
        self.adapter.stop_once()
        self.assertEqual([v["q"] for v in self.transport.body_commands[-1][0].values()], [0.4] * 29)
        self.assertEqual(self.adapter.soft_reset_policy.reset.call_count, 2)

    def test_rejected_readiness_shutdown_sends_no_motor_commands(self):
        self.transport.read_ready_fsm = lambda: (3104, None)
        self.transport.close = lambda: None
        self.assertFalse(self.adapter.check_ready_mode())
        self.adapter.abort("hardware_readiness_rejected")
        self.adapter.close()
        self.assertEqual(self.transport.writes, [])
        self.assertEqual(self.transport.stop_calls, 0)

    def test_initial_measured_pose_and_gradual_ownership(self):
        self.run_first()
        targets, weight = self.transport.writes[-1]
        self.assertEqual(set(targets), set(UPPER_BODY_INDICES))
        self.assertEqual(weight, 0.0)
        self.assertTrue(all(target["q"] == 0.0 for target in targets.values()))
        self.assertEqual(targets[16]["kp"], 2.0)
        self.now += 0.005
        self.adapter.step()
        self.assertGreater(self.transport.writes[-1][1], 0)
        self.assertLess(self.transport.writes[-1][1], 0.001)

    def take_ownership(self):
        self.run_first()
        self.now += 0.005
        self.state()
        self.adapter.step()
        self.assertGreater(self.transport.writes[-1][1], 0)

    def test_abort_holds_owned_pose_and_blocks_further_motion(self):
        self.take_ownership()
        count = len(self.transport.writes)
        self.adapter.abort("manual_stop")
        self.assertTrue(self.adapter.stop_requested.is_set())
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertEqual(len(self.transport.writes), count)
        self.adapter.stop_once()
        targets, weight = self.transport.writes[-1]
        self.assertEqual(weight, 1.0)
        self.assertEqual(targets[16]["q"], 0.0)
        self.assertEqual(targets[16]["dq"], 0.0)
        self.assertEqual((targets[16]["kp"], targets[16]["kd"]), (2.0, 0.5))
        self.assertEqual(self.transport.stop_calls, 0)
        self.assertIsNone(self.adapter.reported_fsm_id)
        self.assertFalse(self.adapter.arm())

    def test_confirmed_damping_requires_arm_release_and_rearm_starts_fresh(self):
        self.take_ownership()
        self.adapter.abort("controller_double_start", request_damping=True)
        self.adapter.release_damp_when_confirmed = True
        self.transport.write_success = False
        self.adapter._request_stock_damping()
        self.assertEqual(self.adapter.abort_reason, "controller_double_start")
        self.assertFalse(self.adapter.damp_settled)
        self.transport.write_success = True
        self.adapter._request_stock_damping()
        self.assertTrue(self.adapter.damp_settled)
        self.assertFalse(self.adapter.armed)
        self.now += 1.0
        self.state()
        self.transport.mode = 4
        self.run_first()
        self.assertIsNone(self.adapter.abort_reason)
        self.assertEqual(self.transport.writes[-1][1], 0.0)

    def test_hold_keeps_the_pose_from_the_stop_and_damps_when_feedback_is_stale(self):
        self.take_ownership()
        self.adapter.abort("manual_stop")
        self.adapter.stop_once()
        self.now += 0.01
        remote = [0] * 40
        motors = [SimpleNamespace(q=0.0, dq=0.0, temperature=(25, 25), tau_est=0.0) for _ in range(35)]
        motors[16] = SimpleNamespace(q=0.04, dq=0.2)
        self.adapter.receive_state(
            SimpleNamespace(
                motor_state=motors, mode_machine=5, wireless_remote=remote
            )
        )
        self.adapter.stop_once()
        self.assertEqual(self.transport.writes[-1][0][16]["q"], 0.0)
        self.assertEqual(self.transport.stop_calls, 0)
        self.now += 0.3
        self.adapter.stop_once()
        self.assertEqual(self.transport.stop_calls, 1)
        self.assertEqual(self.transport.writes[-1], ({}, 0.0))
        self.assertEqual(self.adapter.reported_fsm_id, 1)

    def test_expiry_does_not_extend_when_the_same_command_is_resubmitted(self):
        self.run_first()
        expiry = self.adapter.command_expires_secs
        self.now = expiry + 0.001
        self.state()
        self.adapter.submit(self.command, self.now + 0.1)
        self.assertEqual(self.adapter.abort_reason, "hardware_command_expired")

    def test_loop_stall_aborts_instead_of_catching_up(self):
        self.run_first()
        self.now += 0.03
        self.state()
        self.adapter.step()
        self.assertEqual(self.adapter.abort_reason, "hardware_output_loop_stall")
        self.assertEqual(len(self.transport.writes), 1)

    def test_stale_feedback_and_wrong_machine_refuse_output(self):
        self.state(machine=6)
        self.assertFalse(self.adapter.arm())
        self.state()
        self.run_first()
        self.now += 0.3
        self.adapter.step()
        self.assertEqual(self.adapter.abort_reason, "hardware_feedback_timeout")

    def test_direct_sdk_double_start_bypasses_ros(self):
        self.run_first()
        for pressed in (True, False, True):
            self.now += 0.01
            self.state(pressed)
        self.assertEqual(self.adapter.abort_reason, "hardware_controller_double_start")
        self.assertTrue(self.adapter.stop_requested.is_set())

    def test_leg_velocity_above_the_cap_is_clipped_and_warned(self):
        self.run_first()
        self.now += 0.005
        self.command["dq"][3] = 0.101
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertIsNone(self.adapter.abort_reason)
        self.assertEqual(len(self.transport.writes), 2)
        warning = self.adapter.position_warnings["left_knee"]
        self.assertEqual(warning["velocity_reason"], "joint_velocity_clipped")
        self.assertEqual(warning["dq_commanded"], 0.1)

    def test_leg_torque_is_rejected(self):
        legs = list(self.adapter.profile.legs)
        legs[3] = replace(legs[3], max_torque_nm=0.001)
        self.adapter.profile = replace(self.adapter.profile, legs=tuple(legs))
        self.command["q"][3] = 0.004
        self.run_first()
        self.assertIn("left_knee: joint_torque_limit", self.adapter.abort_reason)
        self.assertEqual(self.transport.writes, [])

    def test_commanded_speed_above_the_cap_is_clipped(self):
        self.run_first()
        self.now += 0.005
        self.command["dq"][16] = 0.101
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertIsNone(self.adapter.abort_reason)
        self.assertEqual(len(self.transport.writes), 2)
        self.assertEqual(
            self.adapter.position_warnings["left_shoulder_roll"]["dq_commanded"], 0.1
        )

    def test_initial_and_runtime_pose_gaps_remain_diagnostic(self):
        self.command["q"][16] = 0.05
        self.command["q"][3] = 0.6
        self.run_first()
        self.assertIsNone(self.adapter.abort_reason)
        self.assertEqual(set(self.transport.writes[-1][0]), set(UPPER_BODY_INDICES))
        self.now += 0.005
        self.command["q"][16] = 0.6
        self.command["q"][3] = -0.6
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertIsNone(self.adapter.abort_reason)
        self.assertEqual(len(self.transport.writes), 2)
        self.assertFalse(self.adapter.stop_requested.is_set())
        ranges = self.adapter.joint_diagnostics_snapshot()["left_shoulder_roll"]
        self.assertEqual(ranges["command_gap_rad"]["max_abs"], 0.6)
        self.assertEqual(ranges["response_error_rad"]["samples"], 0)

    def test_response_diagnostics_wait_for_feedback_after_a_successful_full_weight_write(self):
        self.adapter.profile = replace(self.adapter.profile, handover_secs=0.005)
        self.run_first()
        self.assertIsNone(self.adapter.q_applied)
        self.now += 0.01
        self.state()
        self.command["q"][16] = 0.0005
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertIsNone(self.adapter.abort_reason)
        self.assertEqual(self.adapter.q_applied[16], 0.0005)
        self.adapter.record_response()
        self.assertEqual(
            self.adapter.joint_diagnostics_snapshot()["left_shoulder_roll"]
            ["response_error_rad"]["samples"], 0,
        )
        self.now += 0.01
        self.state()
        # A new proposal does not change the response recorded for the prior write.
        self.command["q"][16] = 0.0
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertIsNone(self.adapter.abort_reason)
        self.assertEqual(len(self.transport.writes), 3)
        self.assertAlmostEqual(
            self.adapter.joint_diagnostics_snapshot()["left_shoulder_roll"]
            ["response_error_rad"]["max_abs"], 0.0005,
        )

    def test_failed_writes_and_uncontrolled_legs_do_not_create_response_targets(self):
        self.adapter.profile = replace(self.adapter.profile, handover_secs=0.005)
        self.run_first()
        self.now += 0.01
        self.state()
        self.adapter.step()
        self.assertEqual(set(self.adapter.q_applied), set(UPPER_BODY_INDICES))
        previous = dict(self.adapter.q_applied)
        stamp = self.adapter.command_time
        self.now += 0.01
        self.state()
        self.command["q"][16] = 0.0005
        self.adapter.submit(self.command, self.now + 0.1)
        self.transport.write_success = False
        self.adapter.step()
        self.assertEqual(self.adapter.abort_reason, "hardware_write_failed")
        self.assertEqual(self.adapter.q_applied, previous)
        self.assertEqual(self.adapter.command_time, stamp)

    def test_sdk_value_error_preserves_original_message(self):
        def invalid_write(*_args):
            raise ValueError("fake SDK encoding failure")

        self.transport.write_arm = invalid_write
        self.run_first()
        self.assertIn("fake SDK encoding failure", self.adapter.abort_reason)
        self.assertTrue(self.adapter.stop_requested.is_set())

    def test_failed_write_latches_stop(self):
        self.transport.write_success = False
        self.run_first()
        self.assertEqual(self.adapter.abort_reason, "hardware_write_failed")

    def test_failed_hold_write_and_failed_damping_rpc_still_release(self):
        self.take_ownership()
        self.adapter.abort("stop")
        self.transport.write_success = False
        self.transport.rpc_success = False
        self.adapter.stop_once()
        self.assertIsNone(self.adapter.damping_rpc_code)
        self.assertEqual(self.transport.writes[-1], ({}, 0.0))
        self.assertEqual(self.transport.stop_calls, 1)
        self.transport.rpc_success = True
        self.adapter.stop_once()
        self.assertEqual(self.adapter.damping_rpc_code, 0)

    def test_exit_before_ownership_does_not_damp_and_exit_after_does(self):
        self.run_first()
        self.transport.close = lambda: None
        self.adapter.close()
        self.assertEqual(self.transport.stop_calls, 0)
        self.setUp()
        self.take_ownership()
        self.transport.close = lambda: None
        self.adapter.close()
        self.assertEqual(self.transport.stop_calls, 1)
        self.assertEqual(self.transport.writes[-1], ({}, 0.0))
        self.assertEqual(self.adapter.reported_fsm_id, 1)

    def test_feedback_and_damping_continue_during_a_blocked_write(self):
        from threading import Event, Thread

        for whole_body in (False, True):
            with self.subTest(whole_body=whole_body):
                self.setUp()
                self.adapter.whole_body = whole_body
                entered, release, stopped = Event(), Event(), Event()

                def blocked_write(targets, _mode, entered=entered, release=release):
                    if targets:
                        entered.set()
                        release.wait(2.0)
                    return True

                if whole_body:
                    self.transport.write_body = blocked_write
                else:
                    self.transport.write_arm = blocked_write
                self.assertTrue(self.adapter.arm())
                self.adapter.submit(self.command, self.now + 0.1)
                if whole_body:
                    self.adapter.step()
                writer = Thread(target=self.adapter.step, daemon=True)

                def stop_from_feedback(stopped=stopped):
                    self.state()
                    self.adapter.abort("controller_stop", request_damping=True)
                    self.adapter.stop_once()
                    stopped.set()

                stopper = Thread(target=stop_from_feedback, daemon=True)
                writer.start()
                try:
                    self.assertTrue(entered.wait(1.0))
                    stopper.start()
                    self.assertTrue(stopped.wait(0.5))
                    self.assertEqual(self.transport.stop_calls, 1)
                    self.assertEqual(self.adapter.abort_reason, "controller_stop")
                    self.assertEqual(self.adapter.stop_requested.is_set(), not whole_body)
                finally:
                    release.set()
                    writer.join(timeout=1.0)
                    if stopper.ident is not None:
                        stopper.join(timeout=1.0)

    def test_template_profile_loads_and_contains_example_targets(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "robot_safety/profiles/g1_type5.template.json"
        )
        profile = HardwareProfile.load(path)
        self.assertEqual(profile.allowed_start_fsm_ids, (4,))
        self.assertGreater(profile.max_publish_gap_secs, 0.02)
        targets = {
            16: 1.5707963267948966,
            23: -1.5707963267948966,
            18: 1.5707963267948966,
            25: 1.5707963267948966,
        }
        for index, limits in zip(UPPER_BODY_INDICES, profile.joints):
            for position in (0.0, targets.get(index, 0.0)):
                self.assertLess(limits.position_min_rad, position)
                self.assertGreater(limits.position_max_rad, position)
            self.assertEqual((limits.kp, limits.kd), (60.0, 1.5))

    def test_profile_requires_complete_finite_settings(self):
        for field, value in (
            ("handover_secs", float("nan")),
            ("expected_machine_type", 6),
            ("allowed_start_fsm_ids", [1]),
            ("joints", {}),
        ):
            with self.subTest(field=field), self.assertRaises(ValueError):
                data = profile_data()
                data[field] = value
                load_profile(data)

    def _state(self, motors, gyro=None):
        message = SimpleNamespace(
            motor_state=motors, mode_machine=5, wireless_remote=[0] * 40
        )
        if gyro is not None:
            message.imu_state = SimpleNamespace(
                quaternion=(1.0, 0.0, 0.0, 0.0), gyroscope=gyro
            )
        self.adapter.receive_state(message)

    def test_measured_speed_and_body_spin_abort(self):
        self.run_first()
        motors = [SimpleNamespace(q=0.0, dq=0.0, temperature=(25, 25), tau_est=0.0) for _ in range(35)]
        self._state(motors, gyro=(5.0, 5.0, 0.0))
        self.assertIsNone(self.adapter.abort_reason)
        motors[3].dq = 0.101
        self._state(motors)
        self.assertIn("left_knee: joint_velocity_limit", self.adapter.abort_reason)
        self.adapter.abort_reason = None
        motors[3].dq = 0.0
        self._state(motors, gyro=(6.01, 0.0, 0.0))
        self.assertIn("base_angular_speed_limit", self.adapter.abort_reason)

    def test_temperature_battery_and_missing_health_reject(self):
        for fault, reason in (("winding", "motor_winding_temperature_limit"),
                              ("casing", "motor_casing_temperature_limit"),
                              ("battery", "low_battery"),
                              ("missing", "missing_motor_temperature")):
            with self.subTest(fault=fault):
                self.setUp()
                if fault == "battery":
                    self.adapter.receive_battery(SimpleNamespace(soc=9))
                else:
                    temps = [(85, 120)] * 29
                    if fault == "winding": temps[0] = (85, 121)
                    if fault == "casing": temps[0] = (86, 120)
                    self.adapter.feedback = replace(self.adapter.feedback, temperature=() if fault == "missing" else tuple(temps))
                self.assertFalse(self.adapter.arm())
                self.assertIn(reason, self.adapter.arm_rejection_reason)

    def test_health_fault_during_execution_aborts(self):
        self.run_first()
        self.adapter.receive_battery(SimpleNamespace(soc=9))
        self.assertEqual(self.adapter.abort_reason, "low_battery")
        self.adapter.stop_once()
        self.assertEqual(self.transport.stop_calls, 1)
        self.setUp()
        self.run_first()
        motors = [SimpleNamespace(q=0., dq=0., temperature=(85, 120), tau_est=0.) for _ in range(35)]
        self._state(motors)
        self.assertIsNone(self.adapter.abort_reason)
        motors[3].temperature = (85, 121)
        self._state(motors)
        self.assertIn("motor_winding_temperature_limit", self.adapter.abort_reason)
        self.adapter.stop_once()
        self.assertEqual(self.transport.stop_calls, 1)

    def test_stale_battery_blocks_next_write_and_requests_damping(self):
        self.run_first()
        self.adapter.battery_time = self.now - 5.0
        count = len(self.transport.writes)
        self.adapter.step()
        self.assertEqual(self.adapter.abort_reason, "battery_feedback_timeout")
        self.assertEqual(len(self.transport.writes), count)
        self.adapter.stop_once()
        self.assertEqual(self.transport.stop_calls, 1)

    def test_clipping_precedes_torque_checks_and_sdk_write(self):
        self.run_first()
        self.adapter.first_output_secs = self.now - self.adapter.profile.handover_secs
        q = list(self.adapter.feedback.q)
        q[16] = 0.98
        self.adapter.feedback = replace(self.adapter.feedback, q=tuple(q))
        joints = list(self.adapter.profile.joints)
        joints[4] = replace(joints[4], max_torque_nm=0.05)
        self.adapter.profile = replace(self.adapter.profile, joints=tuple(joints))
        self.command["q"][16] = 1.2
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertIsNone(self.adapter.abort_reason)
        self.assertEqual(self.transport.writes[-1][0][16]["q"], 1.0)
        self.assertEqual(self.adapter.q_applied[16], 1.0)
        self.assertEqual(self.adapter.command["q"][16], 1.2)
        warning = self.adapter.joint_diagnostics_snapshot()["left_shoulder_roll"]["position_clipping"]
        self.assertEqual(warning["q_proposed"], 1.2)
        self.assertEqual(warning["q_commanded"], 1.0)

    def test_invalid_battery_before_arming_does_not_send_control(self):
        self.adapter.receive_battery(SimpleNamespace(soc=float("nan")))
        self.adapter.stop_once()
        self.assertEqual(self.adapter.abort_reason, "invalid_battery_feedback")
        self.assertEqual(self.transport.stop_calls, 0)
        self.assertFalse(self.transport.writes)


class WholeBodyRepeatTests(unittest.TestCase):
    def test_late_refresh_waits_instead_of_spinning(self):
        delay, following = whole_body_repeat_wait(10.0, 9.999)
        self.assertAlmostEqual(delay, 0.001)
        self.assertEqual(following, 10.0)
        delay, following = whole_body_repeat_wait(10.0, 10.004)
        self.assertEqual((delay, following), (0.002, 10.006))


class TransportEncodingTests(unittest.TestCase):
    def test_startup_initializes_clients_without_control_rpcs(self):
        for enabled in (True, False):
            with self.subTest(output_enabled=enabled):
                sdk = Mock()
                modules = {
                    "unitree_sdk2py." + name: sdk for name in (
                        "core.channel", "g1.loco.g1_loco_client", "idl.default",
                        "idl.unitree_hg.msg.dds_", "utils.crc",
                        "comm.motion_switcher.motion_switcher_client", "g1.audio.g1_audio_client",
                    )
                }
                with patch.dict("sys.modules", modules), patch(
                    "robot_safety.unitree_transport.MotorWriterLock"
                ), patch("robot_safety.unitree_transport.LatestSdkReader") as reader, patch(
                    "robot_safety.unitree_transport.LatestSdkPoller"
                ) as poller:
                    on_state = Mock()
                    transport = UnitreeArmTransport("fake", 0, on_state, Mock(), output_enabled=enabled)
                    self.assertEqual(reader.call_args[0][0], "rt/lowstate")
                    self.assertIs(poller.call_args[0][1], on_state)
                    self.assertEqual(poller.call_args[0][2], 0.005)
                    poller.return_value.start.assert_called_once()
                    self.assertIs(transport.switcher, sdk.MotionSwitcherClient.return_value)
                    self.assertIs(transport._audio(), sdk.AudioClient.return_value)
                if not enabled:
                    sdk.ChannelPublisher.assert_not_called()
                    self.assertIsNone(transport.writer_lock)
                    transport.write_arm({}, 0.)
                    transport.write_body({i: {} for i in range(29)}, 5)
                    transport.release_motion()
                    transport.restore_motion()
                    transport.request_damping()
                sdk.MotionSwitcherClient.assert_called_once()
                sdk.AudioClient.assert_called_once()
                sdk.MotionSwitcherClient.return_value.ReleaseMode.assert_not_called()
                sdk.MotionSwitcherClient.return_value.SelectMode.assert_not_called()
                sdk.LocoClient.return_value.SetFsmId.assert_not_called()
                sdk.AudioClient.return_value.TtsMaker.assert_not_called()
                sdk.ChannelPublisher.return_value.Write.assert_not_called()
                transport.close()

    def test_readiness_and_stop_queries_use_separate_clients(self):
        transport = UnitreeArmTransport.__new__(UnitreeArmTransport)
        transport.readiness_client = SimpleNamespace(GetFsmId=lambda: (0, 4))
        transport.locomotion = SimpleNamespace(GetFsmId=lambda: (0, 1))
        self.assertEqual(transport.read_ready_fsm(), (0, 4))
        self.assertEqual(transport.read_fsm(), (0, 1))

    def test_readiness_retries_only_timeout_and_requires_a_reply(self):
        from unittest.mock import Mock

        for replies, expected_calls, expected in (
            ([(3104, None), (0, 4)], 2, (0, 4)),
            ([(3104, None), (3104, None)], 2, (3104, None)),
            ([(0, 1)], 1, (0, 1)),
            ([(3102, None)], 1, (3102, None)),
        ):
            with self.subTest(replies=replies):
                transport = UnitreeArmTransport.__new__(UnitreeArmTransport)
                transport.readiness_client = Mock()
                transport.readiness_client.GetFsmId.side_effect = replies
                self.assertEqual(transport.read_ready_fsm(), expected)
                self.assertEqual(
                    transport.readiness_client.GetFsmId.call_count, expected_calls
                )
                self.assertEqual(len(transport.readiness_query_results), expected_calls)

    def test_arm_and_body_commands_use_sdk_slots_and_crc(self):
        transport = UnitreeArmTransport.__new__(UnitreeArmTransport)
        transport.output_enabled = True
        transport.write_lock = Lock()

        def make_command():
            return SimpleNamespace(
                motor_cmd=[
                    SimpleNamespace(q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0)
                    for _ in range(35)
                ],
                crc=0,
            )

        transport.make_command = make_command
        transport.crc = SimpleNamespace(Crc=lambda message: 12345)
        written = []
        transport.publisher = SimpleNamespace(
            Write=lambda message: written.append(message) or True
        )
        self.assertTrue(
            transport.write_arm(
                {16: {"q": 0.1, "dq": 0.0, "kp": 2.0, "kd": 0.5, "tau": 0.0}}, 0.2
            )
        )
        message = written[0]
        self.assertEqual(message.motor_cmd[16].q, 0.1)
        self.assertEqual(message.motor_cmd[29].q, 0.2)
        self.assertTrue(all(message.motor_cmd[i].kp == 0 for i in range(12)))
        self.assertEqual(message.crc, 12345)

        transport.lowcmd = transport.publisher
        targets = {i: {"q": i / 100, "dq": 0.1, "kp": 2., "kd": 0.5, "tau": 0.2} for i in range(29)}
        self.assertTrue(transport.write_body(targets, 5))
        msg = written[-1]
        self.assertEqual((msg.mode_machine, msg.mode_pr, msg.crc), (5, 0, 12345))
        self.assertTrue(transport.repeat_body(7))
        repeated = written[-1]
        self.assertIs(repeated, msg)
        self.assertEqual((repeated.mode_machine, repeated.mode_pr, repeated.crc), (7, 0, 12345))
        self.assertEqual(repeated.motor_cmd[4].q, targets[4]["q"])
        transport.clear_body()
        self.assertIsNone(transport.repeat_body(7))
        self.assertIs(written[-1], repeated)
        for i, fields in targets.items():
            self.assertEqual(msg.motor_cmd[i].mode, 1)
            for field, value in fields.items():
                self.assertEqual(getattr(msg.motor_cmd[i], field), value)
        with self.assertRaises(ValueError):
            transport.write_body({0: targets[0]}, 5)

    def test_writer_lock_excludes_and_releases(self):
        from robot_safety.unitree_transport import MotorWriterLock

        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "writer.lock")
            first = MotorWriterLock(path)
            try:
                with self.assertRaises(RuntimeError):
                    MotorWriterLock(path)
            finally:
                first.close()
            MotorWriterLock(path).close()

    def test_soft_reset_mutes_developer_mode_then_speaks(self):
        from robot_safety.unitree_transport import (
            SOFT_RESET_PHRASE,
            UnitreeArmTransport,
        )

        events = []

        class Switcher:
            def CheckMode(self):
                return 0, {"name": "ai"}

            def ReleaseMode(self):
                events.append("release")
                return 0, None

            def SetTimeout(self, timeout):
                return None

            def _Call(self, api_id, parameter):
                events.append(("silent", parameter))
                return 0, None

        class Audio:
            def __init__(self):
                self.tts_index = 0

            def TtsMaker(self, text, speaker_id):
                events.append((text, speaker_id, self.tts_index))

        transport = UnitreeArmTransport.__new__(UnitreeArmTransport)
        transport.switcher = Switcher()
        transport.output_enabled = True
        transport.audio = Audio()
        transport.saved_motion_name = None
        transport.motion_restored = False
        self.assertEqual(transport.release_motion(announce_soft_reset=True), 0)
        transport._speech_thread.join(timeout=1.0)
        self.assertEqual(
            events,
            [
                ("silent", '{"silent": true}'),
                "release",
                (SOFT_RESET_PHRASE, 1, 1),
            ],
        )
