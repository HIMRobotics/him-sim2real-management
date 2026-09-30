"""Exercise hardware output with fake SDK calls, never a robot connection."""

import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from robot_safety.hardware_constants import G1_29_JOINT_NAMES
from robot_safety.hardware_adapter import G1ArmAdapter
from robot_safety.hardware_constants import UPPER_BODY_INDICES
from robot_safety.hardware_profile import HardwareProfile
from robot_safety.unitree_transport import UnitreeArmTransport


def profile_data():
    # Artificial test bounds and gains. These are not a deployable hardware profile.
    return {
        "expected_machine_type": 5,
        "allowed_start_fsm_ids": [4],
        "handover_secs": 2.0,
        "max_publish_gap_secs": 0.02,
        "initial_position_tolerance_rad": 0.005,
        "initial_velocity_tolerance_rad_secs": 0.01,
        "joints": {
            G1_29_JOINT_NAMES[i]: {
                "position_min_rad": -1.0,
                "position_max_rad": 1.0,
                "kp": 2.0,
                "kd": 0.5,
                "max_velocity_rad_secs": 0.1,
                "max_acceleration_rad_secs2": 0.2,
                "max_tracking_error_rad": 0.1,
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
        self.lowcmds = []
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

    def release_motion(self):
        self.release_calls += 1
        return 0

    def restore_motion(self):
        self.restore_calls += 1
        return 0

    def write_lower_body(self, lower_q, arm_q, leg_kp, leg_kd, machine):
        self.lowcmds.append((list(lower_q), list(arm_q), machine))
        return True


class HardwareAdapterTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.adapter = G1ArmAdapter(load_profile(profile_data()), lambda: self.now)
        self.transport = FakeTransport()
        self.adapter.transport = self.transport
        self.state()
        self.assertTrue(self.adapter.check_ready_mode())
        self.command = {name: [0.0] * 29 for name in ("q", "dq", "kp", "kd", "tau")}

    def state(self, pressed=False, f3=False, machine=5):
        remote = [0] * 40
        remote[2] = (4 if pressed else 0) | (128 if f3 else 0)
        self.adapter.receive_state(
            SimpleNamespace(
                motor_state=[SimpleNamespace(q=0.0, dq=0.0) for _ in range(35)],
                mode_machine=machine,
                wireless_remote=remote,
            )
        )

    def run_first(self):
        self.assertTrue(self.adapter.check_ready_mode())
        self.assertTrue(self.adapter.arm())
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()

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
        policy = Mock()
        policy.update.return_value = [0.0] * 15
        with patch("robot_safety.hardware_adapter.BalancePolicy", return_value=policy) as load:
            self.assertTrue(self.adapter._ensure_balance_policy())
            self.assertTrue(self.adapter._ensure_balance_policy())
            self.press_f3_twice()
            self.adapter.stop_once()
            self.now += 0.02
            self.adapter.stop_once()
        load.assert_called_once()
        policy.warmup.assert_called_once()
        self.assertEqual(len(self.transport.lowcmds), 2)

    def test_soft_reset_balance_bypasses_submission_limits(self):
        data = profile_data()
        data["joints"]["left_hip_pitch"]["max_torque_nm"] = 0.1
        self.adapter.profile = load_profile(data)
        policy = Mock()
        policy.update.return_value = [0.0] * 15
        self.adapter.balance_policy = policy
        self.press_f3_twice()
        with patch("robot_safety.hardware_adapter.check_command") as command_check, patch(
            "robot_safety.hardware_adapter.check_torque"
        ) as torque_check:
            self.adapter.stop_once()
            self.now += 0.02
            policy.update.return_value = [0.2] + [0.0] * 14
            self.adapter.stop_once()
        command_check.assert_not_called()
        torque_check.assert_not_called()
        self.assertEqual(len(self.transport.lowcmds), 2)
        self.assertEqual(self.transport.lowcmds[-1][0][0], 0.2)
        self.assertEqual(self.transport.stop_calls, 0)

    def test_balance_inference_failure_requests_damping(self):
        policy = Mock()
        policy.update.side_effect = ValueError("inference failed")
        self.adapter.balance_policy = policy
        self.press_f3_twice()
        self.adapter.stop_once()
        self.assertEqual(self.transport.lowcmds, [])
        self.assertEqual(self.transport.stop_calls, 1)

    def test_failed_warmup_keeps_stock_control_and_does_not_reload_on_f3(self):
        policy = Mock()
        policy.warmup.side_effect = ValueError("warmup failed")
        with patch("robot_safety.hardware_adapter.BalancePolicy", return_value=policy) as load:
            self.assertFalse(self.adapter._ensure_balance_policy())
            self.press_f3_twice()
            self.adapter.stop_once()
        load.assert_called_once()
        self.assertIsNone(self.adapter.balance_policy)
        self.assertEqual(self.transport.release_calls, 0)
        self.assertEqual(self.transport.lowcmds, [])

    def test_double_f3_runs_the_balance_network(self):
        self.take_ownership()
        self.adapter.balance_policy = type(
            "HoldLegs",
            (),
            {"update": staticmethod(lambda q, dq, quat, gyro: list(q[:15]))},
        )()
        self.press_f3_twice()
        self.assertEqual(self.adapter.abort_reason, "hardware_controller_double_f3")
        self.adapter.stop_once()
        self.assertEqual(self.transport.release_calls, 1)
        self.assertEqual(self.transport.stop_calls, 0)
        lower, arms, machine = self.transport.lowcmds[-1]
        self.assertEqual(len(lower), 15)
        self.assertEqual(len(arms), 14)
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

        # The previous session's arm pose is cleared, so the new F3 session captures the new pose.
        self.assertIsNone(self.adapter.arm_hold_q)
        self.state()
        self.press_f3_twice()
        self.adapter.receive_state(
            SimpleNamespace(
                motor_state=[SimpleNamespace(q=0.4, dq=0.0) for _ in range(35)],
                mode_machine=5,
                wireless_remote=[0] * 40,
            )
        )
        self.adapter.stop_once()
        self.assertEqual(self.transport.lowcmds[-1][1], [0.4] * 14)

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
        motors = [SimpleNamespace(q=0.0, dq=0.0) for _ in range(35)]
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

    def test_first_target_jump_is_rejected(self):
        self.command["q"][16] = 0.05
        self.run_first()
        self.assertIsNotNone(self.adapter.abort_reason)
        self.assertEqual(self.transport.writes, [])

    def test_leg_command_tracking_and_torque_are_rejected(self):
        self.run_first()
        self.now += 0.005
        self.command["q"][3] = 0.2
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        reason = self.adapter.abort_reason
        self.assertIn("left_knee: joint_tracking_error_limit", reason)
        self.assertIn("limit=0.1", reason)
        self.assertEqual(len(self.transport.writes), 1)
        self.assertNotIn(3, self.transport.writes[-1][0])

    def test_leg_velocity_change_is_rejected(self):
        self.run_first()
        self.now += 0.005
        self.command["dq"][3] = 0.05
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertIn(
            "left_knee: joint_velocity_target_change_limit",
            self.adapter.abort_reason,
        )
        self.assertEqual(len(self.transport.writes), 1)

    def test_leg_torque_is_rejected(self):
        legs = list(self.adapter.profile.legs)
        legs[3] = replace(legs[3], max_torque_nm=0.001)
        self.adapter.profile = replace(self.adapter.profile, legs=tuple(legs))
        self.command["q"][3] = 0.004
        self.run_first()
        self.assertIn("left_knee: joint_torque_limit", self.adapter.abort_reason)
        self.assertEqual(self.transport.writes, [])

    def test_tracking_limits_and_velocity_discontinuities_are_rejected(self):
        self.run_first()
        self.now += 0.005
        self.command["dq"][16] = 0.05
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        self.assertIsNotNone(self.adapter.abort_reason)
        self.assertEqual(len(self.transport.writes), 1)

    def test_output_rejection_preserves_joint_and_tracking_measurements(self):
        self.run_first()
        self.now += 0.005
        self.command["q"][16] = 0.2
        self.adapter.submit(self.command, self.now + 0.1)
        self.adapter.step()
        reason = self.adapter.abort_reason
        self.assertIn("left_shoulder_roll: joint_tracking_error_limit", reason)
        self.assertIn("limit=0.1", reason)
        self.assertIn("value=0.2", reason)
        self.assertEqual(len(self.transport.writes), 1)
        self.assertTrue(self.adapter.stop_requested.is_set())

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

    def test_abort_rpc_does_not_wait_for_blocked_motion_write(self):
        from threading import Event, Thread

        entered_write = Event()
        release_write = Event()
        original_write = self.transport.write_arm

        def blocked_write(targets, weight):
            if targets:
                entered_write.set()
                release_write.wait(2.0)
            return original_write(targets, weight)

        self.transport.write_arm = blocked_write
        self.assertTrue(self.adapter.arm())
        self.adapter.submit(self.command, self.now + 0.1)
        writer = Thread(target=self.adapter.step, daemon=True)
        writer.start()
        try:
            self.assertTrue(entered_write.wait(1.0))
            self.adapter.abort("abort_during_write")
            self.adapter.stop_once()
            self.assertEqual(self.transport.stop_calls, 0)
            self.assertTrue(self.adapter.stop_requested.is_set())
        finally:
            release_write.set()
            writer.join(timeout=1.0)
        count = len(self.transport.writes)
        self.adapter.step()
        self.assertEqual(len(self.transport.writes), count)

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
        motors = [SimpleNamespace(q=0.0, dq=0.0) for _ in range(35)]
        motors[3].dq = 0.2
        self._state(motors)
        self.assertIsNone(self.adapter.abort_reason)
        self.now += 0.049
        self._state(motors)
        self.assertIsNone(self.adapter.abort_reason)
        motors[3].dq = 0.0
        self._state(motors)
        motors[3].dq = 0.2
        self.now += 0.01
        self._state(motors)
        self.assertIsNone(self.adapter.abort_reason)
        self.now += 0.05
        self._state(motors)
        self.assertIn("left_knee", self.adapter.abort_reason)
        self.assertIn("joint_velocity_limit", self.adapter.abort_reason)

        self.adapter.abort_reason = None
        motors[3].dq = 0.0
        self._state(motors)
        self._state(motors, gyro=(6.1, 0.0, 0.0))
        self.assertIsNone(self.adapter.abort_reason)
        self.now += 0.05
        self._state(motors, gyro=(6.1, 0.0, 0.0))
        self.assertIn("base_angular_speed_limit", self.adapter.abort_reason)


class TransportEncodingTests(unittest.TestCase):
    def test_startup_initializes_clients_without_control_rpcs(self):
        sdk = Mock()
        modules = {
            "unitree_sdk2py." + name: sdk for name in (
                "core.channel", "g1.loco.g1_loco_client", "idl.default",
                "idl.unitree_hg.msg.dds_", "utils.crc",
                "comm.motion_switcher.motion_switcher_client", "g1.audio.g1_audio_client",
            )
        }
        with patch.dict("sys.modules", modules):
            transport = UnitreeArmTransport("fake", 0, Mock())
            self.assertIs(transport.switcher, sdk.MotionSwitcherClient.return_value)
            self.assertIs(transport._audio(), sdk.AudioClient.return_value)
        sdk.MotionSwitcherClient.assert_called_once()
        sdk.AudioClient.assert_called_once()
        sdk.MotionSwitcherClient.return_value.ReleaseMode.assert_not_called()
        sdk.MotionSwitcherClient.return_value.SelectMode.assert_not_called()
        sdk.LocoClient.return_value.SetFsmId.assert_not_called()
        sdk.AudioClient.return_value.TtsMaker.assert_not_called()
        sdk.ChannelPublisher.return_value.Write.assert_not_called()

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

    def test_only_upper_body_and_weight_slots_are_written_with_crc(self):
        transport = UnitreeArmTransport.__new__(UnitreeArmTransport)

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

    def test_soft_reset_mutes_developer_mode_then_speaks(self):
        from robot_safety.unitree_transport import SOFT_RESET_PHRASE, UnitreeArmTransport

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
        transport.audio = Audio()
        transport.saved_motion_name = None
        transport.motion_restored = False
        self.assertEqual(transport.release_motion(), 0)
        transport._speech_thread.join(timeout=1.0)
        self.assertEqual(
            events,
            [
                ("silent", '{"silent": true}'),
                "release",
                (SOFT_RESET_PHRASE, 1, 1),
            ],
        )
