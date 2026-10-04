"""Safety checks for container commands against the MuJoCo G1."""

import sys
import tempfile
import unittest
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [
    str(ROOT / "packages/policy_safety/src"),
    str(ROOT / "robot_safety"),
    str(ROOT / "containers/sim"),
]

from plant import POSES, Plant
from runner import Runner


def hold(runner, state, sequence, **overrides):
    q = list(state["q"])
    frame = {
        "session_id": runner.session_id,
        "sequence": sequence,
        "observation_id": state["observation_id"],
        "q": q,
        "dq": [0.0] * 29,
        "kp": [40.0] * 29,
        "kd": [2.0] * 29,
        "tau": [0.0] * 29,
    }
    frame.update(overrides)
    return frame


class RunnerTest(unittest.TestCase):
    def test_safety_checks_default_to_enabled(self):
        self.assertTrue(Runner().report()["safety_checks"])

    def test_unchecked_commands_bypass_limits_and_timeouts(self):
        runner = Runner(safety_checks=False)
        state = runner.feedback(0.0)
        frame = hold(
            runner,
            state,
            0,
            q=[12.0] * 29,
            dq=[60.0] * 29,
            kp=[600.0] * 29,
            kd=[60.0] * 29,
            tau=[300.0] * 29,
        )
        self.assertIsNone(runner.command(frame, 1.0))
        np.testing.assert_array_equal(runner.active["tau"], frame["tau"])
        state = runner.feedback(1.0)
        self.assertIsNone(runner.command(hold(runner, state, 1), 1.0))
        runner.step(2.0)
        self.assertIsNone(runner.reason)
        report = runner.report()
        self.assertEqual(report["status"], "completed")
        self.assertFalse(report["safety_checks"])
        self.assertFalse(report["fall_check"])
        self.assertFalse(report["hardware_authorization"])

    def test_checked_action_timeout_still_latches(self):
        runner = Runner()
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        runner.step(0.2)
        self.assertEqual(runner.reason, "action_timeout")

    def test_unchecked_observation_has_no_hidden_age_limit(self):
        runner = Runner(safety_checks=False)
        first = runner.feedback(0.0)
        for step in range(600):
            runner.feedback(step * 0.02)
        self.assertIsNone(runner.command(hold(runner, first, 0), 12.0))
        self.assertEqual(
            runner.command(hold(runner, first, 1, observation_id=10000), 12.0),
            "stale_observation",
        )

    def test_unchecked_fall_and_overspeed_do_not_latch(self):
        from unittest.mock import patch

        runner = Runner(safety_checks=False)
        runner.plant.data.qpos[runner.plant.base_qadr + 2] = 0.1
        runner.plant.data.qvel[runner.plant.base_vadr + 3] = 100.0
        mujoco.mj_forward(runner.plant.model, runner.plant.data)
        with patch.object(
            runner.plant, "step_physics", return_value=np.full(29, 100.0)
        ):
            for step in range(runner.hold_steps + 1):
                runner.step(step * 0.02)
        self.assertIsNone(runner.reason)

    def test_unchecked_run_still_rejects_invalid_vectors(self):
        for bad in ([0.0] * 28, [float("nan")] * 29, [float("inf")] * 29, ["bad"] * 29):
            with self.subTest(bad=bad[0]):
                runner = Runner(safety_checks=False)
                state = runner.feedback(0.0)
                reason = runner.command(hold(runner, state, 0, q=bad), 0.0)
                self.assertEqual(reason, "invalid_command")
                self.assertIsNone(runner.active)
                self.assertEqual(runner.report()["status"], "error")

    def test_unchecked_run_still_requires_session_and_sequence(self):
        runner = Runner(safety_checks=False)
        state = runner.feedback(0.0)
        self.assertEqual(
            runner.command(hold(runner, state, 0, session_id="other"), 0.0),
            "session_mismatch",
        )
        runner = Runner(safety_checks=False)
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        self.assertEqual(runner.command(hold(runner, state, 0), 0.0), "sequence")

    def test_unchecked_run_without_commands_is_not_completed(self):
        self.assertEqual(Runner(safety_checks=False).report()["status"], "no_command")

    def test_hold_stays_up(self):
        runner = Runner()
        target = None
        for step in range(50):
            state = runner.feedback(step * 0.02)
            if target is None:
                target = list(state["q"])
            reason = runner.command(hold(runner, state, step, q=target), step * 0.02)
            self.assertIsNone(reason)
            runner.step(step * 0.02 + 0.01)
        report = runner.report()
        self.assertEqual(report["status"], "within_limits")
        self.assertGreater(runner.plant.state()["height"], 0.5)
        self.assertFalse(report["hardware_authorization"])

    def test_circle_walks_without_falling(self):
        model = ROOT / "examples/walker.onnx"
        if not model.is_file():
            self.skipTest("cached walker weights are not on this machine")
        runner = Runner()
        start = None
        for step in range(150):
            result = runner.circle(step * 0.02)
            self.assertIsNone(result.get("reason"))
            position = runner.plant.state()["position"]
            if start is None:
                start = position
        end = runner.plant.state()
        moved = sum((end["position"][i] - start[i]) ** 2 for i in range(2)) ** 0.5
        self.assertGreater(end["height"], 0.5)
        self.assertGreater(end["upright"], 0.8)
        self.assertGreater(moved, 0.3)

    def test_circle_reports_partial_checks_in_report_and_standalone_replay(self):
        import json
        import os
        from unittest.mock import patch

        from view import export_replay, report_summary

        runner = Runner()
        runner.circle(0.0)
        with tempfile.TemporaryDirectory() as tmp:
            trace = Path(tmp) / "trace.npz"
            with patch.dict(os.environ, {"G1_SIM_TRACE": str(trace)}):
                report = runner.report()
            self.assertEqual(report["status"], "completed")
            self.assertEqual(report["control_path"], "direct")
            self.assertFalse(report["checks"]["command_limits"])
            self.assertFalse(report["checks"]["command_protocol"])
            self.assertFalse(report["checks"]["observation_freshness"])
            self.assertTrue(report["checks"]["joint_speed"])
            self.assertTrue(report["checks"]["fall"])
            self.assertIn("PARTIAL", report_summary(report))
            dest = export_replay(trace, Path(tmp) / "view")
            motion = json.loads((dest / "motion.json").read_text())
            self.assertIn("PARTIAL", motion["summary"])
            self.assertIn("submission checks bypassed", motion["summary"])

    def test_circle_still_checks_falls_and_can_disable_runtime_checks(self):
        for safety_checks in (True, False):
            with self.subTest(safety_checks=safety_checks):
                runner = Runner(safety_checks=safety_checks)
                runner.circle(0.0)
                runner.plant.data.qpos[runner.plant.base_qadr + 2] = 0.1
                mujoco.mj_forward(runner.plant.model, runner.plant.data)
                runner.circle(0.02)
                report = runner.report()
                self.assertEqual(report["checks"]["fall"], safety_checks)
                self.assertEqual(report["checks"]["joint_speed"], safety_checks)
                self.assertEqual(
                    report["status"], "violation" if safety_checks else "completed"
                )
                if safety_checks:
                    self.assertEqual(report["reason"], "fall")

    def test_rejected_command_keeps_the_hold(self):
        runner = Runner()
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        held = runner.active["q"].copy()
        q = list(state["q"])
        q[0] = q[0] + 0.2
        reason = runner.command(hold(runner, state, 1, q=q), 0.02)
        self.assertEqual(reason, "joint_tracking_error_limit")
        np.testing.assert_allclose(runner.active["q"], held)
        for step in range(25):
            runner.step(0.03 + step * 0.02)
        self.assertGreater(runner.plant.state()["height"], 0.5)

    def test_feedforward_torque_rejected(self):
        runner = Runner()
        state = runner.feedback(0.0)
        tau = [0.0] * 29
        tau[0] = 1.0
        reason = runner.command(hold(runner, state, 0, tau=tau), 0.0)
        self.assertEqual(reason, "nonzero_feedforward_torque")
        self.assertEqual(runner.report()["status"], "violation")
        self.assertEqual(runner.command(hold(runner, state, 1), 0.02), reason)

    def test_initial_pose_mismatch_rejected(self):
        runner = Runner()
        state = runner.feedback(0.0)
        q = list(state["q"])
        q[0] = q[0] + 0.05
        reason = runner.command(hold(runner, state, 0, q=q), 0.0)
        self.assertEqual(reason, "initial_joint_position_mismatch")

    def test_wrist_torque_rejected(self):
        runner = Runner()
        state = runner.feedback(0.0)
        q = list(state["q"])
        q[20] = q[20] + 0.02
        kp = [40.0] * 29
        kp[20] = 300.0
        reason = runner.command(hold(runner, state, 0, q=q, kp=kp), 0.0)
        self.assertEqual(reason, "joint_torque_limit")
        self.assertEqual(runner.report()["detail"]["joint"], "left_wrist_pitch")

    def test_bad_shape_session_and_sequence(self):
        runner = Runner()
        state = runner.feedback(0.0)
        short = hold(runner, state, 0)
        short["q"] = short["q"][:-1]
        self.assertEqual(runner.command(short, 0.0), "invalid_command")
        runner = Runner()
        state = runner.feedback(0.0)
        wrong = hold(runner, state, 0, session_id="nope")
        self.assertEqual(runner.command(wrong, 0.0), "session_mismatch")
        runner = Runner()
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        self.assertEqual(runner.command(hold(runner, state, 0), 0.02), "sequence")

    def test_stale_observation_rejected(self):
        runner = Runner()
        state = runner.feedback(0.0)
        self.assertEqual(
            runner.command(hold(runner, state, 0), 0.2), "stale_observation"
        )

    def test_missing_command_is_not_a_pass(self):
        runner = Runner()
        runner.feedback(0.0)
        runner.step(0.01)
        self.assertEqual(runner.report()["status"], "no_command")

    def test_trace_records_each_step(self):
        runner = Runner()
        self.assertEqual(len(runner.trace_qpos), 1)
        runner.step(0.01)
        self.assertEqual(len(runner.trace_qpos), 2)
        self.assertEqual(runner.trace_qpos[-1].shape, (runner.plant.model.nq,))

    def test_live_file_follows_each_step(self):
        import json
        import os
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "live.jsonl"
            os.environ["G1_SIM_LIVE"] = str(path)
            try:
                runner = Runner()
                runner.step(0.01)
            finally:
                os.environ.pop("G1_SIM_LIVE", None)
            lines = path.read_text().splitlines()
        self.assertEqual(len(lines), 2)
        frame = json.loads(lines[-1])
        self.assertEqual(len(frame["qpos"]), runner.plant.model.nq)
        self.assertGreater(frame["t"], 0)

    def test_replay_page_includes_the_g1(self):
        import os
        import tempfile

        from view import export_replay

        runner = Runner()
        runner.step(0.01)
        with tempfile.TemporaryDirectory() as tmp:
            trace = Path(tmp) / "trace.npz"
            os.environ["G1_SIM_TRACE"] = str(trace)
            try:
                runner.report()
                dest = export_replay(trace, Path(tmp) / "view")
                self.assertTrue((dest / "index.html").is_file())
                self.assertTrue((dest / "motion.json").is_file())
                self.assertTrue(any((dest / "meshes").glob("*.stl")))
            finally:
                os.environ.pop("G1_SIM_TRACE", None)

    def test_fallen_pose_detected(self):
        runner = Runner()
        runner.plant.data.qpos[2] = 0.1
        runner.plant.data.qpos[3:7] = [0.0, 1.0, 0.0, 0.0]
        mujoco.mj_forward(runner.plant.model, runner.plant.data)
        self.assertTrue(runner.plant.fallen())
        self.assertEqual(runner.report()["pose"], "standing")
        self.assertTrue(runner.report()["fall_check"])

    def test_unknown_pose_rejected(self):
        with self.assertRaises(ValueError):
            Plant("ceiling")

    def test_counterfactual_poses_hold_without_a_fall(self):
        for pose in POSES:
            if pose == "standing":
                continue
            runner = Runner(pose)
            plant = runner.plant
            self.assertFalse(plant.check_fall)
            self.assertFalse(plant.fallen())
            self.assertTrue(all(c.dist >= -0.005 for c in plant.data.contact))
            q = plant.q()
            low, high = plant.model.jnt_range[plant.joints].T
            self.assertTrue(((q >= low) & (q <= high)).all())
            if pose in ("left_side", "right_side"):
                side = plant.data.xmat[plant.base].reshape(3, 3)[2, 1]
                self.assertAlmostEqual(
                    side, -1.0 if pose == "left_side" else 1.0, places=3
                )
            target = None
            for step in range(25):
                state = runner.feedback(step * 0.02)
                if target is None:
                    target = list(state["q"])
                reason = runner.command(
                    hold(runner, state, step, q=target), step * 0.02
                )
                if step == 0:
                    self.assertIsNone(reason, pose)
                if reason:
                    self.assertNotEqual(reason, "fall", pose)
                    break
                stepped = runner.step(step * 0.02 + 0.01)
                self.assertNotEqual(stepped["reason"], "fall", pose)
            report = runner.report()
            self.assertEqual(report["pose"], pose)
            self.assertFalse(report["fall_check"])
            self.assertNotEqual(report["reason"], "fall")

    def test_push_is_applied_only_inside_its_window(self):
        plant = Plant("standing")
        plant.set_push(
            {"body": "pelvis", "force": [10.0, 0.0, 0.0], "start": 0.02, "duration": 0.02}
        )
        plant.step_physics()
        self.assertEqual(float(plant.data.xfrc_applied[plant.base, 0]), 0.0)
        plant.data.time = 0.02
        plant.step_physics()
        self.assertEqual(float(plant.data.xfrc_applied[plant.base, 0]), 10.0)
        plant.data.time = 0.04
        plant.step_physics()
        self.assertEqual(float(plant.data.xfrc_applied[plant.base, 0]), 0.0)

    def test_named_push_writes_one_report_case(self):
        from battery import run_battery

        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(SystemExit):
                run_battery(tmp, poses=["standing"], push="sideways")
            summary = run_battery(tmp, 0.2, poses=["seated"], push="none")
            self.assertTrue((Path(tmp) / "report.json").is_file())
        self.assertEqual(len(summary["cases"]), 1)
        self.assertEqual(summary["cases"][0]["pose"], "seated")
        self.assertEqual(summary["cases"][0]["push"], "none")
        self.assertEqual(summary["status"], summary["cases"][0]["status"])

    def test_example_command_files(self):
        from circle import load_command

        stand = load_command(ROOT / "examples/stand.yaml")
        circle = load_command(ROOT / "examples/circle.yaml")
        self.assertTrue(np.allclose(stand, [0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(circle, [0.45, 0.0, 0.9]))

    def test_circle_battery_uses_the_walker_gait(self):
        model = ROOT / "examples/walker.onnx"
        if not model.is_file():
            self.skipTest("cached walker weights are not on this machine")
        from battery import run_case

        report = run_case("standing", None, 0.2, circle=True)
        self.assertEqual(report["status"], "completed")
        self.assertFalse(report["checks"]["command_limits"])
        self.assertGreaterEqual(report["commands"], 5)
        self.assertGreater(report["motion"]["final_position"][2], 0.5)

    def test_flat_ground_pose_push_uses_the_plant(self):
        from battery import run_case

        report = run_case(
            "standing",
            {"body": "pelvis", "force": [10.0, 0.0, 0.0], "start": 0.02, "duration": 0.02},
            0.2,
        )
        self.assertEqual(report["pose"], "standing")
        self.assertGreaterEqual(report["commands"], 1)
        self.assertIn(report["status"], ("within_limits", "violation"))

    def test_standing_fall_still_latches(self):
        runner = Runner("standing")
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        runner.plant.data.qpos[2] = 0.1
        mujoco.mj_forward(runner.plant.model, runner.plant.data)
        runner.step(0.01)
        self.assertEqual(runner.reason, "fall")

    def test_torn_live_line_resyncs_instead_of_raising(self):
        from live import LiveSession

        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            live = directory / "live.jsonl"
            session = LiveSession(
                directory, live, directory / "start", directory / "report.json",
                "standing", None,
            )
            zeros = ",".join(["0"] * session.model.nq)
            live.write_text('{"t": 0.0, "qpos": [' + zeros + "]}\n, \"qpos\"\n")
            body = session.snapshot(0)
            self.assertTrue(body["reset"])
            self.assertEqual(body["frames"], [])
            live.write_text('{"t": 0.02, "qpos": [' + zeros + "]}\n")
            body = session.snapshot(0)
            self.assertEqual(len(body["frames"]), 1)
            self.assertAlmostEqual(body["frames"][0]["time"], 0.02)


if __name__ == "__main__":
    unittest.main()
