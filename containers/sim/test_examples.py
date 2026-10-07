"""Real ONNX rollouts and the example's ROS command contract."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [
    str(ROOT / "containers/sim"),
    str(ROOT / "containers/examples"),
    str(ROOT),
    str(ROOT / "robot_safety"),
    str(ROOT / "packages/policy_safety/src"),
]

from controllers import ExamplePolicy
from local import run
from plant import Plant
from prepare import prepare
from runner import Runner
from view import export_replay


class ExampleTests(unittest.TestCase):
    def test_walker_is_rejected_when_profile_gains_exceed_torque_limit(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = run("walker", tmp, 10, safety_checks=True)
        self.assertTrue(report["checks"]["profile_gains"])
        self.assertGreater(report["commands"], 0)
        self.assertLess(report["commands"], 500)
        self.assertEqual(report["reason"], "joint_torque_limit")
        self.assertEqual(report["status"], "violation")
        self.assertEqual(report["detail"]["joint"], "right_hip_pitch")
        self.assertGreater(report["detail"]["value"], report["detail"]["limit"])
        self.assertIsNotNone(report["stop_time_seconds"])

    def test_walker_makes_forward_progress_with_checks_disabled(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = run("walker", tmp, 10, safety_checks=False)
        self.assertEqual(report["status"], "completed")
        motion = report["motion"]
        self.assertGreater(
            motion["final_position"][0] - motion["start_position"][0], 3.0
        )
        self.assertFalse(motion["fell"])
        self.assertTrue(motion["upright_at_end"])

    def test_stairs_policy_climbs_unchecked_and_rejects_excess_profile_torque(
        self,
    ):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            off = run("stairs", root / "off", 20, safety_checks=False)
            self.assertEqual(off["status"], "completed")
            self.assertEqual(off["commands"], 1000)
            self.assertFalse(off["motion"]["fell"])
            self.assertEqual(off["motion"]["stairs"]["distinct_treads"], 3)
            self.assertTrue(off["motion"]["stairs"]["climbed_stairs"])
            self.assertTrue(off["motion"]["stairs"]["finished_upright"])
            on = run("stairs", root / "on", 0.5, safety_checks=True)
            self.assertEqual(on["status"], "violation")
            self.assertEqual(on["reason"], "joint_torque_limit")
            self.assertEqual(on["detail"]["joint"], "right_hip_pitch")
            self.assertGreater(on["detail"]["value"], on["detail"]["limit"])
            self.assertGreater(on["commands"], 0)
            self.assertLess(on["commands"], 25)
            view = export_replay(root / "off/trace.npz", root / "view")
            motion = json.loads((view / "motion.json").read_text())
            self.assertIn("Safety checks: OFF", motion["summary"])
            self.assertTrue(any(g["name"] == "stair_3" for g in motion["geoms"]))

    def test_balance_real_policy_and_safety_rejection(self):
        model = Path(
            os.environ.get(
                "HIM_BALANCE_ONNX",
                str(
                    ROOT
                    / "examples/policies/balance/GR00T-WholeBodyControl-Balance.onnx"
                ),
            )
        )
        self.assertTrue(model.is_file(), "balance weights are missing")
        with tempfile.TemporaryDirectory() as tmp:
            off = run("balance", Path(tmp) / "off", 10, safety_checks=False)
            self.assertEqual(off["status"], "completed")
            self.assertEqual(off["commands"], 500)
            self.assertFalse(off["motion"]["fell"])
            self.assertTrue(off["motion"]["upright_at_end"])
            on = run("balance", Path(tmp) / "on", 0.2, safety_checks=True)
            self.assertEqual(on["status"], "within_limits")
            self.assertIsNone(on["reason"])

    def test_first_command_holds_pose_and_policy_does_not_modify_state(self):
        for name in ("walker", "balance"):
            with self.subTest(policy=name):
                project = prepare(name)
                bundle = project / "container/project"
                policy = ExamplePolicy(name, bundle)
                with patch.dict(os.environ, {"G1_JOINT_PROFILE": str(bundle / "joint_profile.json")}):
                    runner = Runner(scene=project / "environment/scene.xml")
                state = runner.feedback(0.0)
                before = json.dumps(state, sort_keys=True)
                frame = policy.command(state)
                self.assertEqual(frame["q"], state["q"])
                self.assertIsNone(runner.command(frame, 0.0))
                np.testing.assert_allclose(runner.active["kp"], policy.kp)
                np.testing.assert_allclose(runner.active["kd"], policy.kd)
                policy.command(state)
                self.assertEqual(json.dumps(state, sort_keys=True), before)

    def test_prepared_environment_and_image_context_are_portable(self):
        with tempfile.TemporaryDirectory() as tmp:
            project = prepare("stairs", Path(tmp) / "submitted")
            plant = Plant(scene=project / "environment/scene.xml")
            self.assertGreaterEqual(plant.model.geom("stair_3").id, 0)
            context = project / "container"
            self.assertTrue(
                (
                    context / "interfaces/him_safety_interfaces/msg/JointCommand.msg"
                ).is_file()
            )
            self.assertTrue((context / "project/walker.onnx.data").is_file())
            import onnx

            prepared_model = onnx.load(
                context / "project/walker.onnx", load_external_data=False
            )
            self.assertLessEqual(prepared_model.ir_version, 9)
            env = {**os.environ, "PYTHONPATH": str(context)}
            proc = subprocess.run(
                [
                    sys.executable,
                    "-c",
                    "from controllers import ExamplePolicy; p=ExamplePolicy('stairs', 'project'); print(p.kp.shape)",
                ],
                cwd=context,
                env=env,
                capture_output=True,
                text=True,
                timeout=20,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn("(29,)", proc.stdout)

    def test_local_preparation_skips_container_sources(self):
        with tempfile.TemporaryDirectory() as tmp:
            project = prepare("walker", tmp, container=False)
            self.assertFalse((project / "container/Dockerfile").exists())
            self.assertFalse((project / "container/interfaces").exists())
            self.assertFalse((project / "container/policy_safety").exists())
            policy = ExamplePolicy("walker", project / "container/project")
            runner = Runner(scene=project / "environment/scene.xml")
            state = runner.feedback(0.0)
            self.assertIsNone(runner.command(policy.command(state), 0.0))

    def test_local_cli_without_docker_and_failed_run_does_not_replay_old_trace(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "run with spaces"
            proc = subprocess.run(
                [
                    str(ROOT / "sim.sh"),
                    "--example",
                    "walker",
                    "--local",
                    "--seconds",
                    "0.04",
                    "--no-safety-checks",
                    "--output",
                    str(output),
                ],
                cwd="/tmp",
                env={**os.environ, "G1_TASK_COMMAND": "[0.1,0,0]"},
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            report = json.loads((output / "report.json").read_text())
            self.assertEqual(report["execution"], "local")
            self.assertEqual(report["task_command"], [0.1, 0, 0])
            self.assertFalse(report["safety_checks"])
            with self.assertRaises(ValueError):
                run("walker", output, scene=Path(tmp) / "absent.xml")
            self.assertFalse((output / "trace.npz").exists())
            self.assertFalse((output / "report.json").exists())

    def test_desktop_sim_self_test_and_local_example(self):
        help_proc = subprocess.run(
            [str(ROOT / "desktop_sim.sh"), "--help"],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        self.assertEqual(help_proc.returncode, 0, help_proc.stderr)
        self.assertIn("--local", help_proc.stdout)
        env = {**os.environ, "G1_SIM_PYTHON": sys.executable}
        with tempfile.TemporaryDirectory() as tmp:
            hold = Path(tmp) / "self-test"
            proc = subprocess.run(
                [
                    str(ROOT / "desktop_sim.sh"),
                    "--self-test",
                    "--seconds",
                    "0.04",
                    "--output",
                    str(hold),
                ],
                env=env,
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
            report = json.loads((hold / "report.json").read_text())
            self.assertEqual(report["execution"], "local")
            self.assertEqual(report["mode"], "self_test")
            self.assertEqual(report["status"], "within_limits")
            self.assertGreaterEqual(report["commands"], 1)
            output = Path(tmp) / "walker"
            proc = subprocess.run(
                [
                    str(ROOT / "desktop_sim.sh"),
                    "--example",
                    "walker",
                    "--local",
                    "--seconds",
                    "0.04",
                    "--no-safety-checks",
                    "--output",
                    str(output),
                ],
                env=env,
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
            report = json.loads((output / "report.json").read_text())
            self.assertEqual(report["execution"], "local")
            self.assertEqual(report["policy"], "walker")
            circle = Path(tmp) / "circle"
            proc = subprocess.run(
                [
                    str(ROOT / "desktop_sim.sh"),
                    "--circle",
                    "--seconds",
                    "0.04",
                    "--output",
                    str(circle),
                ],
                env=env,
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
            report = json.loads((circle / "report.json").read_text())
            self.assertEqual(report["control_path"], "direct")
            self.assertEqual(report["status"], "completed")
            self.assertFalse(report["checks"]["command_protocol"])
            for directory in (hold, output, circle):
                with np.load(directory / "trace.npz") as trace:
                    self.assertGreaterEqual(len(trace["time"]), 3)
                    self.assertGreater(trace["time"][-1], 0)

    def test_simulation_state_has_required_frames_and_bounds(self):
        runner = Runner()
        state = runner.feedback(0.0)
        np.testing.assert_allclose(state["base_velocity"], runner.plant.data.qvel[:6])
        self.assertEqual(len(state["position"]), 3)
        self.assertEqual(len(state["ctrl_low"]), 29)
        self.assertEqual(state["session_id"], runner.session_id)

    def test_missing_balance_weights_report_execution_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            env = {**os.environ, "HIM_BALANCE_ONNX": str(Path(tmp) / "missing.onnx")}
            result = subprocess.run(
                [
                    str(ROOT / "sim.sh"),
                    "--example",
                    "balance",
                    "--local",
                    "--output",
                    tmp,
                ],
                env=env,
                capture_output=True,
                text=True,
                timeout=15,
                check=False,
            )
            self.assertEqual(result.returncode, 2)
            report = json.loads((Path(tmp) / "report.json").read_text())
            self.assertEqual(report["status"], "error")
            self.assertIn("Balance weights missing", report["detail"])
            self.assertFalse((Path(tmp) / "trace.npz").exists())


if __name__ == "__main__":
    unittest.main()
