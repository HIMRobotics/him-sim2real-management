"""Real ONNX rollouts and the example's ROS command contract."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import mujoco
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
    def test_walker_hip_torque_uses_policy_gains_and_does_not_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = run("walker", tmp, 1.0, safety_checks=True)
        self.assertTrue(report["checks"]["profile_gains"])
        self.assertGreater(report["commands"], 6)
        self.assertNotIn(report["reason"], (
            "joint_torque_limit", "joint_feedforward_torque_limit", "joint_velocity_limit"))
        hip = report["warnings"].get("right_hip_pitch", {})
        demands = [item for item in hip.get("limits", []) if item["torque_kind"] == "PD demand"]
        self.assertEqual(demands, [])

    def test_walker_makes_forward_progress_with_checks_disabled(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = run("walker", Path(tmp) / "off", 10, safety_checks=False)
            view = export_replay(Path(tmp) / "off/trace.npz", Path(tmp) / "view")
            summary = json.loads((view / "motion.json").read_text())["summary"]
        self.assertEqual(report["status"], "completed")
        motion = report["motion"]
        self.assertGreater(
            motion["final_position"][0] - motion["start_position"][0], 1.5
        )
        self.assertFalse(motion["fell"])
        self.assertTrue(motion["upright_at_end"])
        self.assertIn("Safety checks: OFF", summary)

    def test_balance_real_policy_and_safety_rejection(self):
        model = Path(
            os.environ.get(
                "HIM_BALANCE_ONNX",
                str(
                    ROOT
                    / "containers/examples/policies/balance/GR00T-WholeBodyControl-Balance.onnx"
                ),
            )
        )
        self.assertTrue(model.is_file(), "balance weights are missing")
        with tempfile.TemporaryDirectory() as tmp:
            off = run("balance", Path(tmp) / "off", 10, safety_checks=False)
            self.assertEqual(off["status"], "completed")
            self.assertEqual(off["commands"], 750)
            self.assertFalse(off["motion"]["fell"])
            self.assertTrue(off["motion"]["upright_at_end"])
            on = run("balance", Path(tmp) / "on", 0.2, safety_checks=True)
            self.assertEqual(on["status"], "within_limits")
            self.assertIsNone(on["reason"])

    def test_startup_warms_policy_without_modifying_state(self):
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
                self.assertFalse(state["control_ready"])
                self.assertEqual(len(frame["q"]), 29)
                self.assertTrue(np.isfinite(frame["q"]).all())
                self.assertIsNone(runner.command(frame, 0.0))
                np.testing.assert_allclose(runner.active["kp"], policy.kp)
                np.testing.assert_allclose(runner.active["kd"], policy.kd)
                policy.command(state)
                self.assertEqual(json.dumps(state, sort_keys=True), before)

    def test_prepared_environment_and_image_context_are_portable(self):
        with tempfile.TemporaryDirectory() as tmp:
            project = prepare("walker", Path(tmp) / "submitted")
            plant = Plant(scene=project / "environment/scene.xml")
            self.assertGreaterEqual(plant.model.geom("safety_floor").id, 0)
            context = project / "container"
            self.assertTrue(
                (
                    context / "interfaces/him_safety_interfaces/msg/JointCommand.msg"
                ).is_file()
            )
            self.assertTrue((context / "project/balance.onnx").is_file())
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
                    "from controllers import ExamplePolicy; p=ExamplePolicy('walker', 'project'); print(p.kp.shape)",
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

    def test_walker_rest_pose_starts_with_feet_on_the_floor(self):
        with tempfile.TemporaryDirectory() as tmp:
            project = prepare("walker", tmp, container=False)
            plant = Plant(scene=project / "environment/scene.xml", check_fall=False)
        self.assertLessEqual(float(plant.data.qpos[plant.base_qadr + 2]), 0.80)
        # Query distance directly because zero-clearance contacts depend on roundoff.
        distances = [
            mujoco.mj_geomDistance(plant.model, plant.data, geom, plant.floor, 1.0, None)
            for geom in plant.robot_geoms if plant.model.geom_contype[geom]
        ]
        self.assertLessEqual(min(distances), 1e-6)
        self.assertGreater(min(distances), -0.005)
        stock = Plant(check_fall=False)
        np.testing.assert_allclose(plant.data.qpos, stock.data.qpos, atol=1e-6)

    def test_image_runs_start_in_the_balance_stance_with_forearms_forward(self):
        plant = Plant(check_fall=False)
        self.assertAlmostEqual(float(plant.data.qpos[2]), 0.7495, places=3)
        np.testing.assert_allclose(plant.q()[15:], 0.0)
        for side in ("left", "right"):
            elbow = plant.data.xpos[plant.model.body(f"{side}_elbow_link").id]
            wrist = plant.data.xpos[plant.model.body(f"{side}_wrist_yaw_link").id]
            forearm = (wrist - elbow) / np.linalg.norm(wrist - elbow)
            self.assertGreater(forearm[0], 0.99)

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
            self.assertIsNone(report["command_file"])
            self.assertFalse(report["safety_checks"])
            with self.assertRaises(ValueError):
                run("walker", output, scene=Path(tmp) / "absent.xml")
            self.assertFalse((output / "trace.npz").exists())
            self.assertFalse((output / "report.json").exists())

    def test_walker_container_command_file_is_the_task(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "forward"
            proc = subprocess.run(
                [
                    str(ROOT / "sim.sh"),
                    "--example",
                    "walker",
                    "--command",
                    str(ROOT / "containers/examples/commands/walker/forward_stop.yaml"),
                    "--local",
                    "--seconds",
                    "0.04",
                    "--no-safety-checks",
                    "--output",
                    str(output),
                ],
                cwd="/tmp",
                capture_output=True,
                text=True,
                timeout=60,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            report = json.loads((output / "report.json").read_text())
            self.assertEqual(report["policy"], "walker")
            self.assertEqual(report["task_command"][:2], [0.5, 0.0])
            self.assertAlmostEqual(report["task_command"][2], 0.0, places=3)
            self.assertTrue(report["command_file"].endswith("forward_stop.yaml"))

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


def test_nvidia_walker_uses_imu_without_measured_linear_velocity(tmp_path):
    project = prepare('walker', tmp_path, container=False)
    policy = ExamplePolicy('walker', project / 'container/project')
    state = Runner(scene=project / 'environment/scene.xml').feedback(0.0)
    del state['base_velocity']
    policy.command(state)
    frame = policy.command(state)
    assert len(frame['q']) == 29
    assert np.isfinite(frame['q']).all()
