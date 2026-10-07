"""Custom scenes, portable replay, and launcher argument tests."""

import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [
    str(ROOT / "packages/policy_safety/src"),
    str(ROOT / "robot_safety"),
    str(ROOT / "containers/sim"),
]

from plant import Plant
from runner import Runner
from view import export_replay

ROBOT = ROOT / "packages/policy_safety/src/policy_safety/robots/g1"


class SceneTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="g1 scene ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.scene = self.root / "scene.xml"
        shutil.copy2(ROBOT / "g1_robot.xml", self.root / "g1_robot.xml")
        (self.root / "third_party").symlink_to(
            ROBOT / "third_party", target_is_directory=True
        )
        (self.root / "obstacles.xml").write_text("""<mujoco><worldbody>
  <geom name="safety_floor" type="plane" size="0 0 0.05" friction="0.7"/>
  <geom name="obstacle" type="box" size="0.2 0.3 0.4" pos="2 0 0.4" rgba="0.8 0.2 0.1 1"/>
</worldbody></mujoco>""")
        self.scene.write_text("""<mujoco model="submitted_environment">
  <include file="g1_robot.xml"/>
  <include file="obstacles.xml"/>
  <option gravity="0 0 -4" timestep="0.01"/>
</mujoco>""")

    def test_custom_scene_resolves_includes_and_assets(self):
        plant = Plant(scene=self.scene)
        self.assertEqual(plant.scene, self.scene.resolve())
        np.testing.assert_allclose(plant.model.opt.gravity, [0, 0, -4])
        np.testing.assert_allclose(plant.model.geom("obstacle").size, [0.2, 0.3, 0.4])
        self.assertEqual(plant.model.opt.timestep, 0.002)
        self.assertEqual(plant.q().shape, (29,))
        self.assertGreater(plant.default_kp.min(), 0)

    def test_extra_free_body_before_robot_uses_correct_base_addresses(self):
        robot_path = self.root / "g1_robot.xml"
        robot = ET.parse(robot_path)
        key = robot.find("keyframe/key")
        key.set("qpos", "3 0 0.1 1 0 0 0 " + key.get("qpos"))
        robot.write(robot_path)
        self.scene.write_text("""<mujoco>
  <worldbody><body name="prop" pos="3 0 0.1"><freejoint/>
    <geom name="prop_geom" type="sphere" size="0.1"/>
  </body></worldbody>
  <include file="g1_robot.xml"/><include file="obstacles.xml"/>
</mujoco>""")
        plant = Plant("supine", scene=self.scene)
        self.assertEqual(plant.model.nq, 43)
        self.assertEqual(plant.base_qadr, 7)
        self.assertEqual(plant.base_vadr, 6)
        np.testing.assert_allclose(plant.data.qpos[:7], [3, 0, 0.1, 1, 0, 0, 0])
        self.assertLess(abs(plant.state()["upright"]), 0.01)

    def test_missing_scene_and_incompatible_model_fail(self):
        with self.assertRaisesRegex(ValueError, "scene does not exist"):
            Plant(scene=self.root / "absent.xml")
        self.scene.write_text("<mujoco/>")
        with self.assertRaises((KeyError, ValueError)):
            Plant(scene=self.scene)

    def test_counterfactual_requires_named_floor(self):
        obstacle = self.root / "obstacles.xml"
        obstacle.write_text(obstacle.read_text().replace("safety_floor", "ground"))
        Plant(scene=self.scene)
        with self.assertRaisesRegex(ValueError, "safety_floor"):
            Plant("supine", scene=self.scene)

    def test_replay_retains_scene_after_original_files_are_removed(self):
        runner = Runner(record_trace=True, scene=self.scene, safety_checks=False)
        runner.step(0.0)
        trace = self.root / "trace.npz"
        with patch.dict(os.environ, {"G1_SIM_TRACE": str(trace)}):
            report = runner.report()
        self.assertEqual(report["scene"], str(self.scene))
        self.scene.unlink()
        (self.root / "g1_robot.xml").unlink()
        (self.root / "obstacles.xml").unlink()
        (self.root / "third_party").unlink()
        dest = export_replay(trace, self.root / "view")
        motion = json.loads((dest / "motion.json").read_text())
        obstacle = next(g for g in motion["geoms"] if g["name"] == "obstacle")
        self.assertEqual(obstacle["type"], "box")
        self.assertEqual(obstacle["size"], [0.2, 0.3, 0.4])
        self.assertEqual(len(obstacle["poses"]), 2)
        self.assertEqual(obstacle["poses"][0][9:], [2.0, 0.0, 0.4])
        self.assertTrue(any(g["type"] == "plane" for g in motion["geoms"]))
        self.assertTrue(any((dest / "meshes").glob("*.stl")))

    def test_legacy_trace_still_replays(self):
        runner = Runner(record_trace=True)
        trace = self.root / "old.npz"
        np.savez(trace, qpos=np.vstack(runner.trace_qpos), time=runner.trace_time)
        self.assertTrue(
            (export_replay(trace, self.root / "view") / "index.html").exists()
        )

    def test_replay_includes_primitives_and_heightfield(self):
        self.scene.write_text(
            self.scene.read_text().replace(
                "</mujoco>",
                """
  <asset><hfield name="terrain" nrow="2" ncol="2" size="1 1 2 0.1"/></asset>
  <worldbody>
    <geom name="terrain" type="hfield" hfield="terrain" pos="4 0 0"/>
    <geom name="ball" type="sphere" size="0.1" pos="2 2 1"/>
    <geom name="oval" type="ellipsoid" size="0.1 0.2 0.3" pos="3 2 1"/>
    <geom name="capsule" type="capsule" size="0.1 0.3" pos="4 2 1"/>
    <geom name="cylinder" type="cylinder" size="0.1 0.3" pos="5 2 1"/>
  </worldbody>
</mujoco>""",
            )
        )
        runner = Runner(record_trace=True, scene=self.scene)
        runner.plant.model.hfield_data[:] = [0.0, 0.25, 0.75, 1.0]
        trace = self.root / "terrain.npz"
        with patch.dict(os.environ, {"G1_SIM_TRACE": str(trace)}):
            runner.report()
        dest = export_replay(trace, self.root / "view")
        motion = json.loads((dest / "motion.json").read_text())
        types = {geom["type"] for geom in motion["geoms"]}
        self.assertTrue(
            {"mesh", "box", "sphere", "ellipsoid", "capsule", "cylinder", "plane"}
            <= types
        )
        terrain = next(g for g in motion["geoms"] if g["name"] == "terrain")
        stl = (dest / terrain["file"]).read_bytes()
        self.assertEqual(struct.unpack_from("<I", stl, 80)[0], 2)
        triangle = struct.unpack_from("<12fH", stl, 84)
        np.testing.assert_allclose(triangle[3:12], [-1, -1, 0, 1, -1, 0.5, -1, 1, 1.5])

    def test_plant_process_receives_scene_and_safety_option(self):
        env = {
            **os.environ,
            "PYTHONPATH": os.pathsep.join(sys.path[:3]),
            "G1_SIM_SCENE": str(self.scene),
            "G1_SIM_SAFETY_CHECKS": "0",
            "G1_SIM_TRACE": str(self.root / "process_trace.npz"),
        }
        proc = subprocess.run(
            [sys.executable, str(ROOT / "containers/sim/plant_server.py")],
            input=json.dumps({"op": "report"}) + "\n",
            text=True,
            capture_output=True,
            env=env,
            cwd="/tmp",
            timeout=20,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        report = json.loads(proc.stdout)
        self.assertEqual(report["scene"], str(self.scene))
        self.assertFalse(report["safety_checks"])
        self.assertFalse(report["fall_check"])
        self.assertEqual(report["status"], "no_command")

    def test_launcher_help_and_missing_scene_do_not_need_root(self):
        script = str(ROOT / "sim.sh")
        help_result = subprocess.run(
            [script, "--help"], capture_output=True, text=True, timeout=5, check=False
        )
        self.assertEqual(help_result.returncode, 0)
        self.assertIn("--no-safety-checks", help_result.stdout)
        result = subprocess.run(
            [script, "--scene", str(self.root / "missing.xml")],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("not a readable file", result.stderr)


if __name__ == "__main__":
    unittest.main()
