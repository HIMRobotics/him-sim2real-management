"""Prepare a portable example image context and matching MuJoCo scene."""

import argparse
import os
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from robot_safety.balance_policy import DEFAULT_JOINT_POSITIONS


def copy_file(source, target):
    source, target = Path(source), Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.exists() or source.stat().st_mtime_ns != target.stat().st_mtime_ns:
        shutil.copy2(source, target)
    return str(target)


def prepare(name, destination=None):
    """Copy only the files needed by this example, without running Docker."""
    if name not in ("stairs", "walker", "balance"):
        raise ValueError("unknown example")
    root = Path(destination or ROOT / ".cache/sim_examples" / name).resolve()
    project = root / "container/project"
    if name == "balance":
        model = Path(
            os.environ.get(
                "HIM_BALANCE_ONNX",
                str(ROOT / ".cache/groot_balance/GR00T-WholeBodyControl-Balance.onnx"),
            )
        )
        if not model.is_file():
            raise ValueError(
                "Balance weights missing. Set HIM_BALANCE_ONNX to GR00T-WholeBodyControl-Balance.onnx."
            )
        copy_file(model, project / "balance.onnx")
        defaults = DEFAULT_JOINT_POSITIONS.tolist()
    else:
        missing = [
            filename
            for filename in ("walker.onnx", "walker.onnx.data", "policy.yaml")
            if not (ROOT / "examples" / filename).is_file()
        ]
        if missing:
            raise ValueError(
                "examples/ is the balance submission, not the walker bundle. Missing "
                + ", ".join(missing)
            )
        for filename in ("walker.onnx", "walker.onnx.data", "policy.yaml"):
            copy_file(ROOT / "examples" / filename, project / filename)
        cfg = yaml.safe_load((project / "policy.yaml").read_text())
        defaults = [j["default_position"] for j in cfg["joints"]]
    env = root / "environment"
    robot = ROOT / "packages/policy_safety/src/policy_safety/robots/g1"
    shutil.copytree(
        robot / "third_party",
        env / "third_party",
        dirs_exist_ok=True,
        copy_function=copy_file,
    )
    copy_file(robot / "THIRD_PARTY_NOTICES.md", env / "THIRD_PARTY_NOTICES.md")
    tree = ET.parse(robot / "g1_robot.xml")
    key = tree.find("keyframe/key[@name='stand']")
    start = (
        [0.002739233746429086, -0.004604265724722594, 0.79, 1, 0, 0, 0]
        if name == "stairs"
        else [0, 0, 0.79, 1, 0, 0, 0]
    )
    key.set("qpos", " ".join(map(str, start + defaults)))
    key.set("ctrl", " ".join(map(str, defaults)))
    tree.write(env / "g1_robot.xml")
    if name == "stairs":
        copy_file(ROOT / "containers/examples/stairs.xml", env / "scene.xml")
    else:
        (env / "scene.xml").write_text("""<mujoco model="example_flat">
  <include file="g1_robot.xml"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody><geom name="safety_floor" type="plane" size="0 0 0.05" friction="1.15"/></worldbody>
</mujoco>\n""")
    context = root / "container"
    for filename in ("controllers.py", "client.py", "Dockerfile", "entrypoint.sh"):
        copy_file(ROOT / "containers/examples" / filename, context / filename)
    for filename in ("__init__.py", "balance_policy.py", "hardware_constants.py"):
        copy_file(ROOT / "robot_safety" / filename, context / "robot_safety" / filename)
    for filename in ("__init__.py", "adapter.py", "config.py"):
        copy_file(
            ROOT / "packages/policy_safety/src/policy_safety" / filename,
            context / "policy_safety" / filename,
        )
    shutil.copytree(
        ROOT / "ros2/him_safety_interfaces",
        context / "interfaces/him_safety_interfaces",
        dirs_exist_ok=True,
    )
    return root


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("example", choices=("stairs", "walker", "balance"))
    args = parser.parse_args()
    print(prepare(args.example))
