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

# Simulation presets reuse the existing controller and model bundle.
SCENARIOS = {
    "walker/thrash": ("walker", "supine", 0, "enforce"),
    "walker/thrash-monitor": ("walker", "supine", 0, "monitor"),
}


def run_settings(name):
    """Resolve an example into controller, pose, checks and detector mode."""
    if name in SCENARIOS:
        return SCENARIOS[name]
    if name in ("stairs", "walker", "balance"):
        return name, "standing", 1, "default"
    raise ValueError("unknown example: " + name)


def copy_file(source, target):
    source, target = Path(source), Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.exists() or source.stat().st_mtime_ns != target.stat().st_mtime_ns:
        shutil.copy2(source, target)
    return str(target)


def prepare(name, destination=None, *, container=True):
    """Copy only the files needed by this example, without running Docker."""
    if name not in ("stairs", "walker", "balance"):
        raise ValueError("unknown example")
    root = Path(destination or ROOT / ".cache/sim_examples" / name).resolve()
    project = root / "container/project"
    if name == "balance":
        model = Path(
            os.environ.get(
                "HIM_BALANCE_ONNX",
                str(ROOT / "examples/policies/balance/GR00T-WholeBodyControl-Balance.onnx"),
            )
        )
        if not model.is_file():
            raise ValueError(
                "Balance weights missing. Set HIM_BALANCE_ONNX to GR00T-WholeBodyControl-Balance.onnx."
            )
        copy_file(model, project / "balance.onnx")
        defaults = DEFAULT_JOINT_POSITIONS.tolist()
    else:
        bundle = ROOT / "examples/policies/walker"
        missing = [
            filename
            for filename in ("walker.onnx", "walker.onnx.data", "walker.yaml")
            if not (bundle / filename).is_file()
        ]
        if missing:
            raise ValueError(
                "Walker bundle missing in examples/policies/walker/: "
                + ", ".join(missing)
            )
        for filename in ("walker.onnx", "walker.onnx.data"):
            copy_file(bundle / filename, project / filename)
        copy_file(bundle / "walker.yaml", project / "policy.yaml")
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
    if not container:
        return root
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
    parser.add_argument("example", choices=("stairs", "walker", "balance", *SCENARIOS))
    parser.add_argument("--assets-only", action="store_true")
    parser.add_argument("--settings", action="store_true", help="Print simulation preset settings.")
    args = parser.parse_args()
    if args.settings:
        print(*run_settings(args.example))
    else:
        print(prepare(run_settings(args.example)[0], container=not args.assets_only))
