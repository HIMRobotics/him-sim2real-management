"""Prepare a portable example image context and matching MuJoCo scene."""

import argparse
import hashlib
import json
import os
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from robot_safety.balance_policy import LEG_KD, LEG_KP
from robot_safety.hardware_constants import G1_29_JOINT_NAMES

# Every example starts in GR00T startup balance. This is the crouch it settles
# into on flat ground on this model, mirrored left to right. Started from the
# policies' own defaults, the knees sag 3 cm and the robot sways back a step
# and turns.
STANCE_LEFT_LEG = [-0.412, 0.044, 0.015, 0.741, -0.325, -0.007]
STANCE = STANCE_LEFT_LEG + [-0.412, -0.044, -0.015, 0.741, -0.325, 0.007] + [0.0] * 17

EXAMPLES = ("walker", "balance", "holosoma")

# Simulation presets reuse the existing controller and model bundle.
SCENARIOS = {
    "walker/thrash": ("walker", "supine", 0, "enforce"),
    "walker/thrash-monitor": ("walker", "supine", 0, "monitor"),
}


def run_settings(name):
    """Resolve an example into controller, pose, checks and detector mode."""
    if name in SCENARIOS:
        return SCENARIOS[name]
    if name in EXAMPLES:
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
    if name not in EXAMPLES:
        raise ValueError("unknown example")
    root = Path(destination or ROOT / ".cache/sim_examples" / name).resolve()
    project = root / "container/project"
    if name == "holosoma":
        from robot_safety.holosoma_policy import HolosomaLocomotionPolicy, holosoma_weights

        copy_file(holosoma_weights(), project / "holosoma.onnx")
        policy = HolosomaLocomotionPolicy(project / "holosoma.onnx")
        gains = list(zip(policy.kp.tolist(), policy.kd.tolist()))
    elif name == "balance":
        model = Path(
            os.environ.get(
                "HIM_BALANCE_ONNX",
                str(ROOT / "containers/examples/policies/balance/GR00T-WholeBodyControl-Balance.onnx"),
            )
        )
        if not model.is_file():
            raise ValueError(
                "Balance weights missing. Set HIM_BALANCE_ONNX to GR00T-WholeBodyControl-Balance.onnx."
            )
        copy_file(model, project / "balance.onnx")
        gains = list(zip(list(LEG_KP) + [40.] * 14, list(LEG_KD) + [1.] * 14))
    else:
        bundle = ROOT / "containers/examples/policies/walker"
        model_names = ("walker.onnx", "balance.onnx", "LICENSE", "provenance.json")
        missing = [
            filename
            for filename in (*model_names, "walker.yaml")
            if not (bundle / filename).is_file()
        ]
        if missing:
            raise ValueError(
                f"Policy bundle missing in {bundle}: "
                + ", ".join(missing)
            )
        (project / "walker.onnx.data").unlink(missing_ok=True)
        for filename in model_names:
            copy_file(bundle / filename, project / filename)
        copy_file(bundle / "walker.yaml", project / "policy.yaml")
        cfg = yaml.safe_load((project / "policy.yaml").read_text())
        defaults = [j["default_position"] for j in cfg["joints"]]
        names = [j["name"].replace("_joint", "") for j in cfg["joints"]]
        if names != list(G1_29_JOINT_NAMES):
            raise ValueError("policy joints must follow the G1 SDK order")
        gains = [(j["kp"], j["kd"]) for j in cfg["joints"]]
    profile = json.loads((ROOT / "robot_safety/profiles/g1_type5.template.json").read_text())
    for joint, (kp, kd) in zip(G1_29_JOINT_NAMES, gains):
        profile["joints"][joint].update(kp=float(kp), kd=float(kd))
    (project / "joint_profile.json").write_text(json.dumps(profile, indent=2) + "\n")
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(project.iterdir()) if p.is_file() and p.name != "hashes.json"}
    (project / "hashes.json").write_text(json.dumps(hashes, indent=2) + "\n")
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
    # Start above the support. The plant lowers the robot until the feet touch.
    start = [0, 0, 0.85, 1, 0, 0, 0]
    key.set("qpos", " ".join(map(str, start + STANCE)))
    key.set("ctrl", " ".join(map(str, STANCE)))
    tree.write(env / "g1_robot.xml")
    (env / "scene.xml").write_text("""<mujoco model="example_flat">
  <include file="g1_robot.xml"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody><geom name="safety_floor" type="plane" size="0 0 0.05" friction="1.15"/></worldbody>
</mujoco>\n""")
    if not container:
        return root
    context = root / "container"
    for filename in ("controllers.py", "client.py", "trial.py", "command.py", "Dockerfile", "entrypoint.sh"):
        copy_file(ROOT / "containers/examples" / filename, context / filename)
    for filename in ("__init__.py", "balance_policy.py", "hardware_constants.py", "holosoma_policy.py"):
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
    parser.add_argument("example", choices=(*EXAMPLES, *SCENARIOS))
    parser.add_argument("--assets-only", action="store_true")
    parser.add_argument("--settings", action="store_true", help="Print simulation preset settings.")
    args = parser.parse_args()
    if args.settings:
        print(*run_settings(args.example))
    else:
        print(prepare(run_settings(args.example)[0], container=not args.assets_only))
