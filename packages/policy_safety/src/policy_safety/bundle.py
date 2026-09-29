"""Prepare a portable example without executing submission code."""

import hashlib
import json
import shutil
from pathlib import Path

from .config import read_json

GAINS = {
    "hip_roll": (99.098, 6.309),
    "knee": (99.098, 6.309),
    "hip_pitch": (40.179, 2.558),
    "hip_yaw": (40.179, 2.558),
    "waist_yaw": (40.179, 2.558),
    "ankle": (28.501, 1.814),
    "waist_pitch": (28.501, 1.814),
    "waist_roll": (28.501, 1.814),
    "wrist_pitch": (16.778, 1.068),
    "wrist_yaw": (16.778, 1.068),
    "shoulder": (14.251, 0.907),
    "elbow": (14.251, 0.907),
    "wrist_roll": (14.251, 0.907),
}


def digest(path):
    hasher = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def model_files(model_path):
    """Check external tensor references before ONNX Runtime opens them."""
    import onnx
    from onnx.external_data_helper import _get_all_tensors

    model_path = Path(model_path).resolve()
    graph = onnx.load(str(model_path), load_external_data=False)
    paths = {model_path}
    for tensor in _get_all_tensors(graph):
        if tensor.data_location == onnx.TensorProto.EXTERNAL:
            location = {item.key: item.value for item in tensor.external_data}.get(
                "location"
            )
            if not location or Path(location).is_absolute():
                raise ValueError("external weights require a relative location")
            target = (model_path.parent / location).resolve()
            if not target.is_relative_to(model_path.parent) or not target.is_file():
                raise ValueError(
                    "external weights must exist inside the model directory"
                )
            paths.add(target)
    return sorted(paths)


def prepare_g1(submission, output):
    submission, output = Path(submission).resolve(), Path(output).resolve()
    cfg = read_json(submission / "policies/model_config.json")
    names = cfg["joint_names"]
    if len(names) != 29 or len(set(names)) != 29:
        raise ValueError("expected 29 unique joints")
    gains = [
        next(value for key, value in GAINS.items() if key in name) for name in names
    ]
    files = model_files(submission / "policies/walker.onnx")
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(submission / "g1_robot.xml", output / "g1_robot.xml")
    shutil.copytree(submission / "third_party", output / "third_party")
    shutil.copy2(
        submission / "THIRD_PARTY_NOTICES.md", output / "THIRD_PARTY_NOTICES.md"
    )
    shutil.copy2(
        submission / "policies/model_config.json", output / "source_model_config.json"
    )
    for source in files:
        target = output / source.relative_to(submission / "policies")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    (output / "scene.xml").write_text("""<mujoco model="g1_safety_flat">
  <include file="g1_robot.xml"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="safety_floor" type="plane" size="0 0 0.05" friction="1.15"/>
  </worldbody>
</mujoco>
""")
    model_paths = {output / f.relative_to(submission / "policies") for f in files}
    robot = {
        "version": 1,
        "contract": "g1-walker-99-v1",
        "normalization": "embedded",
        "scene": "scene.xml",
        "control_seconds": 0.02,
        "physics_seconds": 0.002,
        "joint_names": names,
        "actuator_names": names,
        "default_joint_positions": [cfg["default_joint_pos"][n] for n in names],
        "action_scales": [cfg["action_scales"][n] for n in names],
        "kp": [g[0] for g in gains],
        "kd": [g[1] for g in gains],
        "base_body": "pelvis",
        "floor_geom": "safety_floor",
        "standing_height": 0.79,
        "assets": [
            str(p.relative_to(output))
            for p in sorted(output.rglob("*"))
            if p.is_file() and p not in model_paths
        ],
    }
    (output / "robot.json").write_text(json.dumps(robot, indent=2) + "\n")
    return output / "robot.json"
