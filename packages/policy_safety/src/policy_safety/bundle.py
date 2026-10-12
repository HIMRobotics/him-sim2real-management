"""Prepare a portable screening bundle from a YAML plus ONNX policy."""

import hashlib
import json
import shutil
from pathlib import Path

ROBOT_DIR = Path(__file__).resolve().parent / "robots" / "g1"


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


def read_policy_yaml(directory):
    """Load the one policy YAML. Joint gains and scales come from that file."""
    import yaml

    directory = Path(directory)
    policies = []
    for path in sorted(directory.glob("*.yaml")):
        document = yaml.safe_load(path.read_text())
        if isinstance(document, dict) and any(
            key in document for key in ("contract", "model", "joints")
        ):
            policies.append(document)
    if len(policies) != 1:
        raise ValueError("policy directory needs exactly one policy YAML")
    data = policies[0]
    contracts = {"g1-walker-99-v1": "embedded", "nvidia-decoupled-wbc-516-v1": "explicit"}
    if data.get("contract") not in contracts or data.get("normalization") != contracts[data["contract"]]:
        raise ValueError("unsupported policy contract or normalization")
    joints = data.get("joints")
    if (
        not isinstance(joints, list)
        or len(joints) != 29
        or len({item["name"] for item in joints}) != 29
    ):
        raise ValueError("expected 29 unique joints")
    for item in joints:
        for key in ("name", "default_position", "action_scale", "kp", "kd"):
            if key not in item:
                raise ValueError(f"joint entry missing {key}")
        if float(item["action_scale"]) <= 0 or float(item["kp"]) <= 0 or float(item["kd"]) < 0:
            raise ValueError(f"invalid gains or scale for {item['name']}")
    model_name = data["model"]["file"]
    model_path = directory / model_name
    if not model_path.is_file():
        raise ValueError("policy YAML model file is missing")
    return data, model_path


def prepare_g1(submission, output):
    submission, output = Path(submission).resolve(), Path(output).resolve()
    cfg, model_path = read_policy_yaml(submission)
    joints = cfg["joints"]
    files = model_files(model_path)
    if cfg["contract"] == "nvidia-decoupled-wbc-516-v1":
        files += model_files(model_path.with_name("balance.onnx"))
        for name in ("LICENSE", "provenance.json"):
            source = submission / name
            if source.is_file():
                files.append(source)
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(ROBOT_DIR / "g1_robot.xml", output / "g1_robot.xml")
    shutil.copytree(ROBOT_DIR / "third_party", output / "third_party")
    shutil.copy2(ROBOT_DIR / "THIRD_PARTY_NOTICES.md", output / "THIRD_PARTY_NOTICES.md")
    for source in files:
        target = output / source.relative_to(submission)
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
    model_paths = {output / f.relative_to(submission) for f in files}
    names = [item["name"] for item in joints]
    robot = {
        "version": 1,
        "contract": cfg["contract"],
        "normalization": cfg["normalization"],
        "scene": "scene.xml",
        "control_seconds": float(cfg["control_seconds"]),
        "physics_seconds": float(cfg["physics_seconds"]),
        "joint_names": names,
        "actuator_names": names,
        "default_joint_positions": [float(item["default_position"]) for item in joints],
        "action_scales": [float(item["action_scale"]) for item in joints],
        "kp": [float(item["kp"]) for item in joints],
        "kd": [float(item["kd"]) for item in joints],
        "base_body": "pelvis",
        "floor_geom": "safety_floor",
        "standing_height": 0.80 if cfg["contract"] == "nvidia-decoupled-wbc-516-v1" else 0.79,
        "assets": [
            str(p.relative_to(output))
            for p in sorted(output.rglob("*"))
            if p.is_file() and p not in model_paths
        ],
    }
    (output / "robot.json").write_text(json.dumps(robot, indent=2) + "\n")
    return output / "robot.json"
