"""Write assets/actor.npz and assets/motion.npz for policy.py from a BeyondMimic ONNX.

    ../../.venv-policy-safety/bin/python export_policy.py \
        ../st_jzfwe_groot_robust/exported/policy.onnx

The ONNX holds the actor MLP (no observation normalizer) and the reference clip
as constants indexed by time_step. Joint metadata is in the policy's Isaac Lab
order; both outputs are written in Unitree motor order. Gains and action scales
are recomputed from the motor armature and effort limits because the metadata
rounds them to three decimals; they are checked against it and against
joint_profile.json.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from policy import ISAAC_ORDER, UNITREE_JOINTS

HERE = Path(__file__).resolve().parent
NATURAL_FREQ = 10 * 2 * np.pi
DAMPING_RATIO = 2.0
# (armature, effort limit Nm) per motor; ankles and waist roll/pitch use two 5020s.
MOTORS = {
    "7520_14": (0.01017752, 88.0), "7520_22": (0.025101925, 139.0),
    "5020": (0.003609725, 25.0), "5020_pair": (0.00721945, 50.0), "4010": (0.00425, 5.0),
}
MOTOR_OF = {
    "hip_pitch": "7520_14", "hip_yaw": "7520_14", "waist_yaw": "7520_14",
    "hip_roll": "7520_22", "knee": "7520_22",
    "ankle_pitch": "5020_pair", "ankle_roll": "5020_pair",
    "waist_roll": "5020_pair", "waist_pitch": "5020_pair",
    "shoulder_pitch": "5020", "shoulder_roll": "5020", "shoulder_yaw": "5020",
    "elbow": "5020", "wrist_roll": "5020", "wrist_pitch": "4010", "wrist_yaw": "4010",
}


def floats(meta, key):
    return np.array([float(v) for v in meta[key].split(",")])


def main():
    import onnx
    from onnx import numpy_helper

    parser = argparse.ArgumentParser()
    parser.add_argument("onnx")
    args = parser.parse_args()

    model = onnx.load(args.onnx)
    graph = model.graph
    meta = {p.key: p.value for p in model.metadata_props}
    weights = {t.name: numpy_helper.to_array(t) for t in graph.initializer}
    if [i.name for i in graph.input] != ["obs", "time_step"]:
        raise SystemExit("expected inputs obs and time_step")
    if any(n.op_type in ("Sub", "Div") for n in graph.node):
        raise SystemExit("the graph normalizes observations; export the normalizer too")
    if meta["observation_names"] != "command,motion_anchor_ori_b,base_ang_vel,joint_pos,joint_vel,actions":
        raise SystemExit("unexpected observation terms: " + meta["observation_names"])

    names = [n[:-len("_joint")] for n in meta["joint_names"].split(",")]
    if [UNITREE_JOINTS.index(n) for n in names] != ISAAC_ORDER.tolist():
        raise SystemExit("ONNX joint order differs from policy.ISAAC_ORDER")
    unitree = np.argsort(ISAAC_ORDER)

    armature, effort = np.array([MOTORS[MOTOR_OF[n.removeprefix("left_").removeprefix("right_")]]
                                 for n in UNITREE_JOINTS]).T
    kp = armature * NATURAL_FREQ ** 2
    kd = 2 * DAMPING_RATIO * armature * NATURAL_FREQ
    scale = 0.25 * effort / kp
    default_q = floats(meta, "default_joint_pos")[unitree]
    for key, exact in (("joint_stiffness", kp), ("joint_damping", kd), ("action_scale", scale)):
        if not np.allclose(floats(meta, key)[unitree], exact, atol=6e-4):
            raise SystemExit(f"{key} differs from the motor formula")
    profile = json.loads((HERE / "joint_profile.json").read_text())["joints"]
    for i, name in enumerate(UNITREE_JOINTS):
        if abs(profile[name]["kp"] - kp[i]) > 1e-4 or abs(profile[name]["kd"] - kd[i]) > 1e-4:
            raise SystemExit(f"joint_profile.json gains differ for {name}")

    constants = {}
    for node in graph.node:
        if node.op_type == "Constant":
            constants[node.output[0]] = numpy_helper.to_array(node.attribute[0].t)
    gathers = {n.output[0]: constants[n.input[0]] for n in graph.node if n.op_type == "Gather"}
    bodies = meta["body_names"].split(",")
    torso = bodies.index(meta["anchor_body_name"])
    joint_pos = gathers["joint_pos"][:, unitree]
    joint_vel = gathers["joint_vel"][:, unitree]
    torso_quat = gathers["body_quat_w"][:, torso]

    layers = {f"{kind}{i}": weights[f"actor.{i}.{name}"]
              for i in (0, 2, 4, 6) for kind, name in (("w", "weight"), ("b", "bias"))}
    if layers["w0"].shape != (512, 154) or layers["w6"].shape[0] != 29:
        raise SystemExit("unexpected actor shapes")
    np.savez(HERE / "assets" / "actor.npz", **layers, default_q=default_q,
             action_scale=scale, kp=kp, kd=kd)
    np.savez(HERE / "assets" / "motion.npz", joint_pos=joint_pos.astype(np.float32),
             joint_vel=joint_vel.astype(np.float32), torso_quat=torso_quat.astype(np.float32))
    print(f"wrote assets/actor.npz and assets/motion.npz: {len(joint_pos)} frames, "
          f"anchor {meta['anchor_body_name']}")


if __name__ == "__main__":
    main()
