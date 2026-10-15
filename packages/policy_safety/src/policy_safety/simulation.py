"""Headless simulator with complete branch state and explicit joint mapping."""

import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from thrash import ThrashConfig, ThrashDetector

from .config import positive, read_json, vector


def validate_assets(root, robot):
    """Keep XML dependencies inside the fingerprinted portable bundle."""
    assets = {(root / name).resolve() for name in robot["assets"]}
    scene = (root / robot["scene"]).resolve()
    if scene not in assets or any(
        not p.is_relative_to(root) or not p.is_file() for p in assets
    ):
        raise ValueError("scene and assets must exist inside the bundle manifest")
    pending, visited = [scene], set()
    mesh_directory = Path(".")
    texture_directory = Path(".")
    while pending:
        path = pending.pop(0)
        if path in visited:
            continue
        visited.add(path)
        tree = ET.parse(path)
        for compiler in tree.iter("compiler"):
            mesh_directory = Path(compiler.get("meshdir", str(mesh_directory)))
            texture_directory = Path(compiler.get("texturedir", str(texture_directory)))
        for element in tree.iter():
            location = element.get("file")
            if not location:
                continue
            directory = path.parent if element.tag == "include" else scene.parent
            if element.tag == "mesh":
                directory /= mesh_directory
            elif element.tag == "texture":
                directory /= texture_directory
            dependency = (directory / location).resolve()
            if dependency not in assets:
                raise ValueError(
                    f"XML dependency absent from bundle manifest: {location}"
                )
            if element.tag == "include":
                pending.append(dependency)
    required = {
        p.resolve()
        for p in root.rglob("*")
        if p.suffix.lower() in (".xml", ".stl", ".obj", ".msh", ".png", ".jpg")
    }
    if required - assets:
        raise ValueError("robot assets missing from manifest")
    return assets


class Simulation:
    def __init__(self, robot_path):
        self.path = Path(robot_path).resolve()
        self.robot = read_json(self.path)
        robot = self.robot
        self.assets = validate_assets(self.path.parent, robot)
        if robot["version"] != 1 or robot["contract"] not in (
            "g1-walker-99-v1", "nvidia-decoupled-wbc-516-v1"
        ):
            raise ValueError("unsupported robot contract")
        for name in ("control_seconds", "physics_seconds", "standing_height"):
            positive(robot[name], name)
        self.dt = robot["control_seconds"]
        self.physics_dt = robot["physics_seconds"]
        self.substeps = round(self.dt / self.physics_dt)
        if self.substeps < 1 or not np.isclose(
            self.substeps * self.physics_dt, self.dt, atol=1e-12, rtol=0
        ):
            raise ValueError(
                "control timestep must be an integer multiple of physics timestep"
            )
        self.model = mujoco.MjModel.from_xml_path(
            str(self.path.parent / robot["scene"])
        )
        m = self.model
        if not np.isclose(m.opt.timestep, self.physics_dt, atol=1e-12, rtol=0):
            raise ValueError("scene timestep differs from robot contract")
        if (
            m.nq != 36
            or m.nv != 35
            or m.nu != 29
            or m.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE
        ):
            raise ValueError("expected a free-base 29-joint G1")
        for field in ("joint_names", "actuator_names"):
            if len(robot[field]) != 29 or len(set(robot[field])) != 29:
                raise ValueError(f"{field} requires 29 unique names")
        self.joints = np.array([m.joint(n).id for n in robot["joint_names"]])
        self.actuators = np.array([m.actuator(n).id for n in robot["actuator_names"]])
        # Match each named actuator to the joint it drives.
        if np.any(m.actuator_trnid[self.actuators, 0] != self.joints):
            raise ValueError("actuator transmission does not match joint mapping")
        # Position control requires direct joint drive and no activation dynamics.
        # Check fixed gain, affine feedback bias and unit transmission gear.
        if (
            np.any(m.actuator_trntype[self.actuators] != mujoco.mjtTrn.mjTRN_JOINT)
            or np.any(m.actuator_dyntype[self.actuators] != mujoco.mjtDyn.mjDYN_NONE)
            or np.any(
                m.actuator_gaintype[self.actuators] != mujoco.mjtGain.mjGAIN_FIXED
            )
            or np.any(
                m.actuator_biastype[self.actuators] != mujoco.mjtBias.mjBIAS_AFFINE
            )
            or not np.allclose(m.actuator_gear[self.actuators], [1, 0, 0, 0, 0, 0])
        ):
            raise ValueError("walker requires direct position actuators with unit gear")
        if np.any(m.jnt_type[self.joints] != mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError("walker supports hinge joints only")
        if (
            not np.all(m.jnt_limited[self.joints])
            or not np.all(m.actuator_ctrllimited[self.actuators])
            or not np.all(m.jnt_actfrclimited[self.joints])
        ):
            raise ValueError(
                "joint, control and actuator-force limits must be explicit"
            )
        self.qadr, self.vadr = m.jnt_qposadr[self.joints], m.jnt_dofadr[self.joints]
        self.bounds = m.jnt_range[self.joints].copy()
        self.ctrl_bounds = m.actuator_ctrlrange[self.actuators].copy()
        self.torque_bounds = m.jnt_actfrcrange[self.joints].copy()
        kp, kd = vector(robot["kp"], 29, "kp"), vector(robot["kd"], 29, "kd")
        if np.any(kp <= 0) or np.any(kd <= 0):
            raise ValueError("gains must be positive")
        self.kp = kp
        self.kd = kd
        self.set_torque_scale(1.0)
        self.default = vector(robot["default_joint_positions"], 29, "default positions")
        if np.any(self.default < self.bounds[:, 0]) or np.any(
            self.default > self.bounds[:, 1]
        ):
            raise ValueError("default pose exceeds joint limits")
        self.base = m.body(robot["base_body"]).id
        self.floor = m.geom(robot["floor_geom"]).id
        if m.geom_type[self.floor] != mujoco.mjtGeom.mjGEOM_PLANE:
            raise ValueError(
                "counterfactual initialization requires a flat ground plane"
            )
        self.data = mujoco.MjData(m)
        self.state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        self.rng = np.random.default_rng(0)
        self.command_history = None
        self.thrash_detector = ThrashDetector(
            ThrashConfig(mode="enforce"), len(self.vadr), max_gap_secs=0.2
        )

    def set_torque_scale(self, scale):
        """Deliver this fraction of the intended position-controller torque."""
        if scale <= 0 or not np.isfinite(scale):
            raise ValueError("torque scale must be finite and positive")
        kp = self.kp * scale
        kd = self.kd * scale
        self.model.actuator_gainprm[self.actuators, 0] = kp
        self.model.actuator_biasprm[self.actuators, 1] = -kp
        self.model.actuator_biasprm[self.actuators, 2] = -kd

    def reset(self, seed, pose="standing"):
        self.thrash_detector.reset()
        self.command_history = None
        mujoco.mj_resetData(self.model, self.data)
        self.rng = np.random.default_rng(seed)
        d = self.data
        d.qpos[:3] = [*self.rng.uniform(-0.01, 0.01, 2), self.robot["standing_height"]]
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[self.qadr] = self.default
        if pose == "seated":
            for side in ("left", "right"):
                for joint, angle in (
                    ("hip_pitch", -1.45),
                    ("knee", 0.15),
                    ("ankle_pitch", 0.0),
                ):
                    d.qpos[self.model.joint(f"{side}_{joint}_joint").qposadr[0]] = angle
        elif pose in ("supine", "prone", "left_side", "right_side"):
            root = np.sqrt(0.5)
            d.qpos[3:7] = {
                "supine": [root, 0, -root, 0],
                "prone": [root, 0, root, 0],
                "left_side": [root, -root, 0, 0],
                "right_side": [root, root, 0, 0],
            }[pose]
        elif pose != "standing":
            raise ValueError("unknown pose")
        d.ctrl[self.actuators] = d.qpos[self.qadr]
        # Every pose starts touching the floor, standing with no gap and
        # counterfactuals 1 mm above it, so the first step is not a drop.
        for _ in range(500):
            mujoco.mj_forward(self.model, d)
            distances = [c.dist for c in d.contact if self.floor in c.geom]
            if distances:
                d.qpos[2] += -min(distances) + (0.0 if pose == "standing" else 0.001)
                break
            d.qpos[2] -= 0.005
        else:
            raise ValueError(f"could not place the {pose} pose on the ground")
        mujoco.mj_forward(self.model, d)
        if any(c.dist < -0.005 for c in d.contact):
            raise ValueError(f"invalid {pose} pose: substantial initial penetration")

    def snapshot(self, adapter, command):
        state = np.empty(mujoco.mj_stateSize(self.model, self.state_spec))
        mujoco.mj_getState(self.model, self.data, state, self.state_spec)
        return {
            "physics": state,
            "policy": adapter.snapshot(),
            "command": np.array(command),
            "rng": copy.deepcopy(self.rng.bit_generator.state),
            "command_history": copy.deepcopy(self.command_history),
            "thrash_state": self.thrash_detector.save_state(),
        }

    def restore(self, snapshot, adapter):
        mujoco.mj_setState(self.model, self.data, snapshot["physics"], self.state_spec)
        adapter.restore(snapshot["policy"])
        self.rng.bit_generator.state = copy.deepcopy(snapshot["rng"])
        self.command_history = copy.deepcopy(snapshot["command_history"])
        self.thrash_detector.restore_state(snapshot["thrash_state"])
        mujoco.mj_forward(self.model, self.data)
        return snapshot["command"].copy()

    def apply_push(self, push, elapsed):
        self.data.xfrc_applied[:] = 0
        if push and push["start"] <= elapsed < push["start"] + push["duration"] - 1e-12:
            body_id = self.model.body(push["body"]).id
            force = np.asarray(push["force"], dtype=float)
            self.data.xfrc_applied[body_id, :3] = force
            if push.get("point") is not None:
                rotation = self.data.xmat[body_id].reshape(3, 3)
                point = self.data.xpos[body_id] + rotation @ np.asarray(
                    push["point"], dtype=float
                )
                self.data.xfrc_applied[body_id, 3:] = np.cross(
                    point - self.data.xipos[body_id], force
                )

    def observation(self, adapter, command):
        d = self.data
        return adapter.observation(
            d.qpos[self.qadr], d.qvel[self.vadr], d.qpos[3:7], d.qvel[:6], command
        )


def mesh_point_in_body(model, body_name, mesh_name):
    """Return the mesh bounding-box center in the named body's frame."""
    mesh_id = model.mesh(mesh_name).id
    address = model.mesh_vertadr[mesh_id]
    vertices = model.mesh_vert[address : address + model.mesh_vertnum[mesh_id]]
    center = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
    body_id = model.body(body_name).id
    matches = [
        geom_id
        for geom_id in range(model.ngeom)
        if model.geom_bodyid[geom_id] == body_id
        and model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH
        and model.geom_dataid[geom_id] == mesh_id
    ]
    if not matches:
        raise ValueError(f"mesh {mesh_name} is not attached to {body_name}")
    positions = model.geom_pos[matches]
    if not np.allclose(positions, positions[0]):
        raise ValueError(f"mesh {mesh_name} has conflicting placements on {body_name}")
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.geom_quat[matches[0]])
    return positions[0] + rotation.reshape(3, 3) @ center
