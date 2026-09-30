"""MuJoCo G1 used as the plant behind a container submission."""

import math
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from hardware_constants import G1_29_JOINT_NAMES


def load_model(scene=None):
    """Load a submitted MJCF scene or the bundled flat-floor G1."""
    if scene is not None:
        scene = Path(scene).expanduser().resolve()
        if not scene.is_file():
            raise ValueError(f"MuJoCo scene does not exist: {scene}")
        return mujoco.MjModel.from_xml_path(str(scene))
    robot_dir = (
        Path(__file__).resolve().parents[2]
        / "packages/policy_safety/src/policy_safety/robots/g1"
    )
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".xml", prefix=".container_sim_", dir=robot_dir
    ) as scene:
        scene.write(
            """<mujoco model="g1_container_sim">
  <include file="g1_robot.xml"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="safety_floor" type="plane" size="0 0 0.05" friction="1.15"/>
  </worldbody>
</mujoco>
"""
        )
        scene.flush()
        return mujoco.MjModel.from_xml_path(scene.name)


CONTROL_DT = 0.02
PHYSICS_DT = 0.002
DAMPING_KD = 2.0
FALL_HEIGHT_M = 0.28
FALL_UPRIGHT_MIN = 0.3
# Same starts as the offline screening counterfactuals. Only standing uses the fall check.
POSES = ("standing", "seated", "supine", "prone", "left_side", "right_side")


def quat_to_rpy(quat):
    """Roll, pitch, yaw from a MuJoCo wxyz quaternion."""
    w, x, y, z = (float(v) for v in quat)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


class Plant:
    """29-DOF G1. Commands are PD targets in Unitree joint order."""

    def __init__(self, pose="standing", scene=None, check_fall=True):
        self.scene = Path(scene).expanduser().resolve() if scene else None
        self.model = load_model(self.scene)
        self.model.opt.timestep = PHYSICS_DT
        self.data = mujoco.MjData(self.model)
        self.joint_names = [f"{name}_joint" for name in G1_29_JOINT_NAMES]
        actuators = [self.model.actuator(name).id for name in self.joint_names]
        self.actuators = np.array(actuators)
        joints = [self.model.joint(name).id for name in self.joint_names]
        self.joints = np.array(joints)
        if (
            self.model.nu != 29
            or np.any(self.model.jnt_type[self.joints] != mujoco.mjtJoint.mjJNT_HINGE)
            or np.any(
                self.model.actuator_trntype[self.actuators] != mujoco.mjtTrn.mjTRN_JOINT
            )
            or np.any(self.model.actuator_trnid[self.actuators, 0] != self.joints)
            or np.any(
                self.model.actuator_biastype[self.actuators]
                != mujoco.mjtBias.mjBIAS_AFFINE
            )
            or np.any(
                self.model.actuator_gaintype[self.actuators]
                != mujoco.mjtGain.mjGAIN_FIXED
            )
        ):
            raise ValueError("expected a 29-actuator G1 with affine position actuators")
        self.qadr = self.model.jnt_qposadr[self.joints]
        self.vadr = self.model.jnt_dofadr[self.joints]
        self.default_kp = self.model.actuator_gainprm[self.actuators, 0].copy()
        self.default_kd = -self.model.actuator_biasprm[self.actuators, 2].copy()
        self.base = self.model.body("pelvis").id
        robot_bodies = {self.base}
        for body_id in range(self.base + 1, self.model.nbody):
            if int(self.model.body_parentid[body_id]) in robot_bodies:
                robot_bodies.add(body_id)
        self.robot_geoms = {
            geom_id
            for geom_id in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom_id]) in robot_bodies
        }
        base_joint = self.model.joint("floating_base_joint").id
        if (
            self.model.jnt_type[base_joint] != mujoco.mjtJoint.mjJNT_FREE
            or self.model.jnt_bodyid[base_joint] != self.base
        ):
            raise ValueError("expected floating_base_joint on the G1 pelvis")
        self.base_qadr = int(self.model.jnt_qposadr[base_joint])
        self.base_vadr = int(self.model.jnt_dofadr[base_joint])
        self.floor = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "safety_floor"
        )
        gyro = self.model.sensor("imu-pelvis-angular-velocity")
        accel = self.model.sensor("imu-pelvis-linear-acceleration")
        self.gyro_adr = int(gyro.adr[0] if hasattr(gyro.adr, "__len__") else gyro.adr)
        self.accel_adr = int(
            accel.adr[0] if hasattr(accel.adr, "__len__") else accel.adr
        )
        self.substeps = round(CONTROL_DT / PHYSICS_DT)
        if not math.isclose(self.substeps * PHYSICS_DT, CONTROL_DT):
            raise ValueError("control step is not an integer number of physics steps")
        self.dt = CONTROL_DT
        self.pose = "standing"
        self.fall_enabled = bool(check_fall)
        self.check_fall = self.fall_enabled
        self.reset(pose)

    def reset(self, pose="standing"):
        """Reset to the stand keyframe or a counterfactual on safety_floor."""
        if pose not in POSES:
            raise ValueError("unknown pose")
        self.pose = pose
        if pose != "standing" and self.floor < 0:
            raise ValueError("counterfactual poses require a geom named safety_floor")
        self.check_fall = self.fall_enabled and pose == "standing"
        key = self.model.key("stand").id
        mujoco.mj_resetDataKeyframe(self.model, self.data, key)
        data = self.data
        if pose == "seated":
            for side in ("left", "right"):
                for joint, angle in (
                    ("hip_pitch", -1.45),
                    ("knee", 0.15),
                    ("ankle_pitch", 0.0),
                ):
                    address = self.model.joint(f"{side}_{joint}_joint").qposadr[0]
                    data.qpos[address] = angle
        elif pose != "standing":
            root = math.sqrt(0.5)
            data.qpos[self.base_qadr + 3 : self.base_qadr + 7] = {
                "supine": [root, 0, -root, 0],
                "prone": [root, 0, root, 0],
                "left_side": [root, -root, 0, 0],
                "right_side": [root, root, 0, 0],
            }[pose]
        data.ctrl[self.actuators] = data.qpos[self.qadr]
        if pose != "standing":
            for _ in range(500):
                mujoco.mj_forward(self.model, data)
                distances = [
                    c.dist
                    for c in data.contact
                    if self.floor in c.geom
                    and any(g in self.robot_geoms for g in c.geom)
                ]
                if distances:
                    data.qpos[self.base_qadr + 2] += -min(distances) + 0.001
                    break
                data.qpos[self.base_qadr + 2] -= 0.005
            else:
                raise ValueError("could not place counterfactual on the ground")
        mujoco.mj_forward(self.model, data)
        # The stand keyframe is the sim's normal start. Its feet sit in this
        # floor. Floor poses are placed to the 5 mm penetration tolerance.
        if pose != "standing" and any(
            c.dist < -0.005 and any(g in self.robot_geoms for g in c.geom)
            for c in data.contact
        ):
            raise ValueError(f"invalid {pose} pose: substantial initial penetration")
        if self.check_fall and self.fallen():
            raise ValueError("stand keyframe starts fallen")
        self._apply_arrays(*self.damping_command(self.q(), self.dq()))

    def q(self):
        return self.data.qpos[self.qadr].copy()

    def dq(self):
        return self.data.qvel[self.vadr].copy()

    def damping_command(self, q, dq):
        """Hold the measured angle with damping and no position gain."""
        zeros = np.zeros(29)
        return q.copy(), zeros, zeros, np.full(29, DAMPING_KD), zeros

    def demand(self, q_des, dq_des, kp, kd, tau, q, dq):
        return kp * (q_des - q) + kd * (dq_des - dq) + tau

    def _apply_arrays(self, q_des, dq_des, kp, kd, tau):
        self.model.actuator_gainprm[self.actuators, 0] = kp
        self.model.actuator_biasprm[self.actuators, 0] = tau + kd * dq_des
        self.model.actuator_biasprm[self.actuators, 1] = -kp
        self.model.actuator_biasprm[self.actuators, 2] = -kd
        self.data.ctrl[self.actuators] = q_des

    def apply(self, frame):
        self._apply_arrays(
            np.asarray(frame["q"], dtype=float),
            np.asarray(frame["dq"], dtype=float),
            np.asarray(frame["kp"], dtype=float),
            np.asarray(frame["kd"], dtype=float),
            np.asarray(frame["tau"], dtype=float),
        )

    def step_physics(self):
        """Advance one physics tick. Returns measured joint velocity."""
        mujoco.mj_step(self.model, self.data)
        return self.dq()

    def fallen(self):
        """Upright fall check. Floor counterfactuals do not use it."""
        if not self.check_fall:
            return False
        height = float(self.data.xpos[self.base, 2])
        upright = float(self.data.xmat[self.base].reshape(3, 3)[2, 2])
        return height < FALL_HEIGHT_M or upright < FALL_UPRIGHT_MIN

    def state(self):
        quat = self.data.xquat[self.base]
        sens = self.data.sensordata
        return {
            "q": self.q().tolist(),
            "dq": self.dq().tolist(),
            "quat": [float(v) for v in quat],
            "gyro": [float(v) for v in sens[self.gyro_adr : self.gyro_adr + 3]],
            "accel": [float(v) for v in sens[self.accel_adr : self.accel_adr + 3]],
            "rpy": quat_to_rpy(quat),
            "height": float(self.data.xpos[self.base, 2]),
            "upright": float(self.data.xmat[self.base].reshape(3, 3)[2, 2]),
            "position": self.data.xpos[self.base].tolist(),
            "base_velocity": self.data.qvel[
                self.base_vadr : self.base_vadr + 6
            ].tolist(),
            "ctrl_low": self.model.actuator_ctrlrange[self.actuators, 0].tolist(),
            "ctrl_high": self.model.actuator_ctrlrange[self.actuators, 1].tolist(),
            "sim_time": float(self.data.time),
        }
