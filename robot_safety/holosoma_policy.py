"""Holosoma g1-29dof FastSAC locomotion, without the Unitree SDK.

The observation and decode match ``holosoma_inference`` locomotion:
100-float ``actor_obs`` in alphabetical term order, scalar action scale
0.25, and gains from the ONNX metadata. Targets are Unitree joint order.
Submission callers publish on ``/safety/precheck``. The host also uses this
policy for soft-reset recovery. This module never opens a motor channel.
"""

import json
import os
from pathlib import Path

import numpy as np

from .hardware_constants import G1_29_JOINT_NAMES

ACTION_SCALE = 0.25
RL_RATE = 50
GAIT_PERIOD = 1.0
OBS_DIM = 100
# Holosoma ``get_init_target``: one step per control tick, 500 ticks at 50 Hz.
INIT_STEPS = 500
# Training and inference both concatenate actor_obs with sorted(term names).
# Declaration order in the locomotion config is a different 100-vector.
ACTOR_OBS_SLICES = {
    "actions": (0, 29),
    "base_ang_vel": (29, 32),
    "command_ang_vel": (32, 33),
    "command_lin_vel": (33, 35),
    "cos_phase": (35, 37),
    "dof_pos": (37, 66),
    "dof_vel": (66, 95),
    "projected_gravity": (95, 98),
    "sin_phase": (98, 100),
}
# Unitree order. Same crouch Holosoma ships as default_dof_angles.
DEFAULT_ANGLES = np.array([
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    0.0, 0.0, 0.0,
    0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
    0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
], dtype=float)
_PHASE_DT = 2.0 * np.pi / (RL_RATE * GAIT_PERIOD)


def holosoma_weights():
    """FastSAC checkpoint. ``HIM_HOLOSOMA_ONNX`` overrides the clone path."""
    override = os.environ.get("HIM_HOLOSOMA_ONNX")
    if override:
        path = Path(override)
    else:
        root = Path(os.environ.get("HIM_HOLOSOMA_DIR", "/home/unitree/holosoma"))
        path = (
            root / "src/holosoma_inference/holosoma_inference"
            / "models/loco/g1_29dof/fastsac_g1_29dof.onnx"
        )
    if not path.is_file():
        raise ValueError(
            "Holosoma weights missing at {0}. Set HIM_HOLOSOMA_ONNX.".format(path)
        )
    return path


def projected_gravity(quat):
    """Gravity in the base frame. Quaternion is wxyz, matching Holosoma."""
    q = np.asarray(quat, dtype=float).reshape(1, 4)
    v = np.array([[0.0, 0.0, -1.0]])
    q_w = q[:, 0]
    q_vec = q[:, 1:]
    a = v * (2.0 * q_w ** 2 - 1.0)[:, np.newaxis]
    b = np.cross(q_vec, v) * q_w[:, np.newaxis] * 2.0
    c = q_vec * np.sum(q_vec * v, axis=1, keepdims=True) * 2.0
    return (a - b + c).reshape(3)


def _meta_vector(meta, key, size):
    if key not in meta:
        raise ValueError("Holosoma ONNX is missing metadata {0}".format(key))
    value = np.asarray(json.loads(meta[key]), dtype=float).reshape(-1)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError("Holosoma metadata {0} must be {1} finite numbers".format(key, size))
    return value


def _unitree_order(names):
    stems = [str(name).replace("_joint", "") for name in names]
    if stems == list(G1_29_JOINT_NAMES):
        return np.arange(29)
    missing = [name for name in G1_29_JOINT_NAMES if name not in stems]
    if missing or len(stems) != 29:
        raise ValueError("Holosoma dof_names are not the 29 G1 joints")
    return np.array([stems.index(name) for name in G1_29_JOINT_NAMES])


class HolosomaLocomotionPolicy:
    """One 50 Hz locomotion step. Velocity is body-frame vx, vy, yaw rate.

    Startup matches Holosoma's Y then A. Y spends ``INIT_STEPS`` control
    ticks moving from the measured pose to ``default_angles`` with the
    network off. A then runs standing inference immediately. There is no
    second blend back toward the released pose.
    """

    entry_ramp_seconds = INIT_STEPS / RL_RATE
    entry_blend_seconds = 0.0

    def __init__(self, path, session=None):
        self.path = str(path)
        self.session = session if session is not None else _session(self.path)
        inputs = self.session.get_inputs()
        if len(inputs) != 1 or inputs[0].name != "actor_obs":
            raise ValueError("Holosoma model must take a single actor_obs input")
        meta = self.session.get_modelmeta().custom_metadata_map
        # model index of each Unitree joint. Identity when the file is already SDK order.
        self._order = _unitree_order(json.loads(meta["dof_names"]))
        self.kp = _meta_vector(meta, "kp", 29)[self._order]
        self.kd = _meta_vector(meta, "kd", 29)[self._order]
        self.default_angles = DEFAULT_ANGLES.copy()
        self.last_action = np.zeros(29)
        self.phase = np.array([0.0, np.pi])
        self.standing = False
        self.lin = np.zeros(2)
        self.ang = 0.0

    def entry_target(self):
        """Crouch reached before the network is enabled. Unitree joint order."""
        return self.default_angles.copy()

    def reset(self):
        self.last_action[:] = 0.0
        self.phase[:] = (0.0, np.pi)
        self.standing = False
        self.lin[:] = 0.0
        self.ang = 0.0

    def warmup(self):
        self.update(self.default_angles, np.zeros(29), [1.0, 0.0, 0.0, 0.0], np.zeros(3))
        self.reset()

    def update(self, q, dq, quat, gyro, velocity=None):
        if velocity is None:
            velocity = np.zeros(3)
        velocity = np.asarray(velocity, dtype=float).reshape(3)
        self.lin[:] = velocity[:2]
        self.ang = float(velocity[2])
        self._advance_phase()
        obs = self.observation(q, dq, quat, gyro)
        raw = np.clip(np.asarray(
            self.session.run(None, {"actor_obs": obs.reshape(1, OBS_DIM)})[0],
            dtype=float,
        ).reshape(29), -100.0, 100.0)
        if not np.isfinite(raw).all():
            raise ValueError("holosoma_nonfinite_action")
        self.last_action[:] = raw
        target = raw[self._order] * ACTION_SCALE + self.default_angles
        if not np.isfinite(target).all():
            raise ValueError("holosoma_nonfinite_target")
        return target

    def observation(self, q, dq, quat, gyro):
        q = self._model(np.asarray(q, dtype=float).reshape(29))
        dq = self._model(np.asarray(dq, dtype=float).reshape(29))
        gyro = np.asarray(gyro, dtype=float).reshape(3)
        obs = np.zeros(OBS_DIM, dtype=np.float32)
        obs[0:29] = self.last_action
        obs[29:32] = gyro * 0.25
        obs[32] = self.ang
        obs[33:35] = self.lin
        obs[35:37] = np.cos(self.phase)
        obs[37:66] = q - self._model(self.default_angles)
        obs[66:95] = dq * 0.05
        obs[95:98] = projected_gravity(quat)
        obs[98:100] = np.sin(self.phase)
        if obs.shape != (OBS_DIM,) or not np.isfinite(obs).all():
            raise ValueError("holosoma_bad_observation")
        return obs

    def _advance_phase(self):
        phase = np.fmod(self.phase + _PHASE_DT + np.pi, 2.0 * np.pi) - np.pi
        if np.linalg.norm(self.lin) < 0.01 and abs(self.ang) < 0.01:
            self.phase[:] = np.pi
            self.standing = True
        elif self.standing:
            self.phase[:] = (0.0, np.pi)
            self.standing = False
        else:
            self.phase[:] = phase

    def _model(self, unitree):
        model = np.empty(29, dtype=float)
        model[self._order] = unitree
        return model


def _session(path):
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    return ort.InferenceSession(
        str(path), sess_options=options, providers=["CPUExecutionProvider"]
    )
