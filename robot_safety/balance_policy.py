"""Lower-body balance targets from the published Gear WBC standing network.

The observation layout matches
``decoupled_wbc/sim2mujoco/scripts/run_mujoco_gear_wbc.py`` and
``g1_gear_wbc.yaml``. Joint 0..14 are the Unitree motor order: both legs, then
waist yaw, roll, pitch. The network does not command the arms.
"""

import collections
import os
from pathlib import Path

import numpy as np

from .hardware_constants import LOWER_BODY_INDICES, NUM_JOINTS

_MODEL_RELATIVE = Path(".cache") / "groot_balance" / "GR00T-WholeBodyControl-Balance.onnx"


def resolve_model_path():
    """Find the balance weights.

    The hardware node runs from the colcon install under /tmp, so a path
    next to this file is the install tree, not the repo cache.
    """
    override = os.environ.get("HIM_BALANCE_ONNX")
    if override:
        return Path(override)
    beside_package = Path(__file__).resolve().parents[1] / _MODEL_RELATIVE
    if beside_package.is_file():
        return beside_package
    return Path("/home/unitree/him-sim2real-management") / _MODEL_RELATIVE


MODEL_PATH = resolve_model_path()

NUM_ACTIONS = len(LOWER_BODY_INDICES)
SINGLE_OBS_DIM = 86
OBS_HISTORY_LEN = 6
NUM_OBS = SINGLE_OBS_DIM * OBS_HISTORY_LEN

ANG_VEL_SCALE = 0.5
DOF_POS_SCALE = 1.0
DOF_VEL_SCALE = 0.05
ACTION_SCALE = 0.25
HEIGHT_CMD = np.float32(0.74)

DEFAULT_ANGLES = np.array(
    [
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
        0.0, 0.0, 0.0,
    ],
    dtype=np.float32,
)
LEG_KP = np.array(
    [150, 150, 150, 200, 40, 40, 150, 150, 150, 200, 40, 40, 250, 250, 250],
    dtype=np.float32,
)
LEG_KD = np.array(
    [2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2, 5, 5, 5],
    dtype=np.float32,
)
JOINT_LIMITS = np.array(
    [
        [-2.5307, 2.8798],
        [-0.5236, 2.9671],
        [-2.7576, 2.7576],
        [-0.087267, 2.8798],
        [-0.87267, 0.5236],
        [-0.2618, 0.2618],
        [-2.5307, 2.8798],
        [-2.9671, 0.5236],
        [-2.7576, 2.7576],
        [-0.087267, 2.8798],
        [-0.87267, 0.5236],
        [-0.2618, 0.2618],
        [-2.618, 2.618],
        [-0.52, 0.52],
        [-0.52, 0.52],
    ],
    dtype=np.float32,
)


def gravity_orientation(quat_wxyz):
    """Project world gravity into the body frame. Quaternion is w, x, y, z."""
    w, x, y, z = (float(value) for value in quat_wxyz)
    conjugate = np.array([w, -x, -y, -z], dtype=np.float64)
    gravity = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    return np.array(
        [
            gravity[2] * 2.0 * (conjugate[1] * conjugate[3] + conjugate[0] * conjugate[2]),
            gravity[2] * 2.0 * (conjugate[2] * conjugate[3] - conjugate[0] * conjugate[1]),
            gravity[2]
            * (
                conjugate[0] ** 2
                - conjugate[1] ** 2
                - conjugate[2] ** 2
                + conjugate[3] ** 2
            ),
        ],
        dtype=np.float32,
    )


def single_observation(q, dq, quat_wxyz, gyro, action):
    """Build one 86-value frame. q and dq are all 29 motors."""
    q = np.asarray(q, dtype=np.float32)
    dq = np.asarray(dq, dtype=np.float32)
    action = np.asarray(action, dtype=np.float32)
    if q.shape != (NUM_JOINTS,) or dq.shape != (NUM_JOINTS,):
        raise ValueError("expected 29 joint positions and velocities")
    if action.shape != (NUM_ACTIONS,):
        raise ValueError("expected 15 previous actions")
    defaults = np.zeros(NUM_JOINTS, dtype=np.float32)
    defaults[:NUM_ACTIONS] = DEFAULT_ANGLES
    obs = np.zeros(SINGLE_OBS_DIM, dtype=np.float32)
    obs[3] = HEIGHT_CMD
    obs[7:10] = np.asarray(gyro, dtype=np.float32) * ANG_VEL_SCALE
    obs[10:13] = gravity_orientation(quat_wxyz)
    obs[13:42] = (q - defaults) * DOF_POS_SCALE
    obs[42:71] = dq * DOF_VEL_SCALE
    obs[71:86] = action
    return obs


def clip_lower_targets(targets):
    """Keep the 15 commanded joints inside the Gear WBC joint ranges."""
    targets = np.asarray(targets, dtype=np.float32)
    return np.clip(targets, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])


class BalancePolicy:
    """Run the standing network. A zero twist always selects this file, not walk."""

    def __init__(self, model_path):
        import onnxruntime as ort

        self.session = ort.InferenceSession(
            model_path, providers=["CPUExecutionProvider"]
        )
        self.input_name = self.session.get_inputs()[0].name
        self.action = np.zeros(NUM_ACTIONS, dtype=np.float32)
        self.history = collections.deque(
            [np.zeros(SINGLE_OBS_DIM, dtype=np.float32)] * OBS_HISTORY_LEN,
            maxlen=OBS_HISTORY_LEN,
        )

    def warmup(self):
        """Prime inference without modifying history or sending motor targets."""
        raw = self.session.run(
            None, {self.input_name: np.zeros((1, NUM_OBS), dtype=np.float32)}
        )[0]
        action = np.asarray(raw, dtype=np.float32).reshape(NUM_ACTIONS)
        if not np.all(np.isfinite(action)):
            raise ValueError("balance warmup returned a non-finite action")

    def update(self, q, dq, quat_wxyz, gyro):
        """Append one frame and return clipped lower-body position targets."""
        frame = single_observation(q, dq, quat_wxyz, gyro, self.action)
        self.history.append(frame)
        packed = np.zeros((1, NUM_OBS), dtype=np.float32)
        for index, item in enumerate(self.history):
            packed[0, index * SINGLE_OBS_DIM : (index + 1) * SINGLE_OBS_DIM] = item
        raw = self.session.run(None, {self.input_name: packed})[0]
        self.action = np.asarray(raw, dtype=np.float32).reshape(NUM_ACTIONS)
        if not np.all(np.isfinite(self.action)):
            raise ValueError("balance network returned a non-finite action")
        return clip_lower_targets(self.action * ACTION_SCALE + DEFAULT_ANGLES)
