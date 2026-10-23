"""BeyondMimic G1 motion tracker without state estimation (st_jzfwe_groot_robust).

node.py calls Policy(assets) once and Policy.act(state) at 50 Hz. act() returns
29 joint position targets in Unitree motor order.

The actor is an ELU MLP 154-512-256-128-29 trained in Isaac Lab, so it reads and
writes joints in Isaac Lab order (left, right, waist interleaved). Observations
are permuted into that order and actions back out of it. It needs no base
position or linear velocity: only the pelvis IMU and the joints.

assets/actor.npz and assets/motion.npz are written by export_policy.py from the
ONNX export. motion.npz holds joint_pos, joint_vel [T, 29] in Unitree order and
the reference torso_quat [T, 4] (wxyz). The clip starts on the GR00T balance
stance. Until control_ready the published target is the measured pose. From the
first control_ready step the clip plays from frame 0, as in training, and holds
its last frame. The frame follows the state's clock (sim_time in simulation,
received_at_secs on the robot), so a skipped control tick cannot slow the clip.
"""

from pathlib import Path

import numpy as np

_DATA = np.load(Path(__file__).resolve().parent / "assets" / "actor.npz")
KP = np.asarray(_DATA["kp"], dtype=np.float64)
KD = np.asarray(_DATA["kd"], dtype=np.float64)

UNITREE_JOINTS = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]
# ISAAC_ORDER[i] is the Unitree index of Isaac Lab joint i.
ISAAC_ORDER = np.array([0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
                        16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28])
WAIST_YAW, WAIST_ROLL, WAIST_PITCH = 12, 13, 14
FRAME_SECONDS = 0.02


def _elu(x):
    return np.where(x > 0.0, x, np.expm1(x))


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_inv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _axis_quat(axis, angle):
    q = np.zeros(4)
    q[0] = np.cos(angle / 2.0)
    q[1 + axis] = np.sin(angle / 2.0)
    return q


def yaw_only(q):
    w, x, y, z = q
    return _axis_quat(2, np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def torso_quat(pelvis_quat, q):
    """torso_link orientation in the world from the pelvis IMU and the waist joints."""
    yaw = quat_mul(pelvis_quat, _axis_quat(2, q[WAIST_YAW]))
    return quat_mul(quat_mul(yaw, _axis_quat(0, q[WAIST_ROLL])), _axis_quat(1, q[WAIST_PITCH]))


def _first_two_columns(q):
    w, x, y, z = q
    return np.array([
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
        2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
        2 * (x * z - w * y), 2 * (y * z + w * x),
    ])


class Policy:
    def __init__(self, assets):
        data = np.load(Path(assets) / "actor.npz")
        self.layers = [(data["w%d" % i], data["b%d" % i]) for i in (0, 2, 4, 6)]
        self.default_q = np.asarray(data["default_q"], dtype=np.float64)
        self.action_scale = np.asarray(data["action_scale"], dtype=np.float64)
        clip = np.load(Path(assets) / "motion.npz")
        self.ref_q = np.asarray(clip["joint_pos"], dtype=np.float64)
        self.ref_dq = np.asarray(clip["joint_vel"], dtype=np.float64)
        self.ref_quat = np.asarray(clip["torso_quat"], dtype=np.float64)
        if self.ref_q.ndim != 2 or self.ref_q.shape[1] != 29:
            raise ValueError("motion.npz joint_pos must have shape [T, 29]")
        if not (self.ref_dq.shape == self.ref_q.shape and len(self.ref_quat) == len(self.ref_q)):
            raise ValueError("motion.npz arrays must share the frame count")
        if self.layers[0][0].shape[1] != 154:
            raise ValueError("actor.npz must take the 154-value observation")
        self.frame = 0
        self.start_time = None
        self.last_action = np.zeros(29)
        self.heading = None

    @staticmethod
    def _clock(state):
        for key in ("sim_time", "received_at_secs"):
            if state.get(key) is not None:
                return float(state[key])
        return None

    def act(self, state):
        q = np.asarray(state["q"], dtype=np.float64)
        dq = np.asarray(state["dq"], dtype=np.float64)
        if not state.get("control_ready", False):
            return q.copy()

        torso = torso_quat(np.asarray(state["quat"], dtype=np.float64), q)
        now = self._clock(state)
        if self.heading is None:
            # Turn the clip about vertical so its heading matches the robot's.
            self.heading = yaw_only(quat_mul(torso, quat_inv(self.ref_quat[0])))
            self.start_time = now
        if now is not None and self.start_time is not None:
            self.frame = max(self.frame, int(round((now - self.start_time) / FRAME_SECONDS)))
        k = min(self.frame, len(self.ref_q) - 1)
        ref_quat = quat_mul(self.heading, self.ref_quat[k])

        obs = np.concatenate([
            self.ref_q[k][ISAAC_ORDER],
            self.ref_dq[k][ISAAC_ORDER],
            _first_two_columns(quat_mul(quat_inv(torso), ref_quat)),
            np.asarray(state["gyro"], dtype=np.float64),
            (q - self.default_q)[ISAAC_ORDER],
            dq[ISAAC_ORDER],
            self.last_action,
        ]).astype(np.float32)
        hidden = obs
        for weight, bias in self.layers[:-1]:
            hidden = _elu(weight @ hidden + bias)
        weight, bias = self.layers[-1]
        action = (weight @ hidden + bias).astype(np.float64)
        self.last_action = action
        if now is None:
            self.frame += 1
        target = self.default_q.copy()
        target[ISAAC_ORDER] += action * self.action_scale[ISAAC_ORDER]
        return target
