"""GEAR-SONIC standing hold for the submit.sh startup gate.

This is the default SONIC release (encoder plus decoder), not SONIC v1.1.
The observation layout matches ``observation_config.yaml`` in
``nvidia/GEAR-SONIC``: g1 encoder mode, with a frozen standing reference
aligned to the heading captured on reset. Joint history and actions use
IsaacLab order. Motor targets use Unitree order.

Startup matches the deploy init: three seconds from the released pose to the
standing angles, then the encoder and decoder turn on. ``--startup groot``
keeps the previous GR00T Balance startup.
"""

import math
import os
from collections import deque
from pathlib import Path

import numpy as np

_W = 20.0 * math.pi
_ARMATURE = {
    "5020": 0.003609725,
    "7520_14": 0.010177520,
    "7520_22": 0.025101925,
    "4010": 0.00425,
}
_EFFORT = {"5020": 25.0, "7520_14": 88.0, "7520_22": 139.0, "4010": 5.0}
_STIFFNESS = {name: armature * _W * _W for name, armature in _ARMATURE.items()}
_DAMPING = {name: 4.0 * armature * _W for name, armature in _ARMATURE.items()}

# Hardware / MuJoCo order. Matches gear_sonic_deploy policy_parameters.hpp.
_MOTOR = (
    "7520_22", "7520_22", "7520_14", "7520_22", "5020", "5020",
    "7520_22", "7520_22", "7520_14", "7520_22", "5020", "5020",
    "7520_14", "5020", "5020",
    "5020", "5020", "5020", "5020", "5020", "4010", "4010",
    "5020", "5020", "5020", "5020", "5020", "4010", "4010",
)
_ANKLE = {4, 5, 10, 11}
_WAIST_ROLL_PITCH = {13, 14}

DEFAULT_ANGLES = np.array(
    [
        -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
        -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
        0.0, 0.0, 0.0,
        0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
        0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
    ],
    dtype=np.float64,
)
ACTION_SCALE = np.array(
    [0.25 * _EFFORT[motor] / _STIFFNESS[motor] for motor in _MOTOR],
    dtype=np.float64,
)
KP = np.array(
    [
        (2.0 if index in _ANKLE or index in _WAIST_ROLL_PITCH else 1.0) * _STIFFNESS[motor]
        for index, motor in enumerate(_MOTOR)
    ],
    dtype=np.float64,
)
KD = np.array(
    [
        (2.0 if index in _ANKLE or index in _WAIST_ROLL_PITCH else 1.0) * _DAMPING[motor]
        for index, motor in enumerate(_MOTOR)
    ],
    dtype=np.float64,
)

# Values are IsaacLab indexes. Index is the MuJoCo / Unitree motor index.
ISAACLAB_TO_MUJOCO = np.array(
    [0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8,
     11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28],
    dtype=int,
)
MUJOCO_TO_ISAACLAB = np.array(
    [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
     16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28],
    dtype=int,
)

# Default GEAR-SONIC observation_config.yaml. Unused slots stay zero.
ENCODER_FIELDS = (
    ("encoder_mode_4", 4),
    ("motion_joint_positions_10frame_step5", 290),
    ("motion_joint_velocities_10frame_step5", 290),
    ("motion_root_z_position_10frame_step5", 10),
    ("motion_root_z_position", 1),
    ("motion_anchor_orientation", 6),
    ("motion_anchor_orientation_10frame_step5", 60),
    ("motion_joint_positions_lowerbody_10frame_step5", 120),
    ("motion_joint_velocities_lowerbody_10frame_step5", 120),
    ("vr_3point_local_target", 9),
    ("vr_3point_local_orn_target", 12),
    ("smpl_joints_10frame_step1", 720),
    ("smpl_anchor_orientation_10frame_step1", 60),
    ("motion_joint_positions_wrists_10frame_step1", 60),
)
POLICY_FIELDS = (
    ("token_state", 64),
    ("his_base_angular_velocity_10frame_step1", 30),
    ("his_body_joint_positions_10frame_step1", 290),
    ("his_body_joint_velocities_10frame_step1", 290),
    ("his_last_actions_10frame_step1", 290),
    ("his_gravity_dir_10frame_step1", 30),
)
ENCODER_DIM = sum(size for _, size in ENCODER_FIELDS)
POLICY_DIM = sum(size for _, size in POLICY_FIELDS)
HISTORY = 10


def _offsets(fields):
    cursor = 0
    found = {}
    for name, size in fields:
        found[name] = (cursor, size)
        cursor += size
    return found


ENCODER_OFFSETS = _offsets(ENCODER_FIELDS)
POLICY_OFFSETS = _offsets(POLICY_FIELDS)


def resolve_sonic_dir():
    """Directory containing model_encoder.onnx and model_decoder.onnx."""
    override = os.environ.get("HIM_SONIC_DIR")
    if override:
        return Path(override)
    bundled = Path(__file__).resolve().parents[1] / ".cache" / "sonic"
    if (bundled / "model_encoder.onnx").is_file():
        return bundled
    return Path("/home/unitree/him-sim2real-management/.cache/sonic")


def require_sonic_weights(directory=None):
    directory = Path(directory) if directory is not None else resolve_sonic_dir()
    missing = [
        name for name in ("model_encoder.onnx", "model_decoder.onnx")
        if not (directory / name).is_file()
    ]
    if missing:
        raise FileNotFoundError(
            "SONIC startup weights missing in {0}: {1}. "
            "Download model_encoder.onnx and model_decoder.onnx from "
            "https://huggingface.co/nvidia/GEAR-SONIC into that directory, "
            "or start with --startup groot.".format(directory, ", ".join(missing))
        )
    return directory


def _quat_mul(left, right):
    w1, x1, y1, z1 = (float(value) for value in left)
    w2, x2, y2, z2 = (float(value) for value in right)
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def _quat_conj(quat):
    w, x, y, z = (float(value) for value in quat)
    return np.array([w, -x, -y, -z], dtype=np.float64)


def heading_quat(quat):
    """Yaw-only quaternion, wxyz. Roll and pitch are removed."""
    w, x, y, z = (float(value) for value in quat)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)], dtype=np.float64)


def _rotation_matrix(quat):
    w, x, y, z = (float(value) for value in quat)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def anchor_orientation(base_quat, reference_quat):
    """First two columns of the base-to-reference rotation, row-major."""
    relative = _quat_mul(_quat_conj(base_quat), reference_quat)
    matrix = _rotation_matrix(relative)
    return matrix[:, :2].reshape(-1)


def gravity_dir(quat):
    """Gravity in the body frame. Quaternion is wxyz."""
    inverse = _quat_conj(quat)
    matrix = _rotation_matrix(inverse)
    return matrix @ np.array([0.0, 0.0, -1.0])


def isaaclab_from_motor(values):
    motor = np.asarray(values, dtype=np.float64)
    isaac = np.zeros(29, dtype=np.float64)
    isaac[:] = motor[MUJOCO_TO_ISAACLAB]
    return isaac


def motor_targets(raw_action_isaac):
    """Map a 29-wide IsaacLab action to Unitree position targets."""
    action = np.asarray(raw_action_isaac, dtype=np.float64).reshape(29)
    targets = np.array(DEFAULT_ANGLES, copy=True)
    for motor in range(29):
        targets[motor] += action[ISAACLAB_TO_MUJOCO[motor]] * ACTION_SCALE[motor]
    return targets


def _fill_history(buffer):
    """Oldest frame first. A short history repeats the oldest sample."""
    frames = list(buffer)
    if not frames:
        raise ValueError("SONIC history is empty")
    if len(frames) < HISTORY:
        frames = [frames[0]] * (HISTORY - len(frames)) + frames
    return np.concatenate(frames)


class SonicStandingPolicy:
    """Hold SONIC's standing pose. A zero action is that pose."""

    entry_ramp_seconds = 3.0

    def __init__(self, directory=None, encoder=None, decoder=None):
        self.directory = require_sonic_weights(directory) if encoder is None else Path(directory or ".")
        self.encoder = encoder
        self.decoder = decoder
        if self.encoder is None or self.decoder is None:
            import onnxruntime as ort

            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            options.inter_op_num_threads = 1
            if self.encoder is None:
                self.encoder = ort.InferenceSession(
                    str(self.directory / "model_encoder.onnx"),
                    sess_options=options,
                    providers=["CPUExecutionProvider"],
                )
            if self.decoder is None:
                self.decoder = ort.InferenceSession(
                    str(self.directory / "model_decoder.onnx"),
                    sess_options=options,
                    providers=["CPUExecutionProvider"],
                )
        self._check_session(self.encoder, ENCODER_DIM, 64, "encoder")
        self._check_session(self.decoder, POLICY_DIM, 29, "decoder")
        self.kp = KP.copy()
        self.kd = KD.copy()
        self.reset()

    @staticmethod
    def _check_session(session, in_size, out_size, label):
        inputs = session.get_inputs()
        outputs = session.get_outputs()
        in_last = None if not inputs else inputs[0].shape[-1]
        if len(inputs) != 1 or in_last is None or int(in_last) != in_size:
            raise ValueError(
                "SONIC {0} input is {1}, expected one tensor of length {2}. "
                "This adapter matches the default GEAR-SONIC release, not v1.1.".format(
                    label, [item.shape for item in inputs], in_size
                )
            )
        out_last = None if not outputs else outputs[0].shape[-1]
        if len(outputs) != 1 or out_last is None or int(out_last) != out_size:
            raise ValueError(
                "SONIC {0} output is {1}, expected length {2}".format(
                    label, [item.shape for item in outputs], out_size
                )
            )

    def entry_target(self):
        """Unitree-order pose reached before the network is enabled."""
        return DEFAULT_ANGLES.copy()

    def reset(self):
        self.heading = None
        self.action = np.zeros(29, dtype=np.float64)
        self.joint_position = deque(maxlen=HISTORY)
        self.joint_velocity = deque(maxlen=HISTORY)
        self.last_action = deque(maxlen=HISTORY)
        self.gyro = deque(maxlen=HISTORY)
        self.gravity = deque(maxlen=HISTORY)

    def warmup(self):
        """Prime both networks on zeros. History and heading stay untouched."""
        token = self._run(self.encoder, np.zeros(ENCODER_DIM, dtype=np.float32))
        observation = np.zeros(POLICY_DIM, dtype=np.float32)
        observation[:64] = token
        action = self._run(self.decoder, observation)
        if action.shape != (29,) or not np.isfinite(action).all():
            raise ValueError("SONIC warmup returned a non-finite action")

    def update(self, q, dq, quat, gyro):
        """Return 29 Unitree position targets for the standing reference."""
        q = np.asarray(q, dtype=np.float64).reshape(29)
        dq = np.asarray(dq, dtype=np.float64).reshape(29)
        quat = np.asarray(quat, dtype=np.float64).reshape(4)
        gyro = np.asarray(gyro, dtype=np.float64).reshape(3)
        if not all(np.isfinite(value).all() for value in (q, dq, quat, gyro)):
            raise ValueError("nonfinite SONIC observation")
        if self.heading is None:
            self.heading = heading_quat(quat)
        isaac_q = isaaclab_from_motor(q - DEFAULT_ANGLES)
        isaac_dq = isaaclab_from_motor(dq)
        self.joint_position.append(isaac_q)
        self.joint_velocity.append(isaac_dq)
        self.last_action.append(self.action.copy())
        self.gyro.append(gyro)
        self.gravity.append(gravity_dir(quat))

        stand = isaaclab_from_motor(DEFAULT_ANGLES)
        orientation = anchor_orientation(quat, self.heading)
        encoder_obs = np.zeros(ENCODER_DIM, dtype=np.float32)
        self._write(encoder_obs, ENCODER_OFFSETS["encoder_mode_4"], [0, 0, 0, 0])
        self._write(encoder_obs, ENCODER_OFFSETS["motion_joint_positions_10frame_step5"], np.tile(stand, 10))
        self._write(
            encoder_obs,
            ENCODER_OFFSETS["motion_anchor_orientation_10frame_step5"],
            np.tile(orientation, 10),
        )
        token = self._run(self.encoder, encoder_obs)
        policy_obs = np.zeros(POLICY_DIM, dtype=np.float32)
        self._write(policy_obs, POLICY_OFFSETS["token_state"], token)
        self._write(policy_obs, POLICY_OFFSETS["his_base_angular_velocity_10frame_step1"], _fill_history(self.gyro))
        self._write(policy_obs, POLICY_OFFSETS["his_body_joint_positions_10frame_step1"], _fill_history(self.joint_position))
        self._write(policy_obs, POLICY_OFFSETS["his_body_joint_velocities_10frame_step1"], _fill_history(self.joint_velocity))
        self._write(policy_obs, POLICY_OFFSETS["his_last_actions_10frame_step1"], _fill_history(self.last_action))
        self._write(policy_obs, POLICY_OFFSETS["his_gravity_dir_10frame_step1"], _fill_history(self.gravity))
        if not np.isfinite(policy_obs).all():
            raise ValueError("nonfinite SONIC observation")
        self.action = self._run(self.decoder, policy_obs)
        if not np.isfinite(self.action).all():
            raise ValueError("SONIC returned a non-finite action")
        return motor_targets(self.action)

    @staticmethod
    def _write(buffer, slot, values):
        start, size = slot
        array = np.asarray(values, dtype=np.float32).reshape(-1)
        if array.size != size:
            raise ValueError("SONIC field width {0} does not match {1}".format(array.size, size))
        buffer[start:start + size] = array

    @staticmethod
    def _run(session, vector):
        feed = {session.get_inputs()[0].name: np.asarray(vector, dtype=np.float32).reshape(1, -1)}
        raw = np.asarray(session.run(None, feed)[0], dtype=np.float64).reshape(-1)
        return raw
