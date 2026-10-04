"""G1 walker. The command file supplies vx, vy, and yaw_rate.

This is not the GR00T balance network. That file has no velocity command.
"""

from pathlib import Path

import numpy as np
import yaml

NUM_JOINTS = 29


def load_yaml(path):
    document = yaml.safe_load(Path(path).read_text())
    if not isinstance(document, dict):
        raise ValueError(f"{path} must be a mapping")
    return document


def load_spec(path):
    """Defaults, action scales, and gains in Unitree joint order."""
    joints = load_yaml(path)["joints"]
    if len(joints) != NUM_JOINTS:
        raise ValueError(f"{path} must list {NUM_JOINTS} joints")
    return (
        np.array([joint["default_position"] for joint in joints], dtype=np.float32),
        np.array([joint["action_scale"] for joint in joints], dtype=np.float32),
        np.array([joint["kp"] for joint in joints], dtype=np.float64),
        np.array([joint["kd"] for joint in joints], dtype=np.float64),
    )


def load_command(path):
    """vx, vy, yaw_rate."""
    command = np.asarray(load_yaml(path).get("command"), dtype=np.float32)
    if command.shape != (3,) or not np.isfinite(command).all():
        raise ValueError(f"{path} command must be vx, vy, yaw_rate")
    return command


def inverse_rotate(quat, vec):
    """Rotate a world vector into the body frame. Quaternion is w, x, y, z."""
    w, x, y, z = np.asarray(quat, dtype=np.float64)
    xyz = -np.array([x, y, z])
    vec = np.asarray(vec, dtype=np.float64)
    return vec + 2 * np.cross(xyz, np.cross(xyz, vec) + w * vec)


class WalkerPolicy:
    """99-input walker. Output is 29 joint position targets."""

    def __init__(self, model_path, spec_path, command_path):
        import onnxruntime as ort

        self.default, self.scales, self.kp, self.kd = load_spec(spec_path)
        self.command = load_command(command_path)
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            str(model_path),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.previous = np.zeros(NUM_JOINTS, dtype=np.float32)

    def targets(self, q, dq, quat, base_velocity):
        q = np.asarray(q, dtype=np.float32)
        dq = np.asarray(dq, dtype=np.float32)
        base_velocity = np.asarray(base_velocity, dtype=np.float64)
        observation = np.concatenate(
            [
                inverse_rotate(quat, base_velocity[:3]),
                base_velocity[3:6],
                inverse_rotate(quat, [0, 0, -1]),
                q - self.default,
                dq,
                self.previous,
                self.command,
            ]
        ).astype(np.float32)
        if observation.shape != (99,) or not np.isfinite(observation).all():
            raise ValueError("walker observation is not 99 finite values")
        raw = np.asarray(
            self.session.run(
                [self.output_name], {self.input_name: observation[None, :]}
            )[0]
        )
        if raw.shape != (1, NUM_JOINTS) or not np.isfinite(raw).all():
            raise ValueError("walker action is not 29 finite values")
        self.previous = raw[0].astype(np.float32).copy()
        return (self.default + self.previous * self.scales).astype(np.float64)
