"""G1 walker. The command file supplies vx, vy, and yaw_rate.

This is not the GR00T balance network. That file has no velocity command.
"""

from pathlib import Path

import numpy as np
import yaml
from policy_safety.adapter import G1Walker

NUM_JOINTS = 29


def load_yaml(path):
    document = yaml.safe_load(Path(path).read_text())
    if not isinstance(document, dict):
        raise TypeError(f"{path} must be a mapping")
    return document


def load_command(path):
    """vx, vy, yaw_rate."""
    command = np.asarray(load_yaml(path).get("command"), dtype=np.float32)
    if command.shape != (3,) or not np.isfinite(command).all():
        raise ValueError(f"{path} command must be vx, vy, yaw_rate")
    return command


class WalkerPolicy:
    """Command-file wrapper for the shared G1 walker adapter."""

    def __init__(self, model_path, spec_path, command_path):
        spec = load_yaml(spec_path)
        joints = spec["joints"]
        if len(joints) != NUM_JOINTS:
            raise ValueError(f"{spec_path} must list {NUM_JOINTS} joints")
        self.walker = G1Walker(
            model_path,
            {
                "contract": spec["contract"],
                "normalization": spec["normalization"],
                "default_joint_positions": [j["default_position"] for j in joints],
                "action_scales": [j["action_scale"] for j in joints],
            },
        )
        self.kp = np.array([j["kp"] for j in joints], dtype=np.float64)
        self.kd = np.array([j["kd"] for j in joints], dtype=np.float64)
        self.command = load_command(command_path)

    def targets(self, q, dq, quat, base_velocity):
        observation = self.walker.observation(q, dq, quat, base_velocity, self.command)
        return self.walker.decode(self.walker.infer(observation))
