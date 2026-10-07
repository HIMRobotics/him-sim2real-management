"""NVIDIA Decoupled WBC command-file wrapper."""

from pathlib import Path

import numpy as np
import yaml
from command import load_schedule
from policy_safety.adapter import NvidiaWalker

NUM_JOINTS = 29


def load_yaml(path):
    document = yaml.safe_load(Path(path).read_text())
    if not isinstance(document, dict):
        raise TypeError(f"{path} must be a mapping")
    return document


def load_command(path):
    """vx, vy, yaw_rate."""
    return load_schedule(path).at(0.0)


class WalkerPolicy:
    """Command-file wrapper for the shared G1 walker adapter."""

    def __init__(self, model_path, spec_path, command_path):
        spec = load_yaml(spec_path)
        joints = spec["joints"]
        if len(joints) != NUM_JOINTS:
            raise ValueError(f"{spec_path} must list {NUM_JOINTS} joints")
        self.walker = NvidiaWalker(
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
        self.schedule = load_schedule(command_path)
        self.command = self.schedule.at(0.0)
        self.elapsed = 0.0

    def targets(self, q, dq, quat, base_velocity):
        self.command = self.schedule.at(self.elapsed)
        self.elapsed += 0.02
        observation = self.walker.observation(q, dq, quat, base_velocity, self.command)
        return self.walker.decode(self.walker.infer(observation))
