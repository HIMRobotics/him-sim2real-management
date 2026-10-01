"""Drive the cached G1 walker around one flat circle.

The balance network is the standing policy. A walking step also exceeds the
submission tracking window, so this gait is applied directly and the fall
check still runs.
"""

import os
from pathlib import Path

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = ROOT / "examples/walker.onnx"


def load_command(path):
    """Read vx, vy, yaw_rate from an example command file."""
    document = yaml.safe_load(Path(path).read_text())
    command = np.asarray(document.get("command"), dtype=np.float32)
    if command.shape != (3,) or not np.isfinite(command).all():
        raise ValueError(f"{path} command must be vx, vy, yaw_rate")
    return command


class CircleWalker:
    def __init__(self, model_path=None):
        model_path = Path(
            model_path or os.environ.get("G1_SIM_WALKER") or DEFAULT_MODEL
        )
        spec = model_path.with_suffix(".yaml")
        if not spec.is_file():
            spec = model_path.parent / "policy.yaml"
        if not model_path.is_file() or not spec.is_file():
            raise FileNotFoundError(
                f"Walker weights not found at {model_path} with {spec.name}"
            )
        document = yaml.safe_load(spec.read_text())
        joints = document["joints"]
        from policy_safety.adapter import G1Walker

        self.walker = G1Walker(
            model_path,
            {
                "contract": "g1-walker-99-v1",
                "normalization": "embedded",
                "default_joint_positions": [joint["default_position"] for joint in joints],
                "action_scales": [joint["action_scale"] for joint in joints],
            },
        )
        self.defaults = [joint["default_position"] for joint in joints]
        self.kp = np.array([joint["kp"] for joint in joints], dtype=float)
        self.kd = np.array([joint["kd"] for joint in joints], dtype=float)
        command_path = os.environ.get("G1_SIM_COMMAND") or (
            ROOT / "examples/circle.yaml"
        )
        self.command = load_command(command_path)
        self.started = False

    def settle(self, plant):
        """Place the walker stance on the floor before the first step."""
        data = plant.data
        for index, angle in enumerate(self.defaults):
            data.qpos[plant.qadr[index]] = angle
        for _ in range(800):
            mujoco.mj_forward(plant.model, data)
            distances = [
                contact.dist
                for contact in data.contact
                if plant.floor in contact.geom
                and any(geom in plant.robot_geoms for geom in contact.geom)
            ]
            if distances:
                data.qpos[plant.base_qadr + 2] += -min(distances) + 0.001
                break
            data.qpos[plant.base_qadr + 2] -= 0.005
        else:
            raise RuntimeError("could not place the walker on the floor")
        mujoco.mj_forward(plant.model, data)
        data.qvel[:] = 0

    def targets(self, plant):
        state = plant.state()
        q = np.asarray(state["q"], dtype=np.float32)
        if not self.started:
            return q.astype(np.float64)
        observation = self.walker.observation(
            q,
            np.asarray(state["dq"], dtype=np.float32),
            state["quat"],
            np.asarray(state["base_velocity"], dtype=np.float32),
            self.command,
        )
        return self.walker.decode(self.walker.infer(observation))
