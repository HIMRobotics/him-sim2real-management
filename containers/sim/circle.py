"""Drive NVIDIA Decoupled WBC from a velocity command file."""

import os
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = ROOT / "containers/examples/policies/walker/walker.onnx"
sys.path.insert(0, str(ROOT / "containers/examples"))
from command import load_schedule


def load_command(path):
    """Read the command in effect when an example file starts."""
    return load_schedule(path).at(0.0)


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
        from policy_safety.adapter import ADAPTERS

        self.walker = ADAPTERS[document["contract"]](
            model_path,
            {
                "contract": document["contract"],
                "normalization": document["normalization"],
                "default_joint_positions": [joint["default_position"] for joint in joints],
                "action_scales": [joint["action_scale"] for joint in joints],
            },
        )
        self.defaults = [joint["default_position"] for joint in joints]
        self.kp = np.array([joint["kp"] for joint in joints], dtype=float)
        self.kd = np.array([joint["kd"] for joint in joints], dtype=float)
        command_path = os.environ.get("G1_SIM_COMMAND") or (
            ROOT / "containers/examples/commands/walker/circle.yaml"
        )
        self.schedule = load_schedule(command_path)
        self.started = False
        self.gait_start = None

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
        now = float(state["sim_time"])
        if self.started and self.gait_start is None:
            self.gait_start = now
        observation = self.walker.observation(
            q,
            np.asarray(state["dq"], dtype=np.float32),
            state["quat"],
            np.asarray(state["base_velocity"], dtype=np.float32),
            self.schedule.at(now - self.gait_start) if self.started else np.zeros(3),
        )
        return self.walker.decode(self.walker.infer(observation))
