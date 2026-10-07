"""ONNX examples shared by simulation and hardware containers."""

from pathlib import Path

import numpy as np
import onnxruntime as ort
import yaml
from policy_safety.adapter import G1Walker

from robot_safety.balance_policy import LEG_KD, LEG_KP, BalancePolicy

NAMES = ("stairs", "walker", "balance")


def session(path):
    """Use one inference thread to keep control latency predictable."""
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    return ort.InferenceSession(
        str(path), sess_options=options, providers=["CPUExecutionProvider"]
    )


class ExamplePolicy:
    """Emit only joint commands, never root poses or simulator mutations."""

    def __init__(self, name, project):
        if name not in NAMES:
            raise ValueError("unknown example policy")
        self.name = name
        self.sequence = 0
        self.phase = "walk"
        self.phase_time = 0.0
        self.arm_pose = None
        project = Path(project)
        if name == "balance":
            path = project / "balance.onnx"
            self.policy = BalancePolicy(str(path), session=session(path))
            self.policy.warmup()
            self.kp = np.concatenate((LEG_KP, np.full(14, 40.0)))
            self.kd = np.concatenate((LEG_KD, np.full(14, 1.0)))
        else:
            cfg = yaml.safe_load((project / "policy.yaml").read_text())
            joints = cfg["joints"]
            robot = {
                "contract": cfg["contract"],
                "normalization": cfg["normalization"],
                "default_joint_positions": [j["default_position"] for j in joints],
                "action_scales": [j["action_scale"] for j in joints],
            }
            path = project / cfg["model"]["file"]
            self.policy = G1Walker(path, robot, session=session(path))
            self.kp = np.array([j["kp"] for j in joints])
            self.kd = np.array([j["kd"] for j in joints])
            self.policy.infer(np.zeros((1, 99), dtype=np.float32))
            self.policy.reset()

    def velocity_command(self, state):
        """Command forward walking or the stair submission's scripted phases."""
        if "task_command" in state:
            return np.asarray(state["task_command"], dtype=np.float32)
        if self.name == "walker":
            return np.array([0.6, 0.0, 0.0], dtype=np.float32)
        x, y, z = state["position"]
        if self.phase == "walk":
            if x >= 1.55 and abs(y) < 0.35 and z > 0.60 and state["upright"] > 0.75:
                self.phase, self.phase_time = "stabilize", 0.0
            vx = 0.55
        elif self.phase == "stabilize":
            vx = 0.05
            self.phase_time += 0.02
            if self.phase_time > 0.80:
                self.phase, self.phase_time = "cross", 0.0
        elif self.phase == "cross":
            vx = 0.90
            self.phase_time += 0.02
            if x >= 2.35 or self.phase_time > 8.0:
                self.phase, self.phase_time = "finish_hold", 0.0
        else:
            vx = 0.0
        return np.array(
            [vx, np.clip(-1.2 * y, -0.5, 0.5), np.clip(-1.8 * y, -0.8, 0.8)],
            dtype=np.float32,
        )

    def command(self, state):
        q = np.asarray(state["q"], dtype=float)
        if self.arm_pose is None:
            self.arm_pose = q[15:].copy()
            self.hold_pose = q.copy()
        if self.sequence == 0 or not state.get("control_ready", True):
            target = self.hold_pose.copy()
        elif self.name == "balance":
            lower = self.policy.update(q, state["dq"], state["quat"], state["gyro"])
            target = np.concatenate((lower, self.arm_pose))
        else:
            observation = self.policy.observation(
                q,
                state["dq"],
                state["quat"],
                state["base_velocity"],
                self.velocity_command(state),
            )
            target = self.policy.decode(self.policy.infer(observation))
            if self.name == "stairs":
                if self.phase == "cross" and 1.88 <= state["position"][0] < 2.05:
                    phase = self.phase_time % 0.8
                    if phase < 0.25:
                        target[[0, 6]], target[[3, 9]], target[[4, 10]] = (
                            -0.55,
                            1.05,
                            -0.48,
                        )
                    elif phase < 0.45:
                        target[[0, 6]], target[[3, 9]], target[[4, 10]] = (
                            0.15,
                            0.10,
                            0.05,
                        )
                target[15:] = self.policy.default[15:]
                target = np.clip(target, state["ctrl_low"], state["ctrl_high"])
        frame = {
            "session_id": state["session_id"],
            "sequence": self.sequence,
            "observation_id": state["observation_id"],
            "q": target.tolist(),
            "dq": [0.0] * 29,
            "kp": self.kp.tolist(),
            "kd": self.kd.tolist(),
            "tau": [0.0] * 29,
        }
        self.sequence += 1
        return frame
