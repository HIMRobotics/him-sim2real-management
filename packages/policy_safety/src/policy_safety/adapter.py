"""Trusted adapters define policy semantics independently of file dimensions."""

import numpy as np

from .config import vector


class PolicyViolation(ValueError):
    """Invalid numerical policy output."""

    def __init__(self, reason, details=None):
        super().__init__(reason)
        self.details = details or {}


def inverse_rotate(quat, vec):
    w, x, y, z = np.asarray(quat, dtype=np.float64)
    xyz = -np.array([x, y, z])
    return np.asarray(vec) + 2 * np.cross(xyz, np.cross(xyz, vec) + w * np.asarray(vec))


class G1Walker:
    """Sample walker neural path, without scripted commands or joint overrides."""

    contract = "g1-walker-99-v1"
    tested_behavior = (
        "Unitree G1 29-joint neural locomotion policy with explicit velocity commands, "
        "without the sample submission's scripted phases, hops or arm overrides"
    )

    def __init__(self, model_path, robot, session=None):
        if robot["contract"] != self.contract or robot["normalization"] != "embedded":
            raise ValueError(
                "adapter requires the explicit G1 walker contract and embedded normalization"
            )
        self.default = vector(
            robot["default_joint_positions"], 29, "default positions"
        ).astype(np.float32)
        self.scales = vector(robot["action_scales"], 29, "action scales").astype(
            np.float32
        )
        if np.any(self.scales <= 0):
            raise ValueError("action scales must be positive")
        if session is None:
            import onnxruntime as ort

            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            options.inter_op_num_threads = 1
            session = ort.InferenceSession(
                str(model_path),
                sess_options=options,
                providers=["CPUExecutionProvider"],
            )
        self.session = session
        inputs, outputs = session.get_inputs(), session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("walker must have one input and one output")
        for tensor, size in ((inputs[0], 99), (outputs[0], 29)):
            if (
                tensor.type != "tensor(float)"
                or len(tensor.shape) != 2
                or tensor.shape[1] != size
                or not (
                    tensor.shape[0] in (1, None) or isinstance(tensor.shape[0], str)
                )
            ):
                raise ValueError(
                    f"unexpected walker tensor: {tensor.name} {tensor.shape} {tensor.type}"
                )
        self.input_name, self.output_name = inputs[0].name, outputs[0].name
        self.reset()

    def reset(self):
        self.previous_action = np.zeros(29, dtype=np.float32)
        self.last_raw = np.empty(0, dtype=np.float32)

    def observation(self, q, dq, quat, base_velocity, command):
        # MuJoCo free-joint angular velocity is already in the local frame.
        obs = np.concatenate(
            [
                inverse_rotate(quat, base_velocity[:3]),
                base_velocity[3:6],
                inverse_rotate(quat, [0, 0, -1]),
                np.asarray(q, dtype=np.float32) - self.default,
                np.asarray(dq, dtype=np.float32),
                self.previous_action,
                command,
            ]
        ).astype(np.float32)
        if obs.shape != (99,) or not np.isfinite(obs).all():
            raise PolicyViolation("nonfinite_or_invalid_observation")
        return obs[None, :]

    def infer(self, observation):
        output = np.asarray(
            self.session.run([self.output_name], {self.input_name: observation})[0]
        )
        self.last_raw = output.copy()
        if output.shape != (1, 29) or not np.isfinite(output).all():
            raise PolicyViolation("nonfinite_or_invalid_action")
        self.previous_action = output[0].astype(np.float32).copy()
        return self.previous_action.copy()

    def decode(self, raw):
        target = self.default + raw * self.scales
        if not np.isfinite(target).all():
            raise PolicyViolation("nonfinite_target")
        return target.astype(np.float64)

    def snapshot(self):
        return self.previous_action.copy()

    def command_velocity_and_torque(self):
        """Position actuators use zero desired velocity and feedforward torque."""
        return np.zeros(29), np.zeros(29)

    def restore(self, state):
        self.previous_action = (
            vector(state, 29, "previous action").astype(np.float32).copy()
        )


ADAPTERS = {G1Walker.contract: G1Walker}
