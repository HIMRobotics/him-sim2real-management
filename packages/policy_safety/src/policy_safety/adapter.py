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
    """Legacy arena stairs network, without scripted joint overrides."""

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


class NvidiaWalker:
    """NVIDIA Decoupled WBC Walk/Balance with six observation frames."""

    contract = "nvidia-decoupled-wbc-516-v1"
    tested_behavior = "NVIDIA Decoupled WBC locomotion with fixed neutral upper body"

    def __init__(self, model_path, robot, session=None, balance_session=None):
        from pathlib import Path

        import onnxruntime as ort

        if robot["contract"] != self.contract or robot["normalization"] != "explicit":
            raise ValueError("expected NVIDIA Decoupled WBC with explicit normalization")
        self.default = np.zeros(29, dtype=np.float32)
        self.default[:15] = [-0.1, 0, 0, 0.3, -0.2, 0] * 2 + [0, 0, 0]
        self.scales = np.full(29, 0.25, dtype=np.float32)
        if not np.allclose(robot["default_joint_positions"], self.default):
            raise ValueError("NVIDIA default joint positions do not match the contract")
        if not np.allclose(robot["action_scales"], self.scales):
            raise ValueError("NVIDIA action scale must be 0.25")

        def load(path):
            options = ort.SessionOptions()
            options.intra_op_num_threads = 1
            options.inter_op_num_threads = 1
            return ort.InferenceSession(str(path), sess_options=options,
                                       providers=["CPUExecutionProvider"])

        self.session = session if session is not None else load(model_path)
        self.balance_session = balance_session if balance_session is not None else load(
            Path(model_path).with_name("balance.onnx")
        )
        for model in (self.session, self.balance_session):
            inputs, outputs = model.get_inputs(), model.get_outputs()
            if len(inputs) != 1 or len(outputs) != 1:
                raise ValueError("NVIDIA policy requires one input and one output")
            for tensor, size in ((inputs[0], 516), (outputs[0], 15)):
                if (tensor.type != "tensor(float)" or len(tensor.shape) != 2
                        or tensor.shape[1] != size
                        or not (tensor.shape[0] in (1, None) or isinstance(tensor.shape[0], str))):
                    raise ValueError(f"unexpected NVIDIA tensor: {tensor.shape} {tensor.type}")
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.reset()

    def reset(self):
        self.history = np.zeros((6, 86), dtype=np.float32)
        self.previous_action = np.zeros(15, dtype=np.float32)
        self.last_raw = np.empty(0, dtype=np.float32)
        self.use_balance = True

    def observation(self, q, dq, quat, base_velocity, command):
        q = vector(q, 29, "joint positions").astype(np.float32)
        dq = vector(dq, 29, "joint velocities").astype(np.float32)
        quat = vector(quat, 4, "quaternion")
        velocity = vector(base_velocity, 6, "base velocity")
        command = vector(command, 3, "velocity command").astype(np.float32)
        frame = np.zeros(86, dtype=np.float32)
        frame[:3] = command * [2.0, 2.0, 0.5]
        frame[3] = 0.74
        frame[7:10] = velocity[3:] * 0.5
        frame[10:13] = inverse_rotate(quat, [0, 0, -1])
        frame[13:42] = q - self.default
        frame[42:71] = dq * 0.05
        frame[71:] = self.previous_action
        if not np.isfinite(frame).all():
            raise PolicyViolation("nonfinite_or_invalid_observation")
        self.history[:-1] = self.history[1:]
        self.history[-1] = frame
        self.use_balance = bool(np.linalg.norm(command) < 0.05)
        return self.history.reshape(1, 516).copy()

    def infer(self, observation):
        if observation.shape != (1, 516) or not np.isfinite(observation).all():
            raise PolicyViolation("nonfinite_or_invalid_observation")
        model = self.balance_session if self.use_balance else self.session
        output = np.asarray(model.run(None, {model.get_inputs()[0].name: observation})[0])
        self.last_raw = output.copy()
        if output.shape != (1, 15) or not np.isfinite(output).all():
            raise PolicyViolation("nonfinite_or_invalid_action")
        self.previous_action = output[0].astype(np.float32).copy()
        return self.previous_action.copy()

    def decode(self, raw):
        raw = vector(raw, 15, "NVIDIA action")
        target = self.default.astype(np.float64).copy()
        target[:15] += raw * 0.25
        if not np.isfinite(target).all():
            raise PolicyViolation("nonfinite_target")
        return target

    def snapshot(self):
        return self.history.copy(), self.previous_action.copy(), self.use_balance

    def restore(self, state):
        history, action, use_balance = state
        self.history = np.asarray(history, dtype=np.float32).copy()
        self.previous_action = vector(action, 15, "previous action").astype(np.float32)
        self.use_balance = bool(use_balance)

    def command_velocity_and_torque(self):
        return np.zeros(29), np.zeros(29)


ADAPTERS = {policy.contract: policy for policy in (G1Walker, NvidiaWalker)}
