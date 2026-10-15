"""Your policy. Replace the body of Policy; keep the class name and method signatures.

node.py calls Policy(assets) once at startup and Policy.act(state) once per new
observation, at up to 50 Hz. act() must return within 20 ms.
"""

import numpy as np

# Requested PD gains in Unitree motor order. The host joint profile supplies
# the gains that are actually applied; these only need to be within bounds
# (kp 0..500, kd 0..50).
KP = np.array([100.0] * 12 + [200.0] * 3 + [40.0] * 14)
KD = np.array([2.0] * 12 + [5.0] * 3 + [1.0] * 14)


class Policy:
    def __init__(self, assets):
        """`assets` is the Path of the image's assets/ directory, e.g. for an ONNX file:

        import onnxruntime
        self.session = onnxruntime.InferenceSession(str(assets / "policy.onnx"))
        """
        self.hold = None

    def act(self, state):
        """Return 29 joint position targets (rad) in Unitree motor order.

        `state` is the /safety/policy_state JSON: q, dq (29 each), quat (wxyz),
        gyro, and, when the host provides them, position and base_velocity.
        """
        if self.hold is None:
            self.hold = np.asarray(state["q"], dtype=float).copy()
        return self.hold
