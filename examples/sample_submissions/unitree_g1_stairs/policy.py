"""Parkour policy v4: pretrained walker ONNX + scripted high-level phases.

Labeled: pretrained locomotion (third-party ONNX) + scripted phase commands.
NOT trained for this course by us. NOT hardware-ready.

Free-base: act() returns ONLY joint position targets for MuJoCo position
actuators. No assist_target / no root pose commands.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from parkour_env import HOME_CTRL

ROOT = Path(__file__).resolve().parent
POLICY_DIR = ROOT / "policies"
WALKER_ONNX = POLICY_DIR / "walker.onnx"
CONFIG_PATH = POLICY_DIR / "model_config.json"

try:
    import onnxruntime as ort
except ImportError:  # pragma: no cover
    ort = None  # type: ignore


def _quat_apply_inverse(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    w, x, y, z = map(float, quat)
    w, x, y, z = w, -x, -y, -z
    vx, vy, vz = map(float, vec)
    return np.array(
        [
            (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - w * z) * vy + 2 * (x * z + w * y) * vz,
            2 * (x * y + w * z) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - w * x) * vz,
            2 * (x * z - w * y) * vx + 2 * (y * z + w * x) * vy + (1 - 2 * (x * x + y * y)) * vz,
        ],
        dtype=np.float32,
    )


class ScriptedParkourPolicy:
    """Phase machine driving a pretrained G1 walker via velocity commands.

    Controller type: pretrained+scripted (walker ONNX + scripted cmd schedule).
    Base mode: free-base.
    """

    name = "pretrained_walker_scripted_parkour_v4"
    controller_type = "pretrained+scripted"
    base_mode = "free-base"

    def __init__(self) -> None:
        if ort is None:
            raise ImportError("onnxruntime is required for ScriptedParkourPolicy")
        cfg = json.loads(CONFIG_PATH.read_text())
        self.joint_names = list(cfg["joint_names"])
        self.default = np.array([cfg["default_joint_pos"][n] for n in self.joint_names], dtype=np.float32)
        self.scales = np.array([cfg["action_scales"][n] for n in self.joint_names], dtype=np.float32)
        self._sess = ort.InferenceSession(str(WALKER_ONNX), providers=["CPUExecutionProvider"])
        self._inp = self._sess.get_inputs()[0].name
        self._out = self._sess.get_outputs()[0].name
        self.reset()

    def reset(self) -> None:
        self.current_phase = "walk"
        self._phase_t = 0.0
        self._last_action = np.zeros(29, dtype=np.float32)
        self._stood_latch = False

    def _cmd_for_phase(self, obs: dict[str, Any]) -> np.ndarray:
        x, y, z = map(float, obs["pelvis_pos"])
        quat = np.asarray(obs["pelvis_quat"], dtype=np.float64)
        qw, qx, qy, qz = map(float, quat)
        upright = 1.0 - 2.0 * (qx * qx + qy * qy)

        if self.current_phase == "walk":
            if x >= 1.55 and abs(y) < 0.35 and z > 0.60 and upright > 0.75:
                self._stood_latch = True
                self.current_phase = "stabilize"
                self._phase_t = 0.0
            vx = 0.55
        elif self.current_phase == "stabilize":
            vx = 0.05
            self._phase_t += 0.02
            if self._phase_t > 0.80:
                self.current_phase = "cross"
                self._phase_t = 0.0
        elif self.current_phase == "cross":
            vx = 0.90
            self._phase_t += 0.02
            if x >= 2.35 or self._phase_t > 8.0:
                self.current_phase = "finish_hold"
                self._phase_t = 0.0
        else:
            vx = 0.0
            self._phase_t += 0.02

        vy = float(np.clip(-1.2 * y, -0.50, 0.50))
        yaw = float(np.clip(-1.8 * y, -0.80, 0.80))
        return np.array([vx, vy, yaw], dtype=np.float32)

    def act(self, observation: dict[str, Any]) -> np.ndarray:
        low = np.asarray(observation["ctrl_low"], dtype=np.float64)
        high = np.asarray(observation["ctrl_high"], dtype=np.float64)
        nu = int(observation["nu"])

        quat = np.asarray(observation["pelvis_quat"], dtype=np.float64)
        qvel = np.asarray(observation["qvel"], dtype=np.float64)
        joint_qpos = np.asarray(observation["joint_qpos"], dtype=np.float32)
        joint_qvel = np.asarray(observation["joint_qvel"], dtype=np.float32)

        lin = _quat_apply_inverse(quat, qvel[0:3])
        ang = qvel[3:6].astype(np.float32)
        pg = _quat_apply_inverse(quat, np.array([0.0, 0.0, -1.0]))
        jp = (joint_qpos[:29] - self.default).astype(np.float32)
        jv = joint_qvel[:29].astype(np.float32)
        cmd = self._cmd_for_phase(observation)

        obs = np.concatenate([lin, ang, pg, jp, jv, self._last_action, cmd]).astype(np.float32)
        action = self._sess.run([self._out], {self._inp: obs.reshape(1, -1)})[0][0]
        target = self.default + action * self.scales
        # Soft hop when on top edge (x from obs)
        x = float(observation["pelvis_pos"][0])
        if self.current_phase == "cross" and 1.88 <= x < 2.05:
            # quick crouch then extend for ~0.35s window using phase timer
            u = (self._phase_t % 0.8)
            if u < 0.25:
                target[0] = target[6] = -0.55
                target[3] = target[9] = 1.05
                target[4] = target[10] = -0.48
            elif u < 0.45:
                target[0] = target[6] = 0.15
                target[3] = target[9] = 0.10
                target[4] = target[10] = 0.05
        for i, name in enumerate(self.joint_names):
            if any(p in name for p in ("shoulder", "elbow", "wrist")):
                target[i] = self.default[i]
        self._last_action = action.astype(np.float32).copy()
        return np.clip(target[:nu], low, high).astype(np.float64)


class NeutralPolicy:
    name = "neutral"
    controller_type = "neutral"
    base_mode = "free-base"

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.current_phase = "hold"

    def act(self, observation: dict[str, Any]) -> np.ndarray:
        low = np.asarray(observation["ctrl_low"], dtype=np.float64)
        high = np.asarray(observation["ctrl_high"], dtype=np.float64)
        nu = int(observation["nu"])
        return np.clip(HOME_CTRL[:nu], low, high)
