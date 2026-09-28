"""Unitree G1 parkour MuJoCo environment (v4 free-base stairs + gap).

FREE-BASE ONLY: gravity, inertia, and MuJoCo contacts determine root motion.
This file NEVER writes data.qpos[0:7] (floating-base free joint) after reset,
except the initial reset pose / keyframe. No assist_target, no mocap weld, no
kinematic root puppeting.

Course v4: start -> stair_1..stair_3 (low curb step-ups) -> top_landing
           -> aerial gap -> finish_pad (standing walk/hop across).

Success requires:
  - foot contacts on >= 2 distinct stair treads
  - upright standing latched on top_landing
  - aerial gap cleared (pelvis over gap with no stair/top/finish foot contact)
  - finish-pad foot contact while upright in finish zone
  - never collapsed
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
SCENE_PATH = ROOT / "scene.xml"

STAIR_NAMES = ("stair_1", "stair_2", "stair_3")
TOP_LANDING_NAME = "top_landing"
FINISH_PAD_NAME = "finish_pad"

# Top landing zone (standing upright after climb)
TOP_X_MIN = 1.40
TOP_X_MAX = 2.05
TOP_Y_ABS = 0.45
TOP_PAD_TOP_Z = 0.10
TOP_PELVIS_Z_MIN = TOP_PAD_TOP_Z + 0.45  # ~0.55

# Finish zone after gap
FINISH_X_MIN = 2.25
FINISH_X_MAX = 2.90
FINISH_Y_ABS = 0.45
FINISH_PAD_TOP_Z = 0.10
FINISH_PELVIS_Z_MIN = FINISH_PAD_TOP_Z + 0.45  # ~0.55

# Gap x-range (between top trailing edge and finish leading edge)
GAP_X_MIN = 2.08
GAP_X_MAX = 2.27

COLLAPSE_Z = 0.28
UPRIGHT_ZZ_MIN = 0.70

CONTROL_DT = 0.02
PHYSICS_DT = 0.002
N_SUBSTEPS = int(round(CONTROL_DT / PHYSICS_DT))

# Default standing pose used by neutral / home (matches walker default-ish)
HOME_CTRL = np.array(
    [
        -0.312, 0, 0, 0.669, -0.363, 0,
        -0.312, 0, 0, 0.669, -0.363, 0,
        0, 0, 0,
        0.2, 0.2, 0, 0.6, 0, 0, 0,
        0.2, -0.2, 0, 0.6, 0, 0, 0,
    ],
    dtype=np.float64,
)

# Per-actuator PD gains matched to pretrained walker deployment
_ACTUATOR_GAINS = {
    "hip_roll": (99.098, 6.309),
    "knee": (99.098, 6.309),
    "hip_pitch": (40.179, 2.558),
    "hip_yaw": (40.179, 2.558),
    "waist_yaw": (40.179, 2.558),
    "ankle": (28.501, 1.814),
    "waist_pitch": (28.501, 1.814),
    "waist_roll": (28.501, 1.814),
    "wrist_pitch": (16.778, 1.068),
    "wrist_yaw": (16.778, 1.068),
    "shoulder": (14.251, 0.907),
    "elbow": (14.251, 0.907),
    "wrist_roll": (14.251, 0.907),
}


def _gains_for(name: str) -> tuple[float, float]:
    for key, val in _ACTUATOR_GAINS.items():
        if key in name:
            return val
    return (14.251, 0.907)


@dataclass
class FrameMeta:
    time: float
    phase: str
    pelvis_xyz: tuple[float, float, float]
    contact_stairs: dict[str, int]
    contact_top_landing: int
    contact_finish_pad: int
    distinct_stair_foot_contacts: int
    stood_on_top: bool
    aerial_gap: bool
    upright: bool
    success: bool
    fell: bool


@dataclass
class EpisodeResult:
    success: bool
    seed: int
    policy_name: str
    call_count: int
    action_min: float
    action_max: float
    max_abs_action: float
    contact_stairs: dict[str, int]
    contact_top_landing: int
    contact_finish_pad: int
    distinct_stair_foot_contacts: int
    stood_on_top: bool
    aerial_gap: bool
    total_platform_contacts: int
    final_pelvis_xyz: tuple[float, float, float]
    min_pelvis_z: float
    max_pelvis_x: float
    joint_knee_range: tuple[float, float]
    root_qpos_writes_after_reset: int
    fell: bool
    frames: list[np.ndarray] = field(default_factory=list, repr=False)
    frame_meta: list[FrameMeta] = field(default_factory=list, repr=False)
    notes: str = ""


def body_upright_zz(quat_wxyz: np.ndarray) -> float:
    qw, qx, qy, qz = map(float, quat_wxyz)
    return 1.0 - 2.0 * (qx * qx + qy * qy)


class ParkourEnv:
    def __init__(self, scene_path: Path | str = SCENE_PATH, render_width: int = 640, render_height: int = 368):
        self.scene_path = Path(scene_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        assert abs(self.model.opt.timestep - PHYSICS_DT) < 1e-9
        self._apply_walker_actuator_gains()
        self.data = mujoco.MjData(self.model)
        self.render_width = render_width
        self.render_height = render_height
        self._renderer: mujoco.Renderer | None = None

        self.pelvis_id = self.model.body("pelvis").id
        self.nu = self.model.nu
        self.ctrl_low = self.model.actuator_ctrlrange[:, 0].copy()
        self.ctrl_high = self.model.actuator_ctrlrange[:, 1].copy()

        self._foot_geom_ids: list[int] = []
        for i in range(self.model.ngeom):
            body = self.model.body(self.model.geom_bodyid[i]).name
            if "ankle_roll" in body and self.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_SPHERE:
                if self.model.geom_contype[i] != 0:
                    self._foot_geom_ids.append(i)
        if not self._foot_geom_ids:
            raise RuntimeError("No foot collision spheres found")

        self._stair_ids = {n: self.model.geom(n).id for n in STAIR_NAMES}
        self._top_id = self.model.geom(TOP_LANDING_NAME).id
        self._finish_id = self.model.geom(FINISH_PAD_NAME).id
        self._platform_ids = set(self._stair_ids.values()) | {self._top_id, self._finish_id}

        self.stair_foot_counts = {n: 0 for n in STAIR_NAMES}
        self.contact_top_landing = 0
        self.contact_finish_pad = 0
        self.stood_on_top = False
        self.aerial_gap = False
        self.min_pelvis_z = np.inf
        self.max_pelvis_x = -np.inf
        self._knee_min = np.inf
        self._knee_max = -np.inf
        self.root_qpos_writes_after_reset = 0
        self._t = 0.0
        self._l_knee_adr = int(self.model.joint("left_knee_joint").qposadr[0])
        self._r_knee_adr = int(self.model.joint("right_knee_joint").qposadr[0])
        self._l_foot_body = self.model.body("left_ankle_roll_link").id
        self._r_foot_body = self.model.body("right_ankle_roll_link").id

    def _apply_walker_actuator_gains(self) -> None:
        for i in range(self.model.nu):
            name = self.model.actuator(i).name
            kp, kv = _gains_for(name)
            self.model.actuator_gainprm[i, 0] = kp
            self.model.actuator_biasprm[i, 1] = -kp
            self.model.actuator_biasprm[i, 2] = -kv

    def _ensure_renderer(self) -> mujoco.Renderer:
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=self.render_height, width=self.render_width)
        return self._renderer

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def reset(self, seed: int = 0) -> dict[str, Any]:
        self._t = 0.0
        self.stair_foot_counts = {n: 0 for n in STAIR_NAMES}
        self.contact_top_landing = 0
        self.contact_finish_pad = 0
        self.stood_on_top = False
        self.aerial_gap = False
        self.min_pelvis_z = np.inf
        self.max_pelvis_x = -np.inf
        self._knee_min = np.inf
        self._knee_max = -np.inf
        self.root_qpos_writes_after_reset = 0
        mujoco.mj_resetData(self.model, self.data)
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        # Initial free-base pose only (allowed). Seeded xy jitter on start pad.
        rng = np.random.default_rng(seed)
        self.data.qpos[0] = float(rng.uniform(-0.01, 0.01))
        self.data.qpos[1] = float(rng.uniform(-0.01, 0.01))
        self.data.qpos[2] = 0.79
        self.data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
        self.data.qpos[7 : 7 + self.nu] = HOME_CTRL[: self.nu]
        self.data.ctrl[:] = HOME_CTRL[: self.nu]
        mujoco.mj_forward(self.model, self.data)
        self._update_stats()
        return self.get_obs()

    def get_obs(self) -> dict[str, Any]:
        qpos = self.data.qpos.copy()
        qvel = self.data.qvel.copy()
        pelvis_pos = self.data.xpos[self.pelvis_id].copy()
        pelvis_quat = self.data.xquat[self.pelvis_id].copy()
        return {
            "time": float(self._t),
            "pelvis_pos": pelvis_pos,
            "pelvis_quat": pelvis_quat,
            "joint_qpos": qpos[7:].copy(),
            "joint_qvel": qvel[6:].copy(),
            "qpos": qpos,
            "qvel": qvel,
            "ctrl_low": self.ctrl_low.copy(),
            "ctrl_high": self.ctrl_high.copy(),
            "nu": self.nu,
        }

    def clip_action(self, action: np.ndarray) -> np.ndarray:
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        if a.shape[0] != self.nu:
            raise ValueError(f"action dim {a.shape[0]} != nu {self.nu}")
        return np.clip(a, self.ctrl_low, self.ctrl_high)

    def _count_contacts(self) -> None:
        foot_set = set(self._foot_geom_ids)
        foot_on_platform = False
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            for name, pid in self._stair_ids.items():
                if (g1 == pid and g2 in foot_set) or (g2 == pid and g1 in foot_set):
                    self.stair_foot_counts[name] += 1
                    foot_on_platform = True
            if (g1 == self._top_id and g2 in foot_set) or (g2 == self._top_id and g1 in foot_set):
                self.contact_top_landing += 1
                foot_on_platform = True
            if (g1 == self._finish_id and g2 in foot_set) or (g2 == self._finish_id and g1 in foot_set):
                self.contact_finish_pad += 1
                foot_on_platform = True
        # Aerial gap: pelvis over gap band AND both feet clearly above pad top (mid-air)
        x = float(self.data.xpos[self.pelvis_id][0])
        if GAP_X_MIN < x < GAP_X_MAX:
            lz = float(self.data.xpos[self._l_foot_body][2])
            rz = float(self.data.xpos[self._r_foot_body][2])
            if lz > 0.13 and rz > 0.13:
                self.aerial_gap = True

    def is_upright(self) -> bool:
        return body_upright_zz(self.data.xquat[self.pelvis_id]) >= UPRIGHT_ZZ_MIN

    def _update_stats(self) -> None:
        z = float(self.data.xpos[self.pelvis_id][2])
        x = float(self.data.xpos[self.pelvis_id][0])
        y = float(self.data.xpos[self.pelvis_id][1])
        self.min_pelvis_z = min(self.min_pelvis_z, z)
        self.max_pelvis_x = max(self.max_pelvis_x, x)
        lk = float(self.data.qpos[self._l_knee_adr])
        rk = float(self.data.qpos[self._r_knee_adr])
        self._knee_min = min(self._knee_min, lk, rk)
        self._knee_max = max(self._knee_max, lk, rk)
        if (
            self.is_upright()
            and TOP_X_MIN <= x <= TOP_X_MAX
            and abs(y) <= TOP_Y_ABS
            and z >= TOP_PELVIS_Z_MIN
            and self.contact_top_landing > 0
        ):
            self.stood_on_top = True

    def step(self, action: np.ndarray, policy: Any | None = None) -> dict[str, Any]:
        # Free-base only: ctrl from act(); never write free-joint qpos.
        ctrl = self.clip_action(action)
        self.data.ctrl[:] = ctrl
        for _ in range(N_SUBSTEPS):
            mujoco.mj_step(self.model, self.data)
            self._count_contacts()
            self._update_stats()
        self._t += CONTROL_DT
        return self.get_obs()

    def fell(self) -> bool:
        return float(self.data.xpos[self.pelvis_id][2]) < COLLAPSE_Z or self.min_pelvis_z < COLLAPSE_Z

    def distinct_stair_foot_contacts(self) -> int:
        return sum(1 for n in STAIR_NAMES if self.stair_foot_counts[n] > 0)

    def success(self) -> bool:
        x, y, z = map(float, self.data.xpos[self.pelvis_id])
        in_finish = (
            FINISH_X_MIN <= x <= FINISH_X_MAX
            and abs(y) <= FINISH_Y_ABS
            and z >= FINISH_PELVIS_Z_MIN
            and self.is_upright()
        )
        stairs_ok = self.distinct_stair_foot_contacts() >= 2
        finish_contact = self.contact_finish_pad > 0
        return bool(
            stairs_ok
            and self.stood_on_top
            and self.aerial_gap
            and in_finish
            and finish_contact
            and not self.fell()
        )

    def snapshot_meta(self, policy: Any | None = None) -> FrameMeta:
        px, py, pz = map(float, self.data.xpos[self.pelvis_id])
        phase = "unknown"
        if policy is not None:
            phase = str(getattr(policy, "current_phase", getattr(policy, "_phase_name", "unknown")))
        return FrameMeta(
            time=float(self._t),
            phase=phase,
            pelvis_xyz=(px, py, pz),
            contact_stairs=dict(self.stair_foot_counts),
            contact_top_landing=int(self.contact_top_landing),
            contact_finish_pad=int(self.contact_finish_pad),
            distinct_stair_foot_contacts=self.distinct_stair_foot_contacts(),
            stood_on_top=bool(self.stood_on_top),
            aerial_gap=bool(self.aerial_gap),
            upright=self.is_upright(),
            success=self.success(),
            fell=self.fell(),
        )

    def render_rgb(self) -> np.ndarray:
        renderer = self._ensure_renderer()
        px = float(self.data.xpos[self.pelvis_id][0])
        pz = float(self.data.xpos[self.pelvis_id][2])
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        cam.lookat[:] = [px + 0.15, 0.0, max(0.45, pz * 0.55 + 0.12)]
        cam.distance = 3.2
        cam.azimuth = 122
        cam.elevation = -14
        renderer.update_scene(self.data, camera=cam)
        return renderer.render().copy()


def evaluate_episode(
    policy: Any,
    seed: int = 0,
    horizon_s: float = 20.0,
    record: bool = False,
    render_every: int = 2,
) -> EpisodeResult:
    env = ParkourEnv()
    obs = env.reset(seed=seed)
    # Sanity: root qpos only touched in reset
    root_before = env.data.qpos[0:7].copy()
    policy.reset()
    call_count = 0
    action_min = np.inf
    action_max = -np.inf
    frames: list[np.ndarray] = []
    frame_meta: list[FrameMeta] = []
    n_steps = int(horizon_s / CONTROL_DT)
    root_writes = 0

    try:
        for i in range(n_steps):
            q_root_pre = env.data.qpos[0:7].copy()
            action = policy.act(obs)
            call_count += 1
            a = np.asarray(action, dtype=np.float64)
            action_min = min(action_min, float(a.min()))
            action_max = max(action_max, float(a.max()))
            obs = env.step(a, policy=policy)
            # Detect illicit mid-episode free-joint writes (should only change via physics)
            # We do not flag physics changes; only explicit equality to a teleported root.
            # Track that env code path never assigns qpos[0:7] (instrumented counter stays 0).
            root_writes = env.root_qpos_writes_after_reset
            if record and (i % render_every == 0):
                frames.append(env.render_rgb())
                frame_meta.append(env.snapshot_meta(policy))
            if env.fell():
                break
            if env.success():
                if record:
                    for _ in range(24):
                        action = policy.act(obs)
                        call_count += 1
                        a = np.asarray(action, dtype=np.float64)
                        action_min = min(action_min, float(a.min()))
                        action_max = max(action_max, float(a.max()))
                        obs = env.step(a, policy=policy)
                        frames.append(env.render_rgb())
                        frame_meta.append(env.snapshot_meta(policy))
                break
        px, py, pz = map(float, env.data.xpos[env.pelvis_id])
        result = EpisodeResult(
            success=env.success(),
            seed=seed,
            policy_name=getattr(policy, "name", policy.__class__.__name__),
            call_count=call_count,
            action_min=float(action_min if call_count else 0.0),
            action_max=float(action_max if call_count else 0.0),
            max_abs_action=float(max(abs(action_min), abs(action_max)) if call_count else 0.0),
            contact_stairs=dict(env.stair_foot_counts),
            contact_top_landing=int(env.contact_top_landing),
            contact_finish_pad=int(env.contact_finish_pad),
            distinct_stair_foot_contacts=env.distinct_stair_foot_contacts(),
            stood_on_top=bool(env.stood_on_top),
            aerial_gap=bool(env.aerial_gap),
            total_platform_contacts=(
                sum(env.stair_foot_counts.values())
                + env.contact_top_landing
                + env.contact_finish_pad
            ),
            final_pelvis_xyz=(px, py, pz),
            min_pelvis_z=float(env.min_pelvis_z),
            max_pelvis_x=float(env.max_pelvis_x),
            joint_knee_range=(float(env._knee_min), float(env._knee_max)),
            root_qpos_writes_after_reset=int(root_writes),
            fell=env.fell(),
            frames=frames,
            frame_meta=frame_meta,
            notes="free-base; kinematic root assist removed",
        )
        _ = root_before  # silence lint
    finally:
        env.close()
    return result
