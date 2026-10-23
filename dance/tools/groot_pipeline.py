"""HIM sequence without ROS: GR00T startup, handover at 5 s, then the robust dance policy."""

import importlib
import sys

from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SUBMISSION = REPO / "dance/submission"
sys.path[:0] = [str(REPO / "containers/sim"), str(REPO / "robot_safety"), str(REPO), str(SUBMISSION)]

import numpy as np
from plant import Plant
from robot_safety.startup_balance import StartupBalance


def run(friction=0.3, seconds=60.0, delay_steps=0, foot_friction=None):
    import policy as module
    importlib.reload(module)
    pol = module.Policy(SUBMISSION / "assets")
    plant = Plant()
    plant.model.dof_frictionloss[plant.vadr] = friction
    if foot_friction is not None:
        for g in plant.robot_geoms:
            plant.model.geom_friction[g, 0] = foot_friction
    startup = StartupBalance()
    startup.prepare()
    lo = plant.model.actuator_ctrlrange[plant.actuators, 0]
    hi = plant.model.actuator_ctrlrange[plant.actuators, 1]
    t, handoff, errs, abs_errs, peak = 0.0, None, [], [], np.zeros(29)
    for _ in range(int(seconds / 0.02)):
        state = plant.state()
        state.update(startup.status())
        target = pol.act(state)
        submitted = {"q": target, "dq": np.zeros(29), "kp": module.KP, "kd": module.KD, "tau": np.zeros(29)}
        cmd = startup.command(t, t, plant.state(), submitted, automatic=t >= 5.0)
        cmd = dict(cmd, q=np.clip(cmd["q"], lo, hi))
        if delay_steps >= 10:
            cmd, previous = (previous if t > 0 else cmd), cmd
        for step in range(10):
            if step == delay_steps % 10:
                plant.apply(cmd)
            plant.step_physics()
        t = round(t + 0.02, 8)
        if startup.handoff is not None and handoff is None:
            handoff = t
        if pol.frame > 0:
            k = min(pol.frame, len(pol.ref_q) - 1)
            e = plant.q() - pol.ref_q[k]
            errs.append(float(np.sqrt(np.mean(e ** 2))))
            abs_errs.append(np.abs(e))
            peak = np.maximum(peak, np.abs(plant.data.actuator_force[plant.actuators]))
            if pol.frame >= len(pol.ref_q):
                break
        if plant.state()["height"] < 0.45:
            break
    fell = plant.state()["height"] < 0.45
    per_joint = np.median(np.array(abs_errs), axis=0) if abs_errs else np.zeros(29)
    return {"fell_at_clip_s": round(pol.frame * 0.02, 2) if fell else None,
            "clip_s_played": round(min(pol.frame, len(pol.ref_q)) * 0.02, 2),
            "median_rms_joint_err": round(float(np.median(errs)), 3),
            "median_abs_joint_err": round(float(np.median(np.array(abs_errs))), 3),
            "worst_joint_median": (module.UNITREE_JOINTS[int(per_joint.argmax())], round(float(per_joint.max()), 3)),
            "peak_torque": {n: round(float(peak[i]), 1) for i, n in enumerate(module.UNITREE_JOINTS)
                            if "ankle" in n or n in ("waist_roll", "waist_pitch")}}


if __name__ == "__main__":
    for friction in (0.0, 0.3):
        print("friction", friction, run(friction), flush=True)
