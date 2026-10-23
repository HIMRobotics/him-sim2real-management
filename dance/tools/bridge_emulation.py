"""Emulate the sim bridge's wall-clock timing: physics ticks every 20 ms, the policy
node fires on its own timer (or on each state) and its command lands on the next tick."""

import importlib
import sys

from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SUBMISSION = REPO / "dance/submission"
sys.path[:0] = [str(REPO / "containers/sim"), str(REPO / "robot_safety"), str(REPO), str(SUBMISSION)]

import numpy as np
from plant import Plant
from robot_safety.startup_balance import StartupBalance


def run(node_period=None, clock=True, seconds=52.0, friction=0.3, reply=0.002):
    import policy as module
    importlib.reload(module)
    pol = module.Policy(SUBMISSION / "assets")
    plant = Plant()
    plant.model.dof_frictionloss[plant.vadr] = friction
    startup = StartupBalance()
    startup.prepare()
    lo = plant.model.actuator_ctrlrange[plant.actuators, 0]
    hi = plant.model.actuator_ctrlrange[plant.actuators, 1]
    submitted, acts, next_fire, handled, published = None, 0, 0.0, -1, None
    for tick in range(int(seconds / 0.02)):
        now = tick * 0.02
        step_end = now
        # Node: event-driven replies to the newest state; timer mode polls at node_period.
        if published is not None:
            fire_times = []
            if node_period is None:
                fire_times = [published[0] + reply]
            else:
                while next_fire < now:
                    fire_times.append(next_fire)
                    next_fire += node_period
            for fire in fire_times:
                if published[1] > handled and published[0] <= fire < now:
                    handled = published[1]
                    state = dict(published[2])
                    if not clock:
                        state.pop("sim_time", None)
                    q = pol.act(state)
                    acts += 1
                    submitted = {"q": q, "dq": np.zeros(29), "kp": module.KP, "kd": module.KD, "tau": np.zeros(29)}
        cmd = startup.command(now, now, plant.state(), submitted, automatic=now >= 5.0)
        plant.apply(dict(cmd, q=np.clip(cmd["q"], lo, hi)))
        for _ in range(10):
            plant.step_physics()
        state = plant.state()
        state.update(startup.status())
        published = (now, tick, state)
        if plant.state()["height"] < 0.45:
            return {"fell_at_sim_s": round(now, 2), "policy_calls": acts, "ticks": tick + 1,
                    "clip_frame_at_fall_s": round(pol.frame * 0.02, 2)}
    return {"fell_at_sim_s": None, "policy_calls": acts, "ticks": int(seconds / 0.02),
            "clip_frame_at_end_s": round(pol.frame * 0.02, 2)}


if __name__ == "__main__":
    period = 0.02 * 2600 / 1911
    print("timer, per-call clip clock (the run you saw):", run(period, clock=False), flush=True)
    print("timer, time-based clip clock:", run(period, clock=True), flush=True)
    print("act on each state, time-based clock (fix):", run(None, clock=True), flush=True)
