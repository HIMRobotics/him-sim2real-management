"""Peak joint torque over the dance against the joint profile's max_torque_nm.

Runs the sim Runner in lockstep (the Docker code path without DDS) and samples
every 2 ms physics step while the dance is in control. By default safety checks
are off, as the dance is run. --safety-checks puts the hardware adapter in the
loop on the sim clock: 90% torque-target clipping, the hard torque fault and
Holosoma recovery. It needs HIM_HOLOSOMA_ONNX.
"""

import argparse
import importlib
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SUBMISSION = REPO / "dance/submission"
PROFILE = SUBMISSION / "joint_profile.json"
os.environ.setdefault("G1_JOINT_PROFILE", str(PROFILE))
os.environ.setdefault("G1_SIM_THRASH_MODE", "off")
sys.path[:0] = [str(REPO / "packages/policy_safety/src"), str(REPO / "robot_safety"),
                str(REPO / "containers/sim"), str(REPO), str(SUBMISSION)]

import numpy as np
from runner import Runner


def run(seconds=52.0, safety_checks=False):
    module = importlib.import_module("policy")
    pol = module.Policy(SUBMISSION / "assets")
    runner = Runner(safety_checks=safety_checks)
    plant = runner.plant
    names, limits = [""] * 29, np.zeros(29)
    for i, name, entry in runner.rules.entries:
        names[i], limits[i] = name, entry.max_torque_nm
    demand_peak, applied_peak = np.zeros(29), np.zeros(29)
    demand_at, applied_at = np.zeros(29), np.zeros(29)
    over_secs = np.zeros(29)
    step_physics = plant.step_physics

    def logged_step():
        dq = step_physics()
        if runner.active is not None and runner.startup.handoff is not None and runner.reason is None:
            a = runner.active
            demand = np.abs(plant.demand(a["q"], a["dq"], a["kp"], a["kd"], a["tau"], plant.q(), plant.dq()))
            applied = np.abs(plant.data.qfrc_actuator[plant.vadr])
            t = runner.sim_time + 0.002
            for peak, at, value in ((demand_peak, demand_at, demand), (applied_peak, applied_at, applied)):
                higher = value > peak
                peak[higher], at[higher] = value[higher], t
            over_secs[applied > limits] += 0.002
        return dq

    plant.step_physics = logged_step
    for step in range(int(seconds / 0.02)):
        t = step * 0.02
        state = runner.feedback(t)
        target = pol.act(state)
        runner.command({
            "session_id": runner.session_id, "sequence": step, "observation_id": state["observation_id"],
            "q": list(map(float, target)), "dq": [0.0] * 29, "kp": list(map(float, module.KP)),
            "kd": list(map(float, module.KD)), "tau": [0.0] * 29,
        }, t)
        runner.step(t)
        if plant.state()["height"] < 0.45:
            print(f"fell at {runner.sim_time:.2f} s")
            break
    print(f"handoff {runner.startup.handoff} s, clip frames {pol.frame}, sampled after handoff")
    if runner.reason:
        print(f"stopped at {runner.stop_sim_time:.2f} s: {runner.reason}")
    print()
    print(f"{'joint':22s} {'limit':>6s} {'applied':>8s} {'%':>5s} {'at s':>6s} {'over s':>7s} {'PD demand':>10s} {'%':>5s}")
    for i in np.argsort(-applied_peak / limits):
        print(f"{names[i]:22s} {limits[i]:6.0f} {applied_peak[i]:8.1f} {100 * applied_peak[i] / limits[i]:5.0f} "
              f"{applied_at[i]:6.2f} {over_secs[i]:7.3f} {demand_peak[i]:10.1f} {100 * demand_peak[i] / limits[i]:5.0f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--safety-checks", action="store_true")
    run(safety_checks=parser.parse_args().safety_checks)
