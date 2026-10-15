"""Repeatable safety faults for the standard local simulation runner."""

import math

import mujoco

CASES = ("recovery_tilt", "policy_tilt", "gantry_tilt", "policy_crouch")


class SafetyScenario:
    """Inject one perturbation after measured startup readiness or handoff."""

    def __init__(self, name, tilt_degrees=20):
        if name not in CASES:
            raise ValueError("unknown safety scenario")
        if not math.isfinite(tilt_degrees) or not 0 <= tilt_degrees <= 90:
            raise ValueError("tilt must be between 0 and 90 degrees")
        self.name = name
        self.tilt_degrees = tilt_degrees
        self.event = None

    def apply(self, runner):
        if self.event is not None:
            return
        handoff = runner.startup.handoff
        eligible = (runner.startup.ready if self.name == "gantry_tilt" else
                    handoff is not None and runner.sim_time >= handoff + 1.5)
        if not eligible:
            return
        plant = runner.plant
        angle = math.radians(self.tilt_degrees) / 2
        if self.name == "policy_crouch":
            angle = 0
            plant.data.qpos[plant.qadr[[0, 6]]] = -1.2
            plant.data.qpos[plant.qadr[[3, 9]]] = 2.4
            plant.data.qpos[plant.qadr[[4, 10]]] = -1.2
        plant.data.qpos[plant.base_qadr + 3:plant.base_qadr + 7] = [
            math.cos(angle), 0, math.sin(angle), 0]
        plant.data.qvel[:] = 0
        plant.data.qpos[plant.base_qadr + 2] = 1.2
        plant._place_on_floor(plant.data, clearance=0.001)
        mujoco.mj_forward(plant.model, plant.data)
        runner.hardware.receive()
        runner._sync_hardware()
        self.event = {"case": self.name, "time_secs": runner.sim_time,
                      "tilt_degrees": math.degrees(angle * 2), "initial_velocity": "zero",
                      "standing_height_m": runner.hardware.snapshot()["standing_height_m"]}
        if self.name == "recovery_tilt":
            runner.handle({"op": "abort", "reason": "simulated_policy_instability"})

    def report(self, runner):
        state = runner.hardware.snapshot()
        if self.name == "recovery_tilt":
            passed = (self.event is not None and state["phase"] == "recovery"
                      and state["recovery_writes"] > 0
                      and state["upright"] > math.cos(math.radians(5)))
        else:
            passed = (self.event is not None and state["phase"] == "damping"
                      and state["recovery_writes"] == 0)
        return {"name": self.name, "event": self.event, "passed": bool(passed),
                "limitations": "Imposed pose with zero velocity. No gantry mechanics or SDK latency."}
