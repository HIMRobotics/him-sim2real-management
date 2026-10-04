"""Accept container joint frames, check them, and step the MuJoCo plant."""

import json
import os
import time
import uuid
from pathlib import Path

import mujoco
import numpy as np
from joint_checks import (
    BASE_ANGULAR_SPEED_LIMIT_RAD_S,
    SPEED_HOLD_S,
    JointViolation,
)
from metrics import MotionMetrics
from plant import CONTROL_DT, PHYSICS_DT, POSES, Plant
from policy_safety.adapter import PolicyViolation
from policy_safety.joint_rules import JointRules

OBSERVATION_TIMEOUT = 0.2
ACTION_TIMEOUT = 0.2
SANITY = {
    "q": (-10.0, 10.0),
    "dq": (-50.0, 50.0),
    "kp": (0.0, 500.0),
    "kd": (0.0, 50.0),
    "tau": (-200.0, 200.0),
}


def _finite_vector(values):
    try:
        array = np.asarray(values, dtype=float)
    except (TypeError, ValueError):
        return False, np.empty(0)
    return array.shape == (29,) and bool(np.isfinite(array).all()), array


class Runner:
    """One simulated session. A failed check latches for the rest of the run."""

    def __init__(self, pose=None, scene=None, safety_checks=True, *, record_trace=None):
        pose = pose or os.environ.get("G1_SIM_POSE", "standing")
        if pose not in POSES:
            raise ValueError("unknown pose")
        self.safety_checks = bool(safety_checks)
        self.control_path = "submission"
        self.plant = Plant(pose, scene=scene, check_fall=self.safety_checks)
        self.rules = JointRules(self.plant.joint_names)
        self.session_id = "sim-" + uuid.uuid4().hex[:12]
        self.observation_id = 0
        self.observations = {}
        self.last_sequence = -1
        self.previous = None
        self.last_command_time = None
        self.active = None
        self.reason = None
        self.detail = None
        self.commands = 0
        self.steps = 0
        self.sim_time = 0.0
        speed_caps = np.full(29, np.inf)
        for index, _name, limits in self.rules.entries:
            speed_caps[index] = limits.max_velocity_rad_secs
        self.speed_caps = speed_caps
        self.hold_steps = max(1, round(SPEED_HOLD_S / PHYSICS_DT))
        self.joint_over = np.zeros(29, dtype=int)
        self.base_over = 0
        self.trace_qpos = []
        self.trace_time = []
        self.record_trace = (
            bool(os.environ.get("G1_SIM_TRACE"))
            if record_trace is None
            else record_trace
        )
        self.live_path = os.environ.get("G1_SIM_LIVE")
        self.metrics = MotionMetrics(self.plant)
        self._live = None
        self._circle = None
        self._remember()

    def arm(self):
        return self.session_id

    def feedback(self, time):
        self.observation_id += 1
        self.observations[self.observation_id] = float(time)
        if len(self.observations) > 512:
            oldest = next(iter(self.observations))
            del self.observations[oldest]
        state = self.plant.state()
        state["observation_id"] = self.observation_id
        state["start_button_pressed"] = False
        state["session_id"] = self.session_id
        return state

    def command(self, frame, time):
        if self.reason is not None:
            return self.reason
        reason, detail = self._validate(frame, float(time))
        if reason:
            # Keep the last accepted hold. Dropping it leaves kp at 0, and the
            # body folds even though the rejected target was never applied.
            self._latch(reason, detail, hold=True)
            return reason
        self.commands += 1
        self.last_sequence = int(frame["sequence"])
        self.last_command_time = float(time)
        self.previous = (
            np.asarray(frame["q"], dtype=float).copy(),
            np.asarray(frame["dq"], dtype=float).copy(),
        )
        self.active = {
            key: np.asarray(frame[key], dtype=float).copy()
            for key in ("q", "dq", "kp", "kd", "tau")
        }
        return None

    def step(self, time):
        if (
            self.safety_checks
            and self.reason is None
            and self.last_command_time is not None
            and float(time) - self.last_command_time >= ACTION_TIMEOUT
        ):
            self._latch("action_timeout", None)
        if self.active is None:
            q, dq = self.plant.q(), self.plant.dq()
            q_des, dq_des, kp, kd, tau = self.plant.damping_command(q, dq)
            self.plant._apply_arrays(q_des, dq_des, kp, kd, tau)
        else:
            self.plant.apply(self.active)
        violation = None
        for _ in range(self.plant.substeps):
            dq = self.plant.step_physics()
            self.sim_time += PHYSICS_DT
            self.metrics.observe()
            if (
                not np.isfinite(self.plant.data.qpos).all()
                or not np.isfinite(self.plant.data.qvel).all()
                or not np.isfinite(dq).all()
            ):
                violation = ("nonfinite_simulation", None)
                break
            if not self.safety_checks:
                continue
            self.joint_over = np.where(
                np.abs(dq) > self.speed_caps, self.joint_over + 1, 0
            )
            if np.any(self.joint_over >= self.hold_steps):
                index = int(np.argmax(self.joint_over))
                violation = (
                    "joint_velocity_limit",
                    {"joint": self.plant.joint_names[index]},
                )
                break
            base_speed = float(
                np.linalg.norm(
                    self.plant.data.qvel[
                        self.plant.base_vadr + 3 : self.plant.base_vadr + 6
                    ]
                )
            )
            self.base_over = (
                self.base_over + 1 if base_speed > BASE_ANGULAR_SPEED_LIMIT_RAD_S else 0
            )
            if self.base_over >= self.hold_steps:
                violation = ("base_angular_speed_limit", {"value": base_speed})
                break
        self.steps += 1
        if violation is None and self.plant.fallen():
            violation = ("fall", {"height": self.plant.state()["height"]})
        if violation is not None and self.reason is None:
            self._latch(*violation)
        self._remember()
        return {
            "reason": self.reason,
            "sim_time": self.sim_time,
            "commands": self.commands,
        }

    def checks(self):
        """Report enabled checks for the control path used by this run."""
        submission = self.control_path == "submission"
        return {
            "command_protocol": submission,
            "command_limits": submission and self.safety_checks,
            "observation_freshness": submission and self.safety_checks,
            "action_timeout": self.safety_checks,
            "joint_speed": self.safety_checks,
            "base_angular_speed": self.safety_checks,
            "fall": self.plant.check_fall,
            "finite_simulation": True,
        }

    def report(self, lowstate=False):
        if self._live is not None:
            self._live.close()
            self._live = None
        if self.reason:
            status = "violation" if self.safety_checks else "error"
        elif self.commands < 1:
            status = "no_command"
        else:
            status = "within_limits" if self.checks()["command_limits"] else "completed"
        trace = self._write_trace()
        return {
            "version": 1,
            "status": status,
            "reason": self.reason,
            "detail": self.detail,
            "commands": self.commands,
            "steps": self.steps,
            "seconds": self.sim_time,
            "lowstate": bool(lowstate),
            "hardware_authorization": False,
            "session_id": self.session_id,
            "pose": self.plant.pose,
            "scene": str(self.plant.scene) if self.plant.scene else None,
            "safety_checks": self.safety_checks,
            "control_path": self.control_path,
            "checks": self.checks(),
            "fall_check": self.plant.check_fall,
            "trace": trace,
            "motion": self.metrics.report(),
        }

    def _remember(self):
        if self.record_trace:
            self.trace_qpos.append(self.plant.data.qpos.copy())
            self.trace_time.append(self.sim_time)
        path = self.live_path
        if not path:
            return
        if self._live is None:
            target = Path(path)
            target.parent.mkdir(parents=True, exist_ok=True)
            self._live = target.open("w")
        qpos = [round(float(value), 5) for value in self.plant.data.qpos]
        self._live.write(json.dumps({"t": self.sim_time, "qpos": qpos}) + "\n")
        self._live.flush()

    def _write_trace(self):
        path = os.environ.get("G1_SIM_TRACE")
        if not path or not self.trace_qpos:
            return None
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        model = np.empty(mujoco.mj_sizeModel(self.plant.model), dtype=np.uint8)
        mujoco.mj_saveModel(self.plant.model, buffer=model)
        np.savez_compressed(
            target,
            time=np.asarray(self.trace_time, dtype=float),
            qpos=np.vstack(self.trace_qpos),
            model=model,
            safety_checks=self.safety_checks,
            control_path=self.control_path,
            checks=json.dumps(self.checks()),
        )
        return str(target)

    def circle(self, time):
        """One walking step of a flat circle. Movement limits are not applied."""
        self.control_path = "direct"
        try:
            if self._circle is None:
                from circle import CircleWalker

                self._circle = CircleWalker()
                self._circle.settle(self.plant)
                self._reset_trace()
            if self.reason is None:
                target = self._circle.targets(self.plant)
                self.active = {
                    "q": np.asarray(target, dtype=float).copy(),
                    "dq": np.zeros(29),
                    "kp": self._circle.kp.copy(),
                    "kd": self._circle.kd.copy(),
                    "tau": np.zeros(29),
                }
                self.commands += 1
                self.last_command_time = float(time)
                self._circle.started = True
        except Exception as exc:  # noqa: BLE001
            self._latch("plant_error", {"detail": str(exc)})
        return self.step(time)

    def _reset_trace(self):
        if self._live is not None:
            self._live.close()
            self._live = None
        path = self.live_path
        if path:
            Path(path).unlink(missing_ok=True)
        self.trace_qpos = []
        self.trace_time = []
        self.sim_time = 0.0
        self._remember()

    def handle(self, request):
        op = request.get("op")
        if op == "arm":
            return {"ok": True, "session_id": self.arm()}
        if op == "feedback":
            return {"ok": True, **self.feedback(request["time"])}
        if op == "command":
            reason = self.command(request["frame"], request["time"])
            return {"ok": reason is None, "reason": reason, "detail": self.detail}
        if op in ("circle", "step"):
            started = time.monotonic()
            result = (
                self.circle(request["time"])
                if op == "circle"
                else self.step(request["time"])
            )
            result["ok"] = result.get("reason") not in (
                "plant_error",
                "nonfinite_simulation",
            )
            result["detail"] = self.detail
            if request.get("feedback") and result["ok"]:
                # Both processes share the host monotonic clock.
                origin = request.get("clock_origin", started - request["time"])
                issued = time.monotonic() - origin
                result.update(self.feedback(issued))
            return result
        if op == "report":
            return self.report(request.get("lowstate", False))
        return {"ok": False, "reason": "unknown_op"}

    def _latch(self, reason, detail, hold=False):
        self.reason = reason
        self.detail = detail
        if not hold:
            self.active = None

    def _validate(self, frame, time):
        if not isinstance(frame, dict) or frame.get("session_id") != self.session_id:
            return "session_mismatch", None
        sequence = frame.get("sequence")
        if type(sequence) is not int or sequence <= self.last_sequence:
            return "sequence", None
        observation_id = frame.get("observation_id")
        if (
            type(observation_id) is not int
            or not 1 <= observation_id <= self.observation_id
        ):
            return "stale_observation", None
        if self.safety_checks:
            issued = self.observations.get(observation_id)
            if issued is None or not 0 <= time - issued < OBSERVATION_TIMEOUT:
                return "stale_observation", None
        ok, measured = _finite_vector(self.plant.q())
        if not ok:
            return "invalid_command", {"field": "measured"}
        arrays = {}
        for name, bounds in SANITY.items():
            ok, array = _finite_vector(frame.get(name))
            if not ok or (
                self.safety_checks
                and (np.any(array < bounds[0]) or np.any(array > bounds[1]))
            ):
                return "invalid_command", {"field": name}
            arrays[name] = array
        if not self.safety_checks:
            return None, None
        previous = None
        if self.previous is not None:
            previous = {"target": self.previous[0], "velocity": self.previous[1]}
        dt = CONTROL_DT if previous is None else time - self.last_command_time
        try:
            self.rules.check_command(
                arrays["q"],
                arrays["dq"],
                arrays["tau"],
                measured,
                previous,
                dt,
            )
            demand = self.plant.demand(
                arrays["q"],
                arrays["dq"],
                arrays["kp"],
                arrays["kd"],
                arrays["tau"],
                measured,
                self.plant.dq(),
            )
            self.rules.check_torque(demand)
        except JointViolation as exc:
            return exc.reason, exc.details
        except PolicyViolation as exc:
            return exc.args[0], exc.details
        except ValueError as exc:
            return exc.args[0] if exc.args else "invalid_command", None
        return None, None
