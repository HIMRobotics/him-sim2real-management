"""Accept container joint frames, check them, and step the MuJoCo plant."""

import json
import os
import sys
import time
import uuid
from copy import deepcopy
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mujoco
import numpy as np
from hardware import SimHardware
from joint_checks import (
    TORQUE_TARGET_FRACTION,
    JointViolation,
    check_base_motion,
    checked_targets,
)
from metrics import MotionMetrics
from plant import PHYSICS_DT, POSES, Plant
from policy_safety.joint_rules import JointRules

from robot_safety.config import COMMAND_FIELDS, Config, ThrashConfig
from robot_safety.core import action_error, watchdog_reason
from robot_safety.startup_balance import StartupBalance
from robot_safety.thrash import ThrashDetector

# Stands in for the operator, who launches the policy with left + B once the
# robot has settled in startup balance. Balance is still swaying at 2 s.
# Runs add this before the requested duration.
LAUNCH_SECONDS = 5.0


class Runner:
    """One simulated session. A failed check latches for the rest of the run."""

    def __init__(self, pose=None, scene=None, safety_checks=True, *, record_trace=None,
                 thrash_config=None, config=None, control_path="submission"):
        pose = pose or os.environ.get("G1_SIM_POSE", "standing")
        if pose not in POSES:
            raise ValueError("unknown pose")
        self.safety_checks = bool(safety_checks)
        config = config or Config(thrash=ThrashConfig(
            mode=os.environ.get("G1_SIM_THRASH_MODE") or ("monitor" if safety_checks else "off")
        ))
        thrash_config = thrash_config or config.thrash
        self.config = config = replace(config, thrash=thrash_config)
        self.thrash_detector = ThrashDetector(config.thrash, config.joints, config.robot_timeout)
        self.external_thrash = None
        if control_path not in ("submission", "direct"):
            raise ValueError("unknown control path")
        self.control_path = control_path
        self.plant = Plant(pose, scene=scene, check_fall=self.safety_checks)
        if config.joints != len(self.plant.joint_names):
            raise ValueError("simulation config must match the G1 joint count")
        profile = os.environ.get("G1_JOINT_PROFILE")
        self.rules = (JointRules(self.plant.joint_names, profile) if profile
                      else JointRules(self.plant.joint_names))
        self.session_id = "sim-" + uuid.uuid4().hex[:12]
        self.observation_id = 0
        self.observations = {}
        self.last_sequence = -1
        self.last_command_time = None
        self.last_cycle_time = None
        self.last_robot_time = None
        self.command_observation_id = None
        self.active = None
        self.submitted = None
        self.reason = None
        self.detail = None
        self.stop_sim_time = None
        self.commands = 0
        self.steps = 0
        self.sim_time = 0.0
        self.response_error = None
        self.max_response_error = 0.0
        self.position_warnings = {}
        self.trace_qpos = []
        self.trace_time = []
        self.trace_thrash = []
        self.trace_stop = []
        self.trace_startup = []
        self.record_trace = (
            bool(os.environ.get("G1_SIM_TRACE"))
            if record_trace is None
            else record_trace
        )
        self.live_path = os.environ.get("G1_SIM_LIVE")
        self.metrics = MotionMetrics(self.plant)
        self._live = None
        self._circle = None
        self.hardware = (SimHardware(self.plant, self.rules.path)
                         if self.safety_checks and control_path == "submission" else None)
        self.startup = self.hardware.adapter.startup if self.hardware else StartupBalance()
        if not self.hardware:
            self.startup.prepare()
        self._remember()

    @property
    def uses_hardware(self):
        return self.hardware is not None and self.control_path == "submission"

    def arm(self):
        return self.session_id

    def feedback(self, time):
        self._check_deadlines(float(time))
        self.last_robot_time = float(time)
        self.observation_id += 1
        self.observations[self.observation_id] = float(time)
        if len(self.observations) > self.config.observation_history_size:
            oldest = next(iter(self.observations))
            del self.observations[oldest]
        state = self.plant.state()
        state["observation_id"] = self.observation_id
        state["start_button_pressed"] = False
        state["session_id"] = self.session_id
        state.update(self.startup.status())
        if self.reason:
            state["control_ready"] = False
        return state

    def command(self, frame, time):
        if self.reason is not None:
            return self.reason
        frame = dict(frame) if isinstance(frame, dict) else frame
        reason, detail = self._validate(frame, float(time))
        if reason:
            self._latch(reason, detail)
            return reason
        self._check_deadlines(float(time))
        if self.reason is not None:
            return self.reason
        if self.safety_checks:
            try:
                frame.update(self.rules.process_command(
                    frame, self.plant.q(), self.plant.dq(), initial=self.active is None,
                    warnings=self.position_warnings, enforce_torque=False,
                ))
            except JointViolation as exc:
                self._latch(exc.reason, exc.details)
                return self.reason
            except ValueError as exc:
                self._latch(exc.args[0] if exc.args else "invalid_command", None)
                return self.reason
        self.commands += 1
        self.command_observation_id = frame["observation_id"]
        self.last_sequence = int(frame["sequence"])
        self.last_command_time = float(time)

        self.submitted = {
            key: np.asarray(frame[key], dtype=float).copy()
            for key in COMMAND_FIELDS
        }
        self.active = self.submitted
        if self.uses_hardware:
            self.hardware.adapter.submit(self.submitted, self.hardware.adapter.clock()
                                         + min(self.config.action_timeout, 0.2 - 1e-9))
        return None

    def step(self, time):
        self._check_deadlines(float(time))
        if self.last_command_time is not None:
            self.last_cycle_time = float(time)
        if self.uses_hardware and self.commands:
            if not self.hardware.ready:
                self._latch("hardware_arm_rejected", self.hardware.adapter.arm_rejection_reason)
            self.hardware.step(launch=self.sim_time >= LAUNCH_SECONDS
                               and self.hardware.adapter.startup.ready)
            self._sync_hardware()
        elif self.active is None:
            q, dq = self.plant.q(), self.plant.dq()
            q_des, dq_des, kp, kd, tau = self.plant.damping_command(q, dq)
            self.plant._apply_arrays(q_des, dq_des, kp, kd, tau)
        else:
            try:
                self.active = self.startup.command(
                    self.sim_time, self.sim_time, self.plant.state(), self.submitted,
                    automatic=self.sim_time >= LAUNCH_SECONDS)
                self.active["q"] = self.rules.clip_positions(
                    self.active["q"], self.position_warnings)
                if self.safety_checks:
                    entries = ((i, name, replace(limits, kp=float(self.active["kp"][i]),
                                                kd=float(self.active["kd"][i])))
                               for i, name, limits in self.rules.entries)
                    targets = checked_targets(
                        self.active, entries, self.plant.q(), self.plant.dq(),
                        initial=False, initial_velocity=self.rules.profile["initial_velocity_tolerance_rad_secs"],
                        warnings=self.position_warnings, enforce_torque=False,
                        torque_target_fraction=TORQUE_TARGET_FRACTION)
                    self.active = {key: np.array([targets[i][key] for i in range(29)])
                                   for key in COMMAND_FIELDS}
                self.plant.apply(self.active)
            except Exception as exc:  # noqa: BLE001
                self._latch("startup_balance_error", {"detail": str(exc)})
                self.plant._apply_arrays(*self.plant.damping_command(self.plant.q(), self.plant.dq()))
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
            if self.uses_hardware and self.commands:
                self.hardware.receive()
                self._sync_hardware()
            if self.safety_checks:
                if self.plant.fallen() and not self.uses_hardware:
                    violation = ("fall", {"height": self.plant.state()["height"]})
                    break
                self.rules.note_measured_velocity(dq, self.position_warnings)
                try:
                    if not self.uses_hardware:
                        check_base_motion(self.plant.data.qvel[
                            self.plant.base_vadr + 3:self.plant.base_vadr + 6])
                except JointViolation as exc:
                    violation = (exc.reason, exc.details)
                    break
                if self.active is not None:
                    tau_demand = self.plant.demand(
                        self.active["q"], self.active["dq"], self.active["kp"],
                        self.active["kd"], self.active["tau"], self.plant.q(), self.plant.dq())
                    self.rules.note_torque(tau_demand, self.position_warnings, kind="PD demand")
                    self.rules.note_torque(
                        self.plant.data.qfrc_actuator[self.plant.vadr],
                        self.position_warnings, kind="applied torque")
            if self.reason is None and self.commands:
                reason = self.thrash_detector.observe(dq, self.sim_time)
                if reason:
                    violation = (reason, deepcopy(self.thrash_detector.event))
                    break
        self.steps += 1
        if violation is None and self.plant.fallen() and not self.uses_hardware:
            violation = ("fall", {"height": self.plant.state()["height"]})
        if violation is None and self.reason is None and self.active is not None and self.control_path == "submission":
            q_applied = self.active["q"]
            q_measured = self.plant.q()
            self.response_error = (q_measured - q_applied).tolist()
            self.max_response_error = max(self.max_response_error, float(np.max(np.abs(self.response_error))))
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
        thrash_mode = (self.external_thrash or {}).get("mode", self.thrash_detector.config.mode)
        return {
            "hardware_adapter": self.uses_hardware,
            "standing_posture": self.uses_hardware,
            "automatic_recovery": self.uses_hardware,
            "command_protocol": submission,
            "command_limits": submission and self.safety_checks,
            "profile_gains": submission and self.safety_checks,
            "response_error": False,
            "response_error_diagnostic": submission,
            "motor_temperature": False,
            "battery": False,
            "observation_freshness": submission and self.safety_checks,
            "action_timeout": self.safety_checks,
            "robot_state_timeout": submission and self.safety_checks,
            "loop_timeout": self.safety_checks,
            "joint_speed": self.safety_checks,
            "base_angular_speed": self.safety_checks,
            "fall": self.plant.check_fall and not self.uses_hardware,
            "finite_simulation": True,
            "thrash_monitoring": thrash_mode != "off",
            "thrash_enforcement": thrash_mode == "enforce",
        }

    def report(self, lowstate=False):
        if self._live is not None:
            self._live.close()
            self._live = None
        if self.reason:
            status = "violation" if self.safety_checks or self.checks()["thrash_enforcement"] else "error"
        elif self.commands < 1:
            status = "no_command"
        else:
            status = "within_limits" if self.checks()["command_limits"] else "completed"
        trace = self._write_trace()
        return {
            **self.startup.status(),
            "hardware_safety": self.hardware.snapshot() if self.uses_hardware else None,
            "version": 1,
            "status": status,
            "reason": self.reason,
            "detail": self.detail,
            "stop_time_seconds": self.stop_sim_time,
            "stop": self._stop_snapshot(),
            "warnings": self.position_warnings.copy(),
            "response_error_rad": {"last": self.response_error, "max_abs": self.max_response_error},
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
            "fall_check": self.checks()["fall"],
            "trace": trace,
            "motion": self.metrics.report(),
            "thrash": self._thrash_snapshot(),
        }

    def _thrash_snapshot(self):
        snapshot = (deepcopy(self.external_thrash) if self.external_thrash
                    else self.thrash_detector.snapshot())
        snapshot["control_path"] = self.control_path
        return snapshot

    def _stop_snapshot(self):
        if self.reason is None:
            return None
        return {"reason": self.reason, "detail": deepcopy(self.detail),
                "time_secs": self.stop_sim_time}

    def _startup_snapshot(self):
        """Startup balance progress for the viewer. Direct control has none."""
        if self.control_path == "direct":
            return None
        phase = self.hardware.snapshot()["phase"] if self.uses_hardware and self.reason else self.startup.phase
        return {"phase": phase, "elapsed_secs": round(self.sim_time, 3),
                "launch_secs": LAUNCH_SECONDS, "handoff_at_secs": self.startup.handoff,
                "blend_secs": self.startup.blend_seconds}

    def _remember(self):
        telemetry = self._thrash_snapshot() if self.record_trace or self.live_path else None
        stop = self._stop_snapshot()
        startup = self._startup_snapshot()
        if self.record_trace:
            self.trace_qpos.append(self.plant.data.qpos.copy())
            self.trace_time.append(self.sim_time)
            self.trace_thrash.append(telemetry)
            self.trace_stop.append(stop)
            self.trace_startup.append(startup)
        path = self.live_path
        if not path:
            return
        if self._live is None:
            target = Path(path)
            target.parent.mkdir(parents=True, exist_ok=True)
            self._live = target.open("w")
        qpos = [round(float(value), 5) for value in self.plant.data.qpos]
        from view import limit_warning_text

        warning = limit_warning_text(self.position_warnings) or None
        self._live.write(json.dumps({"t": self.sim_time, "qpos": qpos, "thrash": telemetry,
                                    "stop": stop, "warning": warning, "startup": startup}) + "\n")
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
            thrash=json.dumps(self.trace_thrash),
            stop=json.dumps(self.trace_stop),
            startup=json.dumps(self.trace_startup),
        )
        return str(target)

    def circle(self, time):
        """One walking step of a flat command. Limit exceedances warn and do not stop."""
        if self.control_path != "direct":
            self.startup = StartupBalance()
            self.startup.prepare()
        self.control_path = "direct"
        try:
            if self._circle is None:
                from circle import CircleWalker

                self._circle = CircleWalker()
                self._circle.settle(self.plant)
                self._reset_trace()
            if self.reason is None:
                self._circle.started = self.startup.handoff is not None
                self._circle.gait_start = self.startup.handoff
                target = self._circle.targets(self.plant)
                self.active = {
                    "q": np.asarray(target, dtype=float).copy(),
                    "dq": np.zeros(29),
                    "kp": self._circle.kp.copy(),
                    "kd": self._circle.kd.copy(),
                    "tau": np.zeros(29),
                }
                self.submitted = self.active
                self.commands += 1
                self.last_command_time = float(time)

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
        self.trace_thrash = []
        self.trace_stop = []
        self.trace_startup = []
        self.sim_time = 0.0
        self.thrash_detector.reset()
        self._remember()

    def handle(self, request):
        op = request.get("op")
        if op == "arm":
            if self.uses_hardware and not self.hardware.ready:
                self._latch("hardware_arm_rejected", self.hardware.adapter.arm_rejection_reason)
                self.hardware.transport.request_damping()
                return {"ok": False, "reason": self.reason, "detail": self.detail}
            return {"ok": True, "session_id": self.arm(), "launch_seconds": LAUNCH_SECONDS}
        if op == "feedback":
            return {"ok": True, **self.feedback(request["time"])}
        if op == "command":
            reason = self.command(request["frame"], request["time"])
            return {"ok": reason is None, "reason": reason, "detail": self.detail}
        if op in ("circle", "step"):
            self.external_thrash = request.get("thrash", self.external_thrash)
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
            self.external_thrash = request.get("thrash")
            return self.report(request.get("lowstate", False))
        if op == "abort":
            self._latch(request["reason"], request.get("detail"))
            return {"ok": True, "reason": self.reason}
        return {"ok": False, "reason": "unknown_op"}

    def _check_deadlines(self, time):
        if not self.safety_checks or self.reason is not None:
            return
        submission = self.control_path == "submission"
        reason = watchdog_reason(
            self.config, time, last_cycle=self.last_cycle_time,
            action_time=self.last_command_time,
            robot_time=self.last_robot_time if submission else time,
            observation_time=(self.observations.get(self.command_observation_id, -float("inf"))
                              if self.command_observation_id is not None and submission else None),
        )
        if reason:
            self._latch(reason, None)

    def _sync_hardware(self):
        self.startup = self.hardware.adapter.startup
        self.active = self.hardware.transport.active
        self.position_warnings.update(self.hardware.adapter.position_warnings)
        if self.hardware.adapter.abort_reason and self.reason is None:
            self._latch(self.hardware.adapter.abort_reason, None)

    def _latch(self, reason, detail):
        if self.uses_hardware:
            self.hardware.abort(reason)
        first = self.reason is None
        if first:
            self.reason = reason
            self.startup.fail()
            self.detail = detail
            self.stop_sim_time = self.sim_time
        self.active = None
        if first:
            self._remember()

    def _validate(self, frame, time):
        message = frame
        if isinstance(frame, dict):
            message = dict(frame, session=frame.get("session_id"),
                           seq=frame.get("sequence"), state_id=frame.get("observation_id"))
        return action_error(
            message, self.config, self.session_id, self.last_sequence,
            self.observations, self.observation_id, time, safety_checks=self.safety_checks,
        )
