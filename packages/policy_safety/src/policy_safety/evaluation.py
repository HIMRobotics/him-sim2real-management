"""Scenario evaluation, measurements, traces and acceptance reports."""

import json
import platform
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime

from . import __version__
from .adapter import ADAPTERS, PolicyViolation
from .bundle import digest, model_files
from .config import METRICS, profile_config, suite_config
from .joint_rules import DEFAULT_JOINT_PROFILE, JointRules
from .simulation import Simulation


def write_report(path, report):
    temporary = Path(path).with_suffix(".tmp")
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def verdict(scenarios, calibrated):
    if any(s["status"] in ("error", "incomplete") for s in scenarios):
        return "incomplete", 2
    if any(s["status"] == "violation" for s in scenarios):
        return "violations", 1
    if not scenarios:
        return "incomplete", 2
    return ("within_limits", 0) if calibrated else ("uncalibrated", 3)


class Monitor:
    def __init__(self, simulation, profile):
        self.sim = simulation
        self.profile = profile
        self.previous_velocity = simulation.data.qvel[simulation.vadr].copy()
        self.peaks = {}

    def sample(self, target, applied, target_rate, upright):
        sim, d, m = self.sim, self.sim.data, self.sim.model
        q, dq = d.qpos[sim.qadr], d.qvel[sim.vadr]
        acceleration = (dq - self.previous_velocity) / sim.physics_dt
        self.previous_velocity = dq.copy()
        torque = d.qfrc_actuator[sim.vadr].copy()
        contact_force = 0.0
        for i in range(d.ncon):
            wrench = np.zeros(6)
            mujoco.mj_contactForce(m, d, i, wrench)
            if not np.isfinite(wrench).all():
                contact_force = float("nan")
                break
            contact_force = max(contact_force, float(np.linalg.norm(wrench[:3])))
        metrics = {
            "joint_velocity": float(np.max(np.abs(dq))),
            "joint_acceleration": float(np.max(np.abs(acceleration))),
            "actuator_torque": float(np.max(np.abs(torque))),
            "tracking_error": float(np.max(np.abs(applied - q))),
            "target_rate": float(np.max(np.abs(target_rate))),
            "base_linear_speed": float(np.linalg.norm(d.qvel[:3])),
            "base_angular_speed": float(np.linalg.norm(d.qvel[3:6])),
            "contact_force": contact_force,
            "clipping": float(np.max(np.abs(target - applied))),
        }
        reason = None
        if not all(
            np.isfinite(value).all()
            for value in (d.qpos, d.qvel, acceleration, torque, list(metrics.values()))
        ):
            reason = "nonfinite_simulation"
        else:
            for name, value in metrics.items():
                self.peaks[name] = max(self.peaks.get(name, 0), value)
                if (
                    name in self.profile["maxima"]
                    and value > self.profile["maxima"][name]
                ):
                    reason = reason or f"{name}_limit"
            tolerance = self.profile["joint_position_tolerance_rad"]
            if np.any(q < sim.bounds[:, 0] - tolerance) or np.any(
                q > sim.bounds[:, 1] + tolerance
            ):
                reason = reason or "joint_position_limit"
            if np.any(torque < sim.torque_bounds[:, 0] - 1e-5) or np.any(
                torque > sim.torque_bounds[:, 1] + 1e-5
            ):
                reason = reason or "model_torque_limit"
            base_upright = d.xmat[sim.base].reshape(3, 3)[2, 2]
            if upright and (
                d.xpos[sim.base, 2] < self.profile["fall_height_m"]
                or base_upright < self.profile["fall_upright_min"]
            ):
                reason = reason or "fall"
        return metrics, acceleration, torque, reason


def run_scenario(
    sim, adapter, profile, scenario, trace_path, checkpoint_steps=(), joint_rules=None
):
    command = scenario["command"]
    monitor = Monitor(sim, profile)
    joint_rules = joint_rules or JointRules(sim.robot["joint_names"])
    start = float(sim.data.time)
    start_position = sim.data.qpos[:3].copy()
    previous_target = sim.data.ctrl[sim.actuators].copy()
    snapshots, rows = {}, []
    result = dict(scenario, status="completed", first_violation=None, peaks={})
    steps = round(scenario["seconds"] / sim.dt)
    if steps < 1 or not np.isclose(
        steps * sim.dt, scenario["seconds"], atol=1e-10, rtol=0
    ):
        raise ValueError(
            "scenario duration must be a positive multiple of control timestep"
        )
    state = np.empty(mujoco.mj_stateSize(sim.model, sim.state_spec))
    initial = sim.snapshot(adapter, command)
    target, applied, velocity, torque_command = (np.empty(0) for _ in range(4))
    measured_at_command = np.empty(0)
    try:
        for step in range(steps):
            obs = sim.observation(adapter, command)
            raw = adapter.infer(obs)
            target = adapter.decode(raw)
            applied = np.clip(target, sim.ctrl_bounds[:, 0], sim.ctrl_bounds[:, 1])
            velocity, torque_command = adapter.command_velocity_and_torque()
            measured_at_command = sim.data.qpos[sim.qadr].copy()
            target_rate = (target - previous_target) / sim.dt
            previous_target = target.copy()
            target_invalid = bool(
                np.any(target < sim.ctrl_bounds[:, 0])
                or np.any(target > sim.ctrl_bounds[:, 1])
            )
            monitor.peaks["clipping"] = max(
                monitor.peaks.get("clipping", 0),
                float(np.max(np.abs(target - applied))),
            )
            if target_invalid:
                raise PolicyViolation("decoded_target_out_of_range")
            joint_rules.check_command(
                target,
                velocity,
                torque_command,
                measured_at_command,
                sim.command_history,
                sim.dt,
            )
            sim.data.ctrl[sim.actuators] = applied
            sim.command_history = {"target": target.copy(), "velocity": velocity.copy()}
            for substep in range(sim.substeps):
                elapsed = (step * sim.substeps + substep) * sim.physics_dt
                sim.apply_push(scenario.get("push"), elapsed)
                warning_count = sim.data.warning.number.copy()
                mujoco.mj_step(sim.model, sim.data)
                metrics, acceleration, torque, reason = monitor.sample(
                    target, applied, target_rate, scenario["upright"]
                )
                if np.any(sim.data.warning.number > warning_count):
                    reason = "simulator_warning"
                mujoco.mj_getState(sim.model, sim.data, state, sim.state_spec)
                rows.append(
                    (
                        float(sim.data.time),
                        state.copy(),
                        obs[0].copy(),
                        raw.copy(),
                        target.copy(),
                        applied.copy(),
                        sim.data.qpos[sim.qadr].copy(),
                        sim.data.qvel[sim.vadr].copy(),
                        acceleration.copy(),
                        torque.copy(),
                        sim.data.qpos[:7].copy(),
                        sim.data.qvel[:6].copy(),
                        np.array(list(metrics.values())),
                        sim.data.xfrc_applied.copy(),
                    )
                )
                if reason in ("nonfinite_simulation", "simulator_warning"):
                    raise RuntimeError(reason)
                if reason:
                    raise PolicyViolation(reason)
                joint_rules.check_tracking(target, sim.data.qpos[sim.qadr])
            if step + 1 in checkpoint_steps:
                snapshots[step + 1] = sim.snapshot(adapter, command)
    except PolicyViolation as exc:
        result["status"] = "violation"
        result["first_violation"] = {
            "reason": str(exc),
            "time_seconds": float(sim.data.time - start),
            **exc.details,
        }
    except Exception as exc:  # noqa: BLE001 - Preserve diagnostics from inference backends.
        result["status"] = "error"
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        sim.data.xfrc_applied[:] = 0
    result["peaks"] = monitor.peaks
    result["simulated_seconds"] = max(0.0, float(sim.data.time - start))
    displacement = sim.data.qpos[:3] - start_position
    result["progress"] = {
        "displacement_m": displacement.tolist()
        if np.isfinite(displacement).all()
        else None
    }
    columns = (
        "time",
        "integration_state",
        "observation",
        "raw_action",
        "decoded_target",
        "applied_target",
        "joint_position",
        "joint_velocity",
        "joint_acceleration",
        "actuator_torque",
        "base_pose",
        "base_velocity",
        "metrics",
        "external_wrench",
    )
    arrays = {
        name: np.array([row[i] for row in rows]) for i, name in enumerate(columns)
    }
    np.savez_compressed(
        trace_path,
        **arrays,
        initial_integration_state=initial["physics"],
        initial_policy_state=initial["policy"],
        initial_previous_target=np.empty(0)
        if initial["command_history"] is None
        else initial["command_history"]["target"],
        initial_previous_target_velocity=np.empty(0)
        if initial["command_history"] is None
        else initial["command_history"]["velocity"],
        last_attempted_target=target,
        last_attempted_clipped_target=applied,
        last_attempted_target_velocity=velocity,
        last_attempted_feedforward_torque=torque_command,
        last_measured_position_at_command=measured_at_command,
        last_raw_output=getattr(adapter, "last_raw", np.empty(0)),
        command=initial["command"],
        initial_rng_json=json.dumps(initial["rng"]),
        state_spec=int(sim.state_spec),
        metric_names=np.array(list(monitor.peaks) if not rows else list(metrics)),
    )
    result["trace"] = str(Path("traces") / trace_path.name)
    return result, snapshots


def evaluate(request):
    output = Path(request["output"])
    report_path = output / "report.json"
    sim = Simulation(request["robot_config"])
    suite = suite_config(request.get("suite"))
    profile = profile_config(request["profile"])
    joint_rules = JointRules(
        sim.robot["joint_names"], request.get("joint_profile", DEFAULT_JOINT_PROFILE)
    )
    for name in ("nominal_seconds", "branch_seconds", "counterfactual_seconds"):
        steps = round(suite[name] / sim.dt)
        if steps < 1 or not np.isclose(steps * sim.dt, suite[name], atol=1e-10, rtol=0):
            raise ValueError(f"{name} must be a positive multiple of control timestep")
    if suite["perturbations"]:
        for name in ("push_start_seconds", "push_seconds"):
            ticks = round(suite[name] / sim.physics_dt)
            if not np.isclose(ticks * sim.physics_dt, suite[name], atol=1e-10, rtol=0):
                raise ValueError(f"{name} must be a multiple of physics timestep")
    files = model_files(request["model"])
    robot_root = sim.path.parent
    assets = [(robot_root / f).resolve() for f in sim.robot["assets"]]
    if any(not p.is_relative_to(robot_root) or not p.is_file() for p in assets):
        raise ValueError(
            "robot asset manifest requires existing files within the bundle"
        )
    # Require every XML and mesh in the bundle to be fingerprinted.
    required = {
        p.resolve()
        for p in robot_root.rglob("*")
        if p.suffix.lower() in (".xml", ".stl", ".obj", ".msh", ".png", ".jpg")
    }
    if required - set(assets):
        raise ValueError("robot assets missing from manifest")
    adapter = ADAPTERS[request["adapter"]](request["model"], sim.robot)
    for body in suite["push_bodies"] if suite["perturbations"] else []:
        sim.model.body(body)
    package_files = sorted(Path(__file__).parent.glob("*.py"))
    provenance = {
        "model": {
            str(p.relative_to(Path(request["model"]).resolve().parent)): digest(p)
            for p in files
        },
        "robot_assets": {str(p.relative_to(robot_root)): digest(p) for p in assets},
        "robot_config": digest(sim.path),
        "profile": digest(request["profile"]),
        "joint_profile": digest(joint_rules.path),
        "implementation": {p.name: digest(p) for p in package_files},
    }
    report = {
        "version": 1,
        "status": "running",
        "exit_code": 2,
        "adapter": request["adapter"],
        "tested_behavior": adapter.tested_behavior,
        "hardware_authorization": False,
        "profile": profile,
        "joint_movement_checks": joint_rules.coverage(),
        "acceptance_checks": {
            "calibrated": profile["calibrated"],
            "metric_threshold_scope": "Global scalar limits. Per-joint command-delta and tracking limits are listed in joint_movement_checks.",
            "metric_maxima": profile["maxima"],
            "metrics_without_thresholds": sorted(
                METRICS - set(profile["maxima"]) - {"actuator_torque", "clipping"}
            ),
            "always_checked": [
                "finite_policy_output",
                "decoded_target_range",
                "joint_position_range",
                "model_torque_range",
            ],
            "upright_scenarios_only": ["fall_height", "fall_orientation"],
            "note": "MuJoCo clamps actuator force to model limits. This does not test unsaturated torque demand. Out-of-range decoded targets are rejected before clipping can hide them.",
        },
        "suite": suite,
        "robot": sim.robot,
        "hashes": provenance,
        "versions": {
            "package": __version__,
            "python": platform.python_version(),
            "mujoco": mujoco.__version__,
            "onnxruntime": onnxruntime.__version__,
            "numpy": np.__version__,
        },
        "scenarios": [],
    }
    if request.get("suite"):
        provenance["suite"] = digest(request["suite"])
    (output / "traces").mkdir()

    def save():
        scenarios = report["scenarios"]
        report["coverage"] = {
            "planned": planned,
            "reported": len(scenarios),
            **{
                status: sum(s["status"] == status for s in scenarios)
                for status in ("completed", "violation", "error", "incomplete")
            },
        }
        write_report(report_path, report)

    pushes = [None]
    if suite["perturbations"]:
        for body in suite["push_bodies"]:
            for axis in (0, 1):
                for sign in (-1, 1):
                    force = [0.0, 0.0, 0.0]
                    force[axis] = sign * suite["push_force_n"]
                    pushes.append(
                        {
                            "body": body,
                            "force": force,
                            "start": suite["push_start_seconds"],
                            "duration": suite["push_seconds"],
                        }
                    )
    planned = len(suite["seeds"]) * (
        len(suite["commands"]) * (1 + len(suite["checkpoints"]) * len(pushes))
        + len(suite["poses"])
    )
    save()

    def execute(scenario, checkpoints=()):
        trace = output / "traces" / f"{len(report['scenarios']):05d}.npz"
        report["active_scenario"] = scenario
        save()
        try:
            result, snapshots = run_scenario(
                sim, adapter, profile, scenario, trace, checkpoints, joint_rules
            )
        except Exception as exc:  # noqa: BLE001 - Report each failed scenario independently.
            result, snapshots = (
                dict(scenario, status="error", error=f"{type(exc).__name__}: {exc}"),
                {},
            )
        report["scenarios"].append(result)
        report["active_scenario"] = None
        save()
        return snapshots

    for seed in suite["seeds"]:
        for name, command in suite["commands"].items():
            scenario = {
                "name": name,
                "kind": "nominal",
                "seed": seed,
                "command": command,
                "seconds": suite["nominal_seconds"],
                "upright": True,
            }
            steps = [
                round(suite["nominal_seconds"] * fraction / sim.dt)
                for fraction in suite["checkpoints"]
            ]
            if len(set(steps)) != len(steps) or any(
                s < 1 or s >= round(suite["nominal_seconds"] / sim.dt) for s in steps
            ):
                raise ValueError(
                    "checkpoint times must resolve to distinct interior control steps"
                )
            sim.reset(seed)
            adapter.reset()
            snapshots = execute(scenario, steps)
            for step in steps:
                for push in pushes:
                    branch = dict(
                        scenario,
                        kind="perturbation" if push else "continuation",
                        seconds=suite["branch_seconds"],
                        checkpoint_step=step,
                        push=push,
                    )
                    if step not in snapshots:
                        report["scenarios"].append(
                            dict(
                                branch,
                                status="incomplete",
                                error="nominal_checkpoint_unavailable",
                            )
                        )
                        save()
                        continue
                    sim.restore(snapshots[step], adapter)
                    execute(branch)
        for pose in suite["poses"]:
            scenario = {
                "name": pose,
                "kind": "counterfactual",
                "seed": seed,
                "command": [0, 0, 0],
                "seconds": suite["counterfactual_seconds"],
                "upright": pose == "standing",
            }
            try:
                sim.reset(seed, pose)
                adapter.reset()
            except Exception as exc:  # noqa: BLE001 - Invalid poses are explicit coverage failures.
                report["scenarios"].append(
                    dict(scenario, status="error", error=f"initialization: {exc}")
                )
                save()
                continue
            execute(scenario)
    report["status"], report["exit_code"] = verdict(
        report["scenarios"],
        profile["calibrated"] and not joint_rules.coverage()["uncovered_joints"],
    )
    save()
    return report["exit_code"]
