"""Scenario evaluation, measurements, traces and acceptance reports."""

import json
import platform
from pathlib import Path

import joint_checks
import mujoco
import numpy as np
import onnxruntime
from joint_checks import (
    BASE_ANGULAR_SPEED_LIMIT_RAD_S,
    JointViolation,
    check_base_motion,
    torque_demand,
)

from . import __version__
from .adapter import ADAPTERS, PolicyViolation
from .bundle import digest, model_files
from .config import (
    ISSUE_COLUMNS,
    METRICS,
    SUMMARY_COLUMNS,
    atomic_report,
    plan_scenarios,
    profile_config,
    suite_config,
)
from .joint_rules import DEFAULT_JOINT_PROFILE, JointRules
from .simulation import Simulation, mesh_point_in_body


def raise_violation(reason, details=None):
    raise PolicyViolation(reason, details)


def verdict(scenarios, calibrated):
    if any(s["status"] in ("error", "incomplete") for s in scenarios):
        return "incomplete", 2
    if any(s["status"] == "violation" for s in scenarios):
        return "violations", 1
    if not scenarios:
        return "incomplete", 2
    return ("within_limits", 0) if calibrated else ("uncalibrated", 3)


# Other limit reasons already end with _limit.
LIMIT_REASONS = {
    "nonzero_feedforward_torque",
    "decoded_target_out_of_range",
}


def scenario_label(scenario):
    """Group a run with its gait, shove strength, torque scale, or pose."""
    name = scenario["name"]
    scale = scenario.get("torque_scale")
    if scale not in (None, 1):
        name = f"{name} {round(scale * 100)}%"
    if scenario["kind"] == "perturbation":
        level = (scenario.get("push") or {}).get("level")
        if level:
            return f"{name} push {level}"
        return f"{name} push"
    return name


def issue_flags(scenario):
    """Every class this run hit. A limit and a miss can both be present."""
    status = scenario["status"]
    if status == "incomplete":
        return {"skipped"}
    flags = set()
    if status == "error":
        flags.add("errors")
    if scenario.get("violent_motion"):
        flags.add("flailing")
    recorded = scenario.get("violations") or [scenario.get("first_violation") or {}]
    for entry in recorded:
        reason = entry.get("reason", "")
        if reason == "joint_tracking_error_limit":
            flags.add("misses")
        elif reason == "fall":
            flags.add("falls")
        elif reason in LIMIT_REASONS or reason.endswith("_limit"):
            flags.add("limits")
        elif reason:
            flags.add("other")
    return flags


def problem_class(scenario):
    """Caption text for every class a run hit, in table order."""
    flags = issue_flags(scenario)
    return "+".join(name for name in ISSUE_COLUMNS if name in flags) or "completed"


def summarize_scenarios(scenarios):
    """Count each class on its own. One run can increment several columns."""
    buckets = {}
    for scenario in scenarios:
        label = scenario_label(scenario)
        if label not in buckets:
            buckets[label] = dict.fromkeys(("total", *SUMMARY_COLUMNS), 0)
        bucket = buckets[label]
        bucket["total"] += 1
        flags = issue_flags(scenario)
        for kind in flags:
            bucket[kind] += 1
        bucket["problems"] += bool(flags - {"misses", "skipped"})
        bucket["warnings"] += "misses" in flags
    return [{"scenario": label, **counts} for label, counts in buckets.items()]


class Monitor:
    def __init__(self, simulation, profile):
        self.sim = simulation
        self.profile = profile
        self.previous_velocity = simulation.data.qvel[simulation.vadr].copy()
        self.peaks = {}

    def sample(self, target, applied, upright):
        sim, d, m = self.sim, self.sim.data, self.sim.model
        q, dq = d.qpos[sim.qadr], d.qvel[sim.vadr]
        acceleration = (dq - self.previous_velocity) / sim.physics_dt
        self.previous_velocity = dq
        torque = d.qfrc_actuator[sim.vadr]
        contact_force = 0.0
        wrench = np.zeros(6)
        for i in range(d.ncon):
            mujoco.mj_contactForce(m, d, i, wrench)
            if not np.isfinite(wrench).all():
                contact_force = float("nan")
                break
            contact_force = max(contact_force, float(np.linalg.norm(wrench[:3])))
        metrics = {
            "joint_velocity": float(np.max(np.abs(dq))),
            "joint_acceleration": float(np.max(np.abs(acceleration))),
            "actuator_torque": float(np.max(np.abs(torque))),
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
    sim,
    adapter,
    profile,
    scenario,
    trace_path,
    checkpoint_steps=(),
    joint_rules=None,
    *,
    continue_after_violation=False,
):
    command = scenario["command"]
    monitor = Monitor(sim, profile)
    joint_rules = joint_rules or JointRules(sim.robot["joint_names"])
    start = float(sim.data.time)
    start_position = sim.data.qpos[:3].copy()
    snapshots, rows = {}, []
    result = dict(scenario, status="completed", first_violation=None, peaks={}, warnings={})
    steps = round(scenario["seconds"] / sim.dt)
    if steps < 1 or not np.isclose(
        steps * sim.dt, scenario["seconds"], atol=1e-10, rtol=0
    ):
        raise ValueError(
            "scenario duration must be a positive multiple of control timestep"
        )
    state = np.empty(mujoco.mj_stateSize(sim.model, sim.state_spec))
    speed_caps = np.full(sim.vadr.size, np.inf)
    for index, _name, limits in joint_rules.entries:
        speed_caps[index] = limits.max_velocity_rad_secs
    sustained_flail = False
    initial = sim.snapshot(adapter, command)
    target, applied, velocity, torque_command = (np.empty(0) for _ in range(4))
    measured_at_command = np.empty(0)
    continue_after_violation = continue_after_violation or bool(checkpoint_steps)

    def record_violation(exc):
        entry = {
            "reason": getattr(exc, "reason", str(exc)),
            "time_seconds": float(sim.data.time - start),
            **exc.details,
        }
        violations = result.setdefault("violations", [])
        if not any(item["reason"] == entry["reason"] for item in violations):
            violations.append(entry)
        if result["first_violation"] is not None:
            return
        result["status"] = "violation"
        result["first_violation"] = entry

    def screen(fn, *args, **kwargs):
        """Keep a checkpoint run alive after the first failed command."""
        try:
            fn(*args, **kwargs)
        except (PolicyViolation, JointViolation) as exc:
            if not continue_after_violation:
                raise
            record_violation(exc)

    try:
        sim.set_torque_scale(scenario.get("torque_scale", 1.0))
        kp = sim.model.actuator_gainprm[sim.actuators, 0]
        kd = -sim.model.actuator_biasprm[sim.actuators, 2]
        if sim.command_history is None:
            # Validate the initial hold before treating policy outputs as motion.
            hold = sim.data.ctrl[sim.actuators].copy()
            zero = np.zeros(len(hold))
            screen(
                joint_rules.check_command,
                hold,
                zero,
                zero,
                sim.data.qpos[sim.qadr],
                initial=True,
            )
            sim.command_history = {"target": hold, "velocity": zero}
        for step in range(steps):
            obs = sim.observation(adapter, command)
            raw = adapter.infer(obs)
            target = adapter.decode(raw)
            applied = joint_rules.clip_positions(target, result["warnings"], sim.ctrl_bounds)
            velocity, torque_command = adapter.command_velocity_and_torque()
            measured_at_command = sim.data.qpos[sim.qadr]
            commanded = applied
            monitor.peaks["clipping"] = max(
                monitor.peaks.get("clipping", 0),
                float(np.max(np.abs(target - applied))),
            )
            screen(
                joint_rules.check_command,
                commanded,
                velocity,
                torque_command,
                measured_at_command,
                initial=False,
                warnings=result["warnings"],
            )
            sim.data.ctrl[sim.actuators] = applied
            sim.command_history = {
                "target": commanded.copy(),
                "velocity": velocity.copy(),
            }
            policy_values = tuple(
                value.copy() for value in (obs[0], raw, target, applied)
            )
            for substep in range(sim.substeps):
                elapsed = (step * sim.substeps + substep) * sim.physics_dt
                sim.apply_push(scenario.get("push"), elapsed)
                warning_count = sim.data.warning.number.copy()
                screen(
                    joint_rules.check_torque,
                    torque_demand(
                        commanded, 0, kp, kd,
                        sim.model.actuator_biasprm[sim.actuators, 0],
                        sim.data.qpos[sim.qadr], sim.data.qvel[sim.vadr],
                    ),
                )
                mujoco.mj_step(sim.model, sim.data)
                metrics, acceleration, torque, reason = monitor.sample(
                    target, applied, scenario["upright"]
                )
                if np.any(sim.data.warning.number > warning_count):
                    reason = "simulator_warning"
                mujoco.mj_getState(sim.model, sim.data, state, sim.state_spec)
                rows.append(
                    (
                        float(sim.data.time),
                        state.copy(),
                        *policy_values,
                        sim.data.qpos[sim.qadr],
                        sim.data.qvel[sim.vadr],
                        acceleration,
                        torque,
                        sim.data.qpos[:7].copy(),
                        sim.data.qvel[:6].copy(),
                        np.array(list(metrics.values())),
                        sim.data.xfrc_applied.copy(),
                    )
                )
                if reason in ("nonfinite_simulation", "simulator_warning"):
                    raise RuntimeError(reason)
                if reason:
                    screen(raise_violation, reason)
                measured_dq = sim.data.qvel[sim.vadr]
                if np.any(np.abs(measured_dq) > speed_caps) or np.any(
                    np.abs(sim.data.qvel[3:6]) > BASE_ANGULAR_SPEED_LIMIT_RAD_S
                ):
                    sustained_flail = True
                screen(joint_rules.check_measured_velocity, measured_dq)
                screen(check_base_motion, sim.data.qvel[3:6])
                screen(joint_rules.check_torque, torque)
            response_error = sim.data.qpos[sim.qadr] - applied
            monitor.peaks["response_error"] = max(monitor.peaks.get("response_error", 0), float(np.max(np.abs(response_error))))
            if step + 1 in checkpoint_steps:
                snapshots[step + 1] = sim.snapshot(adapter, command)
    except (PolicyViolation, JointViolation) as exc:
        record_violation(exc)
    except Exception as exc:  # noqa: BLE001 - Preserve diagnostics from inference backends.
        result["status"] = "error"
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        sim.data.xfrc_applied[:] = 0
        sim.set_torque_scale(1.0)
    result["violent_motion"] = sustained_flail
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


def load_session(request):
    """Validate inputs for both screening and rendering before inference."""
    sim = Simulation(request["robot_config"])
    profile = profile_config(request["profile"])
    rules = JointRules(
        sim.robot["joint_names"], request.get("joint_profile", DEFAULT_JOINT_PROFILE)
    )
    files = model_files(request["model"])
    if request["adapter"] == "nvidia-decoupled-wbc-516-v1":
        files += model_files(Path(request["model"]).with_name("balance.onnx"))
    adapter = ADAPTERS[request["adapter"]](request["model"], sim.robot)
    return sim, adapter, profile, rules, files


def evaluate(request):
    output = Path(request["output"])
    report_path = output / "report.json"
    suite = suite_config(request.get("suite"))
    plan = plan_scenarios(suite)
    planned = plan["baseline_count"] + len(plan["pushes"])
    sim, adapter, profile, joint_rules, files = load_session(request)
    for name in ("nominal_seconds", "branch_seconds", "counterfactual_seconds"):
        steps = round(suite[name] / sim.dt)
        if steps < 1 or not np.isclose(steps * sim.dt, suite[name], atol=1e-10, rtol=0):
            raise ValueError(f"{name} must be a positive multiple of control timestep")
    if suite["perturbations"]:
        for name in ("push_start_seconds", "push_seconds"):
            ticks = round(suite[name] / sim.physics_dt)
            if not np.isclose(ticks * sim.physics_dt, suite[name], atol=1e-10, rtol=0):
                raise ValueError(f"{name} must be a multiple of physics timestep")
    robot_root = sim.path.parent
    push_targets = [{"body": body} for body in suite["push_bodies"]]
    if suite["perturbations"]:
        for target in push_targets:
            sim.model.body(target["body"])
        for spec in suite["push_points"]:
            sim.model.body(spec["body"])
            push_targets.append(
                {
                    "body": spec["body"],
                    "label": spec["name"],
                    "point": mesh_point_in_body(
                        sim.model, spec["body"], spec["mesh"]
                    ).tolist(),
                }
            )
    package_files = sorted(Path(__file__).parent.glob("*.py"))
    package_files.append(Path(joint_checks.__file__))
    provenance = {
        "model": {
            str(p.relative_to(Path(request["model"]).resolve().parent)): digest(p)
            for p in files
        },
        "robot_assets": {
            str(p.relative_to(robot_root)): digest(p) for p in sorted(sim.assets)
        },
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
        "clip_to_stops": True,
        "profile": profile,
        "joint_movement_checks": joint_rules.coverage(),
        "acceptance_checks": {
            "calibrated": profile["calibrated"],
            "metric_threshold_scope": "Global scalar limits. Per-joint physical limits and startup checks are listed in joint_movement_checks.",
            "metric_maxima": profile["maxima"],
            "metrics_without_thresholds": sorted(
                METRICS - set(profile["maxima"]) - {"actuator_torque", "clipping"}
            ),
            "always_checked": [
                "finite_policy_output",
                "position_clipping_with_warning",
                "joint_position_range",
                "model_torque_range",
                "profile_torque_demand_and_applied_bounds",
            ],
            "upright_scenarios_only": ["fall_height", "fall_orientation"],
            "note": "Positions are clipped to the intersection of joint-profile and model bounds with per-joint warnings. PD torque checks use the clipped command. Response error uses the command actually applied. Runtime command gaps do not reject commands.",
        },
        "suite": suite,
        "push_sampling": {
            "method": "site_strength_stratified_without_replacement",
            "seed": suite["sample_seed"],
            "available": plan["push_candidates"],
            "selected": len(plan["pushes"]),
        },
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
        report["scenario_summary"] = summarize_scenarios(scenarios)
        atomic_report(report_path, report)

    targets_by_name = {}
    for target in push_targets:
        targets_by_name[target.get("label", target["body"])] = target

    pushes_by_checkpoint = {}
    for choice in plan["pushes"]:
        target = targets_by_name[choice["target"]]
        force = [0.0, 0.0, 0.0]
        force[choice["axis"]] = choice["sign"] * choice["force_n"]
        push = {
            **target,
            "level": choice["level"],
            "force": force,
            "start": suite["push_start_seconds"],
            "duration": suite["push_seconds"],
        }
        key = (choice["seed"], choice["command"], choice["checkpoint"])
        pushes_by_checkpoint.setdefault(key, []).append(push)
    save()

    def execute(scenario, checkpoints=()):
        trace = output / "traces" / f"{len(report['scenarios']):05d}.npz"
        report["active_scenario"] = scenario
        save()
        try:
            if "checkpoint_step" not in scenario:
                pose = (
                    scenario["name"]
                    if scenario["kind"] == "counterfactual"
                    else "standing"
                )
                sim.reset(scenario["seed"], pose)
                adapter.reset()
            result, snapshots = run_scenario(
                sim,
                adapter,
                profile,
                scenario,
                trace,
                checkpoints,
                joint_rules,
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
            snapshots = execute(scenario, steps)
            for fraction, step in zip(suite["checkpoints"], steps):
                pushes = pushes_by_checkpoint.get((seed, name, fraction), [])
                for push in [None, *pushes]:
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
                                status="error",
                                error="nominal_checkpoint_unavailable",
                            )
                        )
                        save()
                        continue
                    sim.restore(snapshots[step], adapter)
                    execute(branch)
            for scale in suite["torque_scales"]:
                execute(dict(scenario, torque_scale=scale))
        for pose in suite["poses"]:
            scenario = {
                "name": pose,
                "kind": "counterfactual",
                "seed": seed,
                "command": [0, 0, 0],
                "seconds": suite["counterfactual_seconds"],
                "upright": pose == "standing",
            }
            execute(scenario)
    report["status"], report["exit_code"] = verdict(
        report["scenarios"],
        profile["calibrated"] and not joint_rules.coverage()["uncovered_joints"],
    )
    save()
    return report["exit_code"]
