"""Policy contracts, simulation causality and failure reporting."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("onnxruntime")
pytest.importorskip("onnx")


from policy_safety.adapter import G1Walker, inverse_rotate
from policy_safety.cli import run_worker
from policy_safety.config import plan_scenarios, profile_config, scheduled_directions, suite_config
from policy_safety.evaluation import Monitor, run_scenario, summarize_scenarios, verdict
from policy_safety.simulation import Simulation, mesh_point_in_body


class Session:
    def __init__(self, value=0):
        self.value = value

    def get_inputs(self):
        return [
            SimpleNamespace(name="observation", type="tensor(float)", shape=["N", 99])
        ]

    def get_outputs(self):
        return [SimpleNamespace(name="action", type="tensor(float)", shape=["N", 29])]

    def run(self, names, inputs):
        self.observation = inputs["observation"].copy()
        if isinstance(self.value, Exception):
            raise self.value
        return [np.full((1, 29), self.value, dtype=np.float32)]


@pytest.fixture
def setup(bundle):
    sim = Simulation(bundle / "robot.json")
    sim.reset(0)
    adapter = G1Walker(None, sim.robot, Session())
    profile = profile_config(
        Path(__file__).parents[1] / "src/policy_safety/profiles/provisional.json"
    )
    return sim, adapter, profile


def scenario(**changes):
    result = {
        "name": "test",
        "seed": 0,
        "kind": "nominal",
        "command": [0, 0, 0],
        "seconds": 0.04,
        "upright": True,
    }
    result.update(changes)
    return result


def test_torque_scale_changes_motor_gains_and_restores(setup, tmp_path):
    sim, adapter, profile = setup
    kp = sim.model.actuator_gainprm[sim.actuators, 0].copy()
    kd = -sim.model.actuator_biasprm[sim.actuators, 2].copy()
    sim.set_torque_scale(0.95)
    assert np.allclose(sim.model.actuator_gainprm[sim.actuators, 0], kp * 0.95)
    assert np.allclose(-sim.model.actuator_biasprm[sim.actuators, 2], kd * 0.95)
    sim.set_torque_scale(1.05)
    assert np.allclose(sim.model.actuator_gainprm[sim.actuators, 0], kp * 1.05)
    sim.set_torque_scale(1.0)
    assert np.allclose(sim.model.actuator_gainprm[sim.actuators, 0], kp)
    assert np.allclose(-sim.model.actuator_biasprm[sim.actuators, 2], kd)
    adapter.session.value = RuntimeError("inference failed")
    result, _ = run_scenario(
        sim, adapter, profile, scenario(torque_scale=0.95), tmp_path / "failure.npz"
    )
    assert result["status"] == "error"
    np.testing.assert_array_equal(sim.model.actuator_gainprm[sim.actuators, 0], kp)


def test_observation_coordinates_history_and_scaling(setup):
    _sim, adapter, _ = setup
    quat = np.array([np.sqrt(0.5), 0, 0, np.sqrt(0.5)])
    velocity = np.array([1, 0, 0, 0.1, 0.2, 0.3])
    adapter.previous_action[:] = 0.4
    obs = adapter.observation(
        adapter.default + 0.1, np.ones(29), quat, velocity, [0.3, 0, 0]
    )
    np.testing.assert_allclose(obs[0, :3], [0, -1, 0], atol=1e-6)
    np.testing.assert_allclose(obs[0, 3:6], velocity[3:])
    np.testing.assert_allclose(obs[0, 6:9], [0, 0, -1], atol=1e-6)
    np.testing.assert_allclose(obs[0, 9:38], 0.1, atol=1e-6)
    np.testing.assert_allclose(obs[0, 38:67], 1)
    np.testing.assert_allclose(obs[0, 67:96], 0.4)
    np.testing.assert_allclose(obs[0, 96:], [0.3, 0, 0])
    np.testing.assert_allclose(
        adapter.decode(np.ones(29)), adapter.default + adapter.scales
    )
    state = adapter.snapshot()
    adapter.reset()
    adapter.restore(state)
    np.testing.assert_array_equal(adapter.previous_action, state)


def test_sample_neural_path_and_embedded_normalization(bundle):
    import onnx

    sim = Simulation(bundle / "robot.json")
    sim.reset(0)
    adapter = G1Walker(bundle / "walker.onnx", sim.robot)
    config = json.loads((bundle / "source_model_config.json").read_text())
    q, dq, quat, vel = (
        sim.data.qpos[sim.qadr],
        sim.data.qvel[sim.vadr],
        sim.data.qpos[3:7],
        sim.data.qvel[:6],
    )
    # Independent layout matching the supplied policy's neural path.
    expected = np.concatenate(
        [
            inverse_rotate(quat, vel[:3]),
            vel[3:6],
            inverse_rotate(quat, [0, 0, -1]),
            q.astype(np.float32)
            - np.array(
                [config["default_joint_pos"][n] for n in config["joint_names"]],
                dtype=np.float32,
            ),
            dq,
            np.zeros(29),
            [0.3, 0, 0],
        ]
    ).astype(np.float32)[None]
    obs = sim.observation(adapter, [0.3, 0, 0])
    np.testing.assert_array_equal(obs, expected)
    raw = adapter.session.run(None, {adapter.input_name: expected})[0][0]
    np.testing.assert_array_equal(adapter.infer(obs), raw)
    graph = onnx.load(str(bundle / "walker.onnx"), load_external_data=False)
    assert [node.op_type for node in graph.graph.node[:2]] == ["Sub", "Div"]
    assert sim.dt == 0.02 and sim.physics_dt == 0.002
    assert list(sim.model.actuator_trnid[sim.actuators, 0]) == list(sim.joints)


def test_checkpoint_replays_identically(setup, tmp_path):
    sim, adapter, profile = setup
    _, snapshots = run_scenario(
        sim, adapter, profile, scenario(seconds=0.08), tmp_path / "first.npz", [2]
    )
    assert 2 in snapshots
    sim.restore(snapshots[2], adapter)
    expected_random = sim.rng.random()
    first, _ = run_scenario(sim, adapter, profile, scenario(), tmp_path / "a.npz")
    sim.restore(snapshots[2], adapter)
    assert sim.rng.random() == expected_random
    second, _ = run_scenario(sim, adapter, profile, scenario(), tmp_path / "b.npz")
    assert first["status"] == second["status"]
    with np.load(tmp_path / "a.npz") as a, np.load(tmp_path / "b.npz") as b:
        np.testing.assert_allclose(
            a["integration_state"], b["integration_state"], atol=1e-12, rtol=0
        )


def test_push_integral_and_clearing(setup, tmp_path):
    sim, adapter, profile = setup
    push = {"body": "pelvis", "force": [10, 0, 0], "start": 0.02, "duration": 0.02}
    result, _ = run_scenario(
        sim, adapter, profile, scenario(seconds=0.08, push=push), tmp_path / "push.npz"
    )
    assert result["status"] == "completed"
    with np.load(tmp_path / "push.npz") as trace:
        force = trace["external_wrench"][:, sim.base, 0]
        assert np.count_nonzero(force) == 10
        assert np.sum(force) * sim.physics_dt == pytest.approx(0.2)
        assert force[-1] == 0
    assert not sim.data.xfrc_applied.any()


def test_leg_arm_and_head_pushes(setup):
    sim, _, _ = setup
    suite = suite_config()
    for body in (
        "left_hip_yaw_link",
        "left_knee_link",
        "left_ankle_roll_link",
        "right_hip_yaw_link",
        "right_knee_link",
        "right_ankle_roll_link",
        "left_shoulder_roll_link",
        "left_elbow_link",
        "left_wrist_yaw_link",
        "right_shoulder_roll_link",
        "right_elbow_link",
        "right_wrist_yaw_link",
    ):
        assert body in suite["push_bodies"]
        sim.model.body(body)
    spec = suite["push_points"][0]
    assert spec["name"] == "head"
    point = mesh_point_in_body(sim.model, spec["body"], spec["mesh"])
    assert point[2] > 0.3
    sim.apply_push(
        {
            "body": spec["body"],
            "force": [10, 0, 0],
            "point": point.tolist(),
            "start": 0,
            "duration": 1,
        },
        0,
    )
    torso = sim.model.body("torso_link").id
    assert sim.data.xfrc_applied[torso, 0] == 10
    assert sim.data.xfrc_applied[torso, 4] > 1
    knee = sim.model.body("left_knee_link").id
    sim.apply_push(
        {"body": "left_knee_link", "force": [0, 10, 0], "start": 0, "duration": 1},
        0,
    )
    assert sim.data.xfrc_applied[knee, 1] == 10
    assert not sim.data.xfrc_applied[torso].any()

    def shove_count(target):
        return sum(
            target == name
            for command in suite["commands"]
            for fraction in suite["checkpoints"]
            for name, _axis, _sign in scheduled_directions(suite, command, fraction)
        )

    assert shove_count("pelvis") == 24
    assert shove_count("head") == 24
    assert shove_count("left_knee_link") == 4
    assert shove_count("right_wrist_yaw_link") == 4
    directions = sum(
        len(scheduled_directions(suite, command, fraction))
        for command in suite["commands"]
        for fraction in suite["checkpoints"]
    )
    assert directions == 120
    assert [level["force_n"] for level in suite["push_levels"]] == [10.0, 50.0, 150.0]
    assert directions * len(suite["push_levels"]) == 360
    assert suite["torque_scales"] == [0.95, 1.05]


def test_push_point_names_must_differ_from_bodies(tmp_path):
    path = tmp_path / "suite.json"
    path.write_text(
        json.dumps(
            {
                "push_points": [
                    {"name": "pelvis", "body": "torso_link", "mesh": "head_link"}
                ]
            }
        )
    )
    with pytest.raises(ValueError, match="push point"):
        suite_config(path)


@pytest.mark.parametrize(
    "pose", ["standing", "seated", "supine", "prone", "left_side", "right_side"]
)
def test_counterfactual_initialization(setup, pose):
    sim, _, profile = setup
    sim.reset(0, pose)
    assert all(c.dist >= -0.005 for c in sim.data.contact)
    q = sim.data.qpos[sim.qadr]
    assert np.all(q >= sim.bounds[:, 0]) and np.all(q <= sim.bounds[:, 1])
    monitor = Monitor(sim, profile)
    _, _, _, reason = monitor.sample(q, q, np.zeros(29), pose == "standing")
    assert reason != "fall"
    if pose in ("left_side", "right_side"):
        left_axis_height = sim.data.xmat[sim.base].reshape(3, 3)[2, 1]
        assert left_axis_height == pytest.approx(-1 if pose == "left_side" else 1)
    if pose in ("supine", "prone", "left_side", "right_side"):
        assert monitor.sample(q, q, np.zeros(29), True)[-1] == "fall"


@pytest.mark.parametrize(
    ("value", "status", "reason"),
    [
        (float("nan"), "violation", "nonfinite_or_invalid_action"),
        (1000, "violation", "decoded_target_out_of_range"),
        (RuntimeError("inference failed"), "error", None),
    ],
)
def test_bad_inference_is_reported(setup, tmp_path, value, status, reason):
    sim, adapter, profile = setup
    adapter.session.value = value
    result, _ = run_scenario(sim, adapter, profile, scenario(), tmp_path / "bad.npz")
    assert result["status"] == status
    if reason:
        assert result["first_violation"]["reason"] == reason


def test_oscillating_targets_trigger_rate_limit(setup, tmp_path):
    sim, adapter, profile = setup
    profile["maxima"]["target_rate"] = 2.0

    class OscillatingSession(Session):
        def run(self, names, inputs):
            result = super().run(names, inputs)
            self.value = -self.value
            return result

    adapter.session = OscillatingSession(0.05)
    result, _ = run_scenario(sim, adapter, profile, scenario(), tmp_path / "rate.npz")
    assert result["first_violation"]["reason"] == "target_rate_limit"
    assert result["first_violation"]["time_seconds"] > sim.dt


def test_scenario_summary_counts_overlapping_issues():
    outcomes = [
        scenario(status="completed"),
        scenario(
            status="violation",
            violent_motion=True,
            violations=[
                {"reason": "joint_torque_limit"},
                {"reason": "joint_tracking_error_limit"},
            ],
        ),
        scenario(status="error"),
        scenario(status="incomplete"),
    ]
    (row,) = summarize_scenarios(outcomes)
    assert row == {
        "scenario": "test",
        "total": 4,
        "problems": 2,
        "warnings": 1,
        "limits": 1,
        "misses": 1,
        "flailing": 1,
        "falls": 0,
        "other": 0,
        "errors": 1,
        "skipped": 1,
    }


def test_flail_ignores_an_impact_shorter_than_50ms():
    from policy_safety.evaluation import overspeed_held

    counts = np.zeros(1, dtype=int)
    for _ in range(24):
        counts, held = overspeed_held(counts, [True], 25)
        assert not held
    counts, held = overspeed_held(counts, [True], 25)
    assert held
    counts, held = overspeed_held(counts, [False], 25)
    assert not held and counts[0] == 0


def test_verdicts_and_calibration():
    assert verdict([{"status": "completed"}], True) == ("within_limits", 0)
    assert verdict([{"status": "completed"}], False) == ("uncalibrated", 3)
    assert verdict([{"status": "violation"}], False) == ("violations", 1)
    assert verdict([{"status": "violation"}, {"status": "incomplete"}], False) == (
        "incomplete",
        2,
    )


def test_timeout_writes_report(tmp_path):
    report = run_worker({"output": str(tmp_path)}, 1e-6)
    assert report["exit_code"] == 2
    assert report["worker_error"] == "wall_clock_timeout"
    assert (tmp_path / "worker.log").is_file()


def test_velocity_threshold_is_enforced_only_when_configured(setup):
    sim, _, profile = setup
    sim.data.qvel[sim.vadr[0]] = 3.0
    q = sim.data.qpos[sim.qadr].copy()
    assert Monitor(sim, profile).sample(q, q, np.zeros(29), True)[-1] is None
    profile["maxima"]["joint_velocity"] = 2.0
    assert (
        Monitor(sim, profile).sample(q, q, np.zeros(29), True)[-1]
        == "joint_velocity_limit"
    )


def test_simulator_warning_is_error_not_policy_violation(setup, tmp_path, monkeypatch):
    from policy_safety import evaluation

    sim, adapter, profile = setup
    original_step = evaluation.mujoco.mj_step

    def warning_step(model, data):
        original_step(model, data)
        data.warning[0].number += 1

    monkeypatch.setattr(evaluation.mujoco, "mj_step", warning_step)
    result, _ = run_scenario(
        sim, adapter, profile, scenario(), tmp_path / "warning.npz"
    )
    assert result["status"] == "error"
    assert "simulator_warning" in result["error"]
    assert result["first_violation"] is None


def test_fall_stops_upright_rollout_early(setup, tmp_path):
    sim, adapter, profile = setup
    sim.reset(0, "supine")
    result, _ = run_scenario(
        sim, adapter, profile, scenario(seconds=0.1), tmp_path / "fall.npz"
    )
    assert result["status"] == "violation"
    assert result["first_violation"]["reason"] == "fall"
    assert result["simulated_seconds"] < 0.1


def test_invalid_suite_rejected(tmp_path):
    path = tmp_path / "suite.json"
    path.write_text(json.dumps({"seeds": [-1]}))
    with pytest.raises(ValueError, match="seeds"):
        suite_config(path)


def test_raw_clipping_is_recorded(setup, tmp_path):
    sim, adapter, profile = setup
    adapter.session.value = 1000
    result, _ = run_scenario(
        sim, adapter, profile, scenario(), tmp_path / "clipped.npz"
    )
    with np.load(tmp_path / "clipped.npz") as trace:
        assert np.any(
            trace["last_attempted_target"] != trace["last_attempted_clipped_target"]
        )
        assert trace["applied_target"].size == 0
    assert result["simulated_seconds"] == 0
    assert result["peaks"]["clipping"] > 0
    assert result["first_violation"]["reason"] == "decoded_target_out_of_range"


def test_clip_to_stops_checks_the_clipped_command(setup, tmp_path):
    sim, adapter, profile = setup
    adapter.session.value = 1000
    result, _ = run_scenario(
        sim,
        adapter,
        profile,
        scenario(),
        tmp_path / "clipped-command.npz",
        clip_to_stops=True,
    )
    assert result["first_violation"]["reason"] == "joint_tracking_error_limit"
    assert result["status"] == "violation"


def test_joint_mismatch_rejected_before_actuation(setup, tmp_path):
    sim, adapter, profile = setup
    sim.data.ctrl[sim.actuators[15]] += 0.031
    before = sim.data.ctrl.copy()
    result, _ = run_scenario(
        sim, adapter, profile, scenario(), tmp_path / "mismatch.npz"
    )
    assert result["status"] == "violation"
    assert result["first_violation"]["reason"] == "initial_joint_position_mismatch"
    assert result["first_violation"]["joint"] == "left_shoulder_pitch"
    assert result["first_violation"]["limit"] == 0.03
    assert result["simulated_seconds"] == 0
    np.testing.assert_array_equal(before, sim.data.ctrl)


@pytest.mark.parametrize("joint", [0, 15])
def test_first_policy_motion_uses_movement_limits(setup, tmp_path, joint):
    sim, adapter, profile = setup
    target = sim.data.qpos[sim.qadr].copy()
    target[joint] += 0.04
    adapter.decode = lambda raw: target.copy()
    result, _ = run_scenario(
        sim, adapter, profile, scenario(seconds=0.02), tmp_path / "first.npz"
    )
    assert result["status"] == "completed"
    assert result["simulated_seconds"] == pytest.approx(0.02)


def test_branch_restores_joint_command_history(setup, tmp_path):
    sim, adapter, profile = setup
    _, snapshots = run_scenario(
        sim, adapter, profile, scenario(seconds=0.08), tmp_path / "source.npz", [2]
    )
    assert snapshots[2]["command_history"] is not None
    sim.reset(0)
    assert sim.command_history is None
    sim.restore(snapshots[2], adapter)
    dq = np.zeros(29)
    dq[15] = 0.021
    adapter.command_velocity_and_torque = lambda: (dq.copy(), np.zeros(29))
    result, _ = run_scenario(sim, adapter, profile, scenario(), tmp_path / "branch.npz")
    assert result["first_violation"]["reason"] == "joint_velocity_target_change_limit"
    assert result["simulated_seconds"] == 0


def test_tracking_checked_between_policy_updates(setup, tmp_path, monkeypatch):
    from policy_safety import evaluation

    sim, adapter, profile = setup
    original_step = evaluation.mujoco.mj_step

    def drift_step(model, data):
        original_step(model, data)
        data.qpos[sim.qadr[15]] -= 0.31

    monkeypatch.setattr(evaluation.mujoco, "mj_step", drift_step)
    result, _ = run_scenario(
        sim, adapter, profile, scenario(), tmp_path / "tracking.npz"
    )
    assert result["first_violation"]["reason"] == "joint_tracking_error_limit"
    assert result["first_violation"]["joint"] == "left_shoulder_pitch"
    assert result["simulated_seconds"] == sim.physics_dt


def test_evaluation_enforces_profile_and_reports_sampled_coverage(
    bundle, tmp_path, monkeypatch
):
    from policy_safety.bundle import digest
    from policy_safety.evaluation import ADAPTERS, evaluate
    from policy_safety.joint_rules import DEFAULT_JOINT_PROFILE

    monkeypatch.setitem(
        ADAPTERS,
        "g1-walker-99-v1",
        lambda path, robot: G1Walker(path, robot, Session(0.01)),
    )
    suite = tmp_path / "suite.json"
    suite.write_text(
        json.dumps(
            {
                "nominal_seconds": 0.08,
                "branch_seconds": 0.04,
                "counterfactual_seconds": 0.04,
                "push_start_seconds": 0,
                "push_seconds": 0.02,
            }
        )
    )
    data = json.loads(DEFAULT_JOINT_PROFILE.read_text())
    data["joints"]["left_hip_pitch"]["max_tracking_error_rad"] = 0.001
    joint_path = tmp_path / "strict-joints.json"
    joint_path.write_text(json.dumps(data))
    output = tmp_path / "result"
    output.mkdir()
    code = evaluate(
        {
            "output": str(output),
            "robot_config": str(bundle / "robot.json"),
            "model": str(bundle / "walker.onnx"),
            "adapter": "g1-walker-99-v1",
            "suite": str(suite),
            "joint_profile": str(joint_path),
            "profile": str(
                Path(__file__).parents[1]
                / "src/policy_safety/profiles/provisional.json"
            ),
        }
    )
    report = json.loads((output / "report.json").read_text())
    assert code == 1
    assert report["hashes"]["joint_profile"] == digest(joint_path)
    assert not report["joint_movement_checks"]["uncovered_joints"]
    assert report["scenarios"][0]["first_violation"]["limit"] == 0.001
    assert report["coverage"]["planned"] == report["coverage"]["reported"] == 100
    assert report["coverage"]["error"] == report["coverage"]["incomplete"] == 0
    assert report["push_sampling"]["selected"] == 58
    assert sum(s["kind"] == "perturbation" for s in report["scenarios"]) == 58


def test_nominal_violation_still_runs_its_continuation(bundle, tmp_path, monkeypatch):
    from policy_safety.evaluation import ADAPTERS, evaluate

    monkeypatch.setitem(
        ADAPTERS,
        "g1-walker-99-v1",
        lambda path, robot: G1Walker(path, robot, Session(1000)),
    )
    suite_path = tmp_path / "suite.json"
    suite_path.write_text(
        json.dumps(
            {
                "seeds": [0],
                "commands": {"stand": [0, 0, 0]},
                "nominal_seconds": 0.08,
                "branch_seconds": 0.04,
                "checkpoints": [0.5],
                "perturbations": False,
                "poses": [],
                "torque_scales": [],
            }
        )
    )
    output = tmp_path / "result"
    output.mkdir()
    code = evaluate(
        {
            "output": str(output),
            "robot_config": str(bundle / "robot.json"),
            "model": str(bundle / "walker.onnx"),
            "suite": str(suite_path),
            "adapter": "g1-walker-99-v1",
            "profile": str(
                Path(__file__).parents[1]
                / "src/policy_safety/profiles/provisional.json"
            ),
        }
    )
    report = json.loads((output / "report.json").read_text())
    assert code == 1
    assert report["coverage"] == {
        "planned": 2,
        "reported": 2,
        "completed": 0,
        "violation": 2,
        "error": 0,
        "incomplete": 0,
    }
    assert report["scenarios"][0]["status"] == "violation"
    assert report["scenarios"][1]["status"] == "violation"
    assert report["scenarios"][0]["simulated_seconds"] == pytest.approx(0.08)


def test_tensor_shape_mismatch_rejected(setup):
    sim, _, _ = setup

    class WrongSession(Session):
        def get_outputs(self):
            return [SimpleNamespace(name="action", type="tensor(float)", shape=[1, 12])]

    with pytest.raises(ValueError, match="unexpected walker tensor"):
        G1Walker(None, sim.robot, WrongSession())


def test_calibrated_profile_requires_all_limits(tmp_path):
    profile_path = (
        Path(__file__).parents[1] / "src/policy_safety/profiles/provisional.json"
    )
    profile = json.loads(profile_path.read_text())
    profile["calibrated"] = True
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(profile))
    with pytest.raises(ValueError, match="calibrated profiles"):
        profile_config(path)


def test_external_weights_are_fingerprinted_and_confined(bundle, tmp_path):
    import onnx
    from policy_safety.bundle import model_files

    assert {p.name for p in model_files(bundle / "walker.onnx")} == {
        "walker.onnx",
        "walker.onnx.data",
    }
    graph = onnx.load(str(bundle / "walker.onnx"), load_external_data=False)
    for tensor in graph.graph.initializer:
        if tensor.data_location == onnx.TensorProto.EXTERNAL:
            for item in tensor.external_data:
                if item.key == "location":
                    item.value = "../outside.data"
    path = tmp_path / "escape.onnx"
    path.write_bytes(graph.SerializeToString())
    with pytest.raises(ValueError, match="external weights"):
        model_files(path)


def test_manifest_rejects_unhashed_dependencies(bundle):
    from policy_safety.simulation import validate_assets

    robot = json.loads((bundle / "robot.json").read_text())
    robot["assets"].remove("g1_robot.xml")
    with pytest.raises(ValueError, match="XML dependency"):
        validate_assets(bundle, robot)


def test_failing_worker_returns_error_report(tmp_path):
    report = run_worker({"output": str(tmp_path)}, 30)
    assert report["exit_code"] == 2
    assert "KeyError" in report["worker_error"]


def test_unclamped_leg_torque_rejected(setup, tmp_path):
    sim, adapter, profile = setup
    # A large measured speed produces demand even with an unchanged target.
    sim.data.qvel[sim.vadr[0]] = 100.0
    result, _ = run_scenario(sim, adapter, profile, scenario(), tmp_path / "torque.npz")
    assert result["status"] == "violation"
    assert result["first_violation"]["reason"] == "joint_torque_limit"
    assert result["first_violation"]["joint"] == "left_hip_pitch"
    assert result["first_violation"]["time_seconds"] == 0


def test_mp4_starts_with_ftyp(tmp_path):
    from policy_safety.render import write_mp4

    frames = [np.full((8, 12, 3), fill, dtype=np.uint8) for fill in (10, 200)]
    path = write_mp4(tmp_path / "clip.mp4", frames, 20)
    assert path.read_bytes()[4:8] == b"ftyp"
    with pytest.raises(RuntimeError, match="No such file"):
        write_mp4(tmp_path / "missing" / "clip.mp4", frames, 20)


@pytest.mark.parametrize(
    "joint,spin,flail", [(9, 0, False), (9.1, 0, True), (0, 5.4, False), (0, 5.5, True)]
)
def test_sustained_speed_monitor_uses_profile_and_body_caps(
    setup, tmp_path, monkeypatch, joint, spin, flail
):
    sim, adapter, profile = setup

    def step(model, data):
        data.time += model.opt.timestep
        data.qvel[sim.vadr[0]] = joint
        data.qvel[3] = spin

    monkeypatch.setattr("policy_safety.evaluation.mujoco.mj_step", step)
    result, _ = run_scenario(
        sim, adapter, profile, scenario(seconds=0.06), tmp_path / "speed.npz"
    )
    assert result["violent_motion"] is flail
    if flail:
        reason = "joint_velocity_limit" if joint else "base_angular_speed_limit"
        assert result["first_violation"]["reason"] == reason
        assert result["simulated_seconds"] == pytest.approx(0.05)
    else:
        assert result["status"] == "completed"




def test_sampling_preserves_coverage_and_baselines():
    suite = suite_config()
    plan = plan_scenarios(suite)
    pushes = plan["pushes"]
    assert plan["baseline_count"] == 42
    assert plan["push_candidates"] == 360
    assert len(pushes) == len({tuple(push.items()) for push in pushes}) == 58
    assert len({(p["target"], p["level"]) for p in pushes}) == 45
    assert plan_scenarios(suite) == plan
    suite["sample_seed"] = 1
    assert plan_scenarios(suite)["pushes"] != pushes
    suite["max_scenarios"] = None
    assert len(plan_scenarios(suite)["pushes"]) == 360
    suite.update(seeds=[0, 1], max_scenarios=100)
    plan = plan_scenarios(suite)
    assert plan["baseline_count"] == 84 and len(plan["pushes"]) == 16
    suite["max_scenarios"] = 83
    with pytest.raises(ValueError, match="84 baseline scenarios"):
        plan_scenarios(suite)
