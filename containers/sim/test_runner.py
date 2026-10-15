"""Safety checks for container commands against the MuJoCo G1."""

import math
import sys
import tempfile
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [
    str(ROOT / "packages/policy_safety/src"),
    str(ROOT / "robot_safety"),
    str(ROOT / "containers/sim"),
]

from plant import POSES, Plant
from runner import LAUNCH_SECONDS, Runner

from robot_safety.config import Config, ThrashConfig
from robot_safety.core import Supervisor
from robot_safety.hardware_adapter import G1HardwareAdapter
from robot_safety.hardware_constants import UPPER_BODY_INDICES
from robot_safety.hardware_profile import HardwareProfile
from robot_safety.thrash import ThrashDetector


def hold(runner, state, sequence, **overrides):
    q = list(state["q"])
    frame = {
        "session_id": runner.session_id,
        "sequence": sequence,
        "observation_id": state["observation_id"],
        "q": q,
        "dq": [0.0] * 29,
        "kp": [40.0] * 29,
        "kd": [2.0] * 29,
        "tau": [0.0] * 29,
    }
    frame.update(overrides)
    return frame


class RunnerTest(unittest.TestCase):
    def test_thrashing_telemetry_tracks_frames_in_live_and_replay(self):
        import json
        import os

        from live import LiveSession
        from view import export_replay, report_summary

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            live = root / "live.jsonl"
            trace = root / "trace.npz"
            with patch.dict(os.environ, {"G1_SIM_LIVE": str(live), "G1_SIM_TRACE": str(trace)}):
                runner = Runner(record_trace=True)
                # Isolate output diagnostics after startup has completed.
                runner.hardware.adapter.balance_released = True
                runner.startup.command = lambda now, stamp, state, command, **kw: command
                state = runner.feedback(0.0)
                runner.command(hold(runner, state, 0), 0.0)
                runner.step(0.0)
                bad = hold(runner, state, 1)
                bad["tau"][0] = 89.0
                runner.command(bad, 0.01)
                runner.handle({"op": "abort", "reason": "later_fault"})
                report = runner.report()
            samples = [json.loads(line) for line in live.read_text().splitlines()]
            telemetry = samples[-1]["thrash"]
            self.assertEqual(telemetry["mode"], "monitor")
            self.assertEqual(len(telemetry["metrics"]["oscillatory_rms"]), 29)
            self.assertEqual(telemetry["thresholds"]["persistence_secs"], runner.config.thrash.persistence_secs)
            self.assertIsNone(samples[0]["stop"])
            self.assertEqual(samples[-1]["stop"]["reason"], "later_fault")
            summary = report_summary(report)
            self.assertIn("Limit warning", summary)
            self.assertIn("left_hip_pitch", summary)
            self.assertIn("89", summary)
            self.assertIn("88", summary)
            runner.thrash_detector.reset()
            self.assertEqual(runner.trace_thrash[-1], telemetry)
            session = LiveSession(root, live, root / "start", root / "report.json", "standing", None)
            frames = session.snapshot(0)["frames"]
            self.assertEqual([frame["thrash"] for frame in frames], [s["thrash"] for s in samples])
            dest = export_replay(trace, root / "view")
            motion = json.loads((dest / "motion.json").read_text())
            self.assertEqual(motion["thrash"], [s["thrash"] for s in samples])
            self.assertEqual(len(motion["thrash"]), motion["frames"])
            self.assertEqual(len(motion["joint_names"]), 29)
            self.assertEqual(motion["stop"][-1], frames[-1]["stop"])
            self.assertIn("later_fault", motion["stop"][-1]["description"])
            runner._reset_trace()
            self.assertEqual(len(runner.trace_thrash), 1)
            self.assertEqual(runner.trace_thrash[0]["metrics"], {})
            runner.report()

    def test_startup_progress_reaches_live_and_replay_viewers(self):
        import json
        import os

        from live import LiveSession
        from view import export_replay

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            live, trace = root / "live.jsonl", root / "trace.npz"
            with patch.dict(os.environ, {"G1_SIM_LIVE": str(live), "G1_SIM_TRACE": str(trace)}):
                runner = Runner(record_trace=True)
                for step in range(3):
                    state = runner.feedback(step * 0.02)
                    runner.command(hold(runner, state, step), step * 0.02)
                    runner.step(step * 0.02)
                runner.report()
            samples = [json.loads(line) for line in live.read_text().splitlines()]
            last = samples[-1]["startup"]
            self.assertEqual(last["launch_secs"], LAUNCH_SECONDS)
            self.assertAlmostEqual(last["elapsed_secs"], 0.06)
            self.assertIsNone(last["handoff_at_secs"])
            session = LiveSession(root, live, root / "start", root / "report.json", "standing", None)
            frames = session.snapshot(0)["frames"]
            self.assertEqual(frames[-1]["startup"],
                             "Startup balance: 0.1 s in, 4.9 s left before the policy takes over")
            motion = json.loads((export_replay(trace, root / "view") / "motion.json").read_text())
            self.assertEqual(motion["startup"], [frame["startup"] for frame in frames])

    def test_live_page_is_never_cached_and_names_its_run(self):
        import json
        import threading
        import urllib.request
        from http.server import ThreadingHTTPServer

        import live
        from view import export_idle

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            first = json.loads((export_idle(root / "view") / "motion.json").read_text())["page_id"]
            second = json.loads((export_idle(root / "view") / "motion.json").read_text())["page_id"]
            self.assertNotEqual(first, second)
            session = live.LiveSession(root / "view", root / "live.jsonl", root / "start",
                                       root / "report.json", "standing", None)
            servers = []

            class Recorded(ThreadingHTTPServer):
                def __init__(self, *args):
                    super().__init__(*args)
                    servers.append(self)

            with patch.object(live, "ThreadingHTTPServer", Recorded):
                threading.Thread(target=live.serve, args=(session, "127.0.0.1", 0), daemon=True).start()
                for _ in range(200):
                    if servers:
                        break
                    time.sleep(0.01)
            server = servers[0]
            self.addCleanup(server.server_close)
            self.addCleanup(server.shutdown)
            base = f"http://127.0.0.1:{server.server_address[1]}"
            for path in ("/", "/frames?after=0"):
                with urllib.request.urlopen(base + path) as response:
                    self.assertEqual(response.headers["Cache-Control"], "no-store")
                    body = response.read()
            self.assertEqual(json.loads(body)["page_id"], second)
            third = json.loads((export_idle(root / "view") / "motion.json").read_text())["page_id"]
            with urllib.request.urlopen(base + "/frames?after=0") as response:
                self.assertEqual(json.loads(response.read())["page_id"], third)

    def test_viewer_describes_startup_handoff_and_policy_control(self):
        from view import describe_startup

        base = {"phase": "balancing", "launch_secs": 5.0, "handoff_at_secs": None, "blend_secs": 1.0}
        cases = (
            (dict(base, elapsed_secs=2.0), "Startup balance: 2.0 s in, 3.0 s left before the policy takes over"),
            (dict(base, elapsed_secs=5.4), "Startup balance: 5.4 s in, waiting for the robot to settle"),
            (dict(base, phase="handoff", elapsed_secs=5.5, handoff_at_secs=5.1),
             "Handing over to the policy: 0.4 of 1.0 s"),
            (dict(base, phase="running", elapsed_secs=7.0, handoff_at_secs=5.1), "Policy in control since 5.1 s"),
            (dict(base, phase="fault", elapsed_secs=1.0), ""),
            (None, ""),
        )
        for startup, text in cases:
            with self.subTest(text=text):
                self.assertEqual(describe_startup(startup), text)

    def test_viewer_describes_velocity_torque_thrash_and_timeout_stops(self):
        from view import describe_stop, report_summary

        cases = (
            ("joint_velocity_limit", {"joint": "left_elbow", "value": 38, "limit": 37, "units": "rad/s"},
             ("q dot", "left_elbow", "38", "37", "rad/s")),
            ("joint_torque_limit", {"joint": "left_wrist_pitch", "value": 6, "limit": 5,
                                    "units": "Nm", "torque_kind": "PD demand"},
             ("left_wrist_pitch", "6", "5", "Nm", "PD demand")),
            ("base_angular_speed_limit", {"joint": "0", "value": 6.16, "limit": 6, "units": "rad/s"},
             ("Base axis: x", "6.16", "6")),
            ("thrashing_energy", {"joint_indices": [9, 10], "oscillatory_rms": [2.0] * 29,
                                  "weighted_rms": 1.6, "weighted_rms_limit": 1.5,
                                  "persistence_elapsed_secs": 0.5},
             ("right_knee", "right_ankle_pitch", "osc RMS=2.000",
              "Whole-body RMS: 1.600 / 1.500", "500 ms")),
            ("action_timeout", None, ("action_timeout",)),
            ("invalid_action", {"field": "q", "joint_index": 20, "value": 11,
                                "bounds": [-10, 10], "rejection": "invalid_command"},
             ("left_wrist_pitch", "11", "Bounds", "invalid_command")),
        )
        for reason, detail, expected in cases:
            with self.subTest(reason=reason):
                record = {"reason": reason, "detail": detail, "stop_time_seconds": 0.25}
                # Legacy reports still include the complete stop details.
                text = report_summary(record)
                self.assertIn("0.250 s", text)
                for value in expected:
                    self.assertIn(value, text)
                self.assertEqual(describe_stop({"reason": reason, "detail": detail})["detail"], detail)

    def test_shared_thrashing_modes_gate_simulated_output(self):
        for mode, safety in (("monitor", True), ("enforce", True), ("off", True),
                             ("monitor", False), ("enforce", False), ("off", False)):
            with self.subTest(mode=mode, safety=safety):
                runner = Runner(safety_checks=safety, thrash_config=ThrashConfig(mode=mode))
                self.assertIsInstance(runner.thrash_detector, ThrashDetector)
                initial_q = runner.plant.q().copy()

                def oscillate(runner=runner, initial_q=initial_q):
                    phase = 2 * math.pi * 8 * (runner.sim_time + 0.002)
                    joints = [0, 1, 2, 3, 6, 7, 8, 9]
                    runner.plant.data.qpos[runner.plant.qadr[joints]] = initial_q[joints] + 0.12 * math.sin(phase)
                    runner.plant.data.qvel[runner.plant.vadr[joints]] = 0.12 * 2 * math.pi * 8 * math.cos(phase)
                    return runner.plant.dq()

                with patch.object(runner.plant, "step_physics", side_effect=oscillate):
                    for step in range(100):
                        now = step * 0.02
                        state = runner.feedback(now)
                        self.assertIsNone(runner.command(hold(runner, state, step), now))
                        runner.step(now)
                        if runner.reason:
                            break
                report = runner.report()
                if mode == "enforce":
                    self.assertEqual(runner.reason, "thrashing_energy")
                    self.assertIsNone(runner.active)
                    self.assertEqual(runner.command(hold(runner, state, step + 1), now), runner.reason)
                    self.assertEqual(report["thrash"]["event"]["reason"], runner.reason)
                else:
                    self.assertIsNone(runner.reason)
                    self.assertIsNotNone(runner.active)
                    self.assertEqual(report["thrash"]["event"] is not None, mode == "monitor")
                self.assertEqual(report["checks"]["thrash_enforcement"], mode == "enforce")
                self.assertEqual(report["thrash"]["mode"], mode)

    def test_external_checker_abort_and_report(self):
        runner = Runner(thrash_config=ThrashConfig(mode="off"))
        state = runner.feedback(0.0)
        runner.command(hold(runner, state, 0), 0.0)
        runner.handle({"op": "abort", "reason": "thrashing_energy"})
        runner.handle({"op": "abort", "reason": "later_fault"})
        self.assertEqual(runner.reason, "thrashing_energy")
        self.assertIsNone(runner.active)
        event = {"reason": "thrashing_energy", "joint_indices": [15, 16]}
        report = runner.handle({"op": "report", "thrash": {"mode": "enforce", "event": event}})
        self.assertEqual(report["thrash"]["event"], event)
        self.assertTrue(report["checks"]["thrash_enforcement"])

    def test_safety_checks_default_to_enabled(self):
        self.assertTrue(Runner().report()["safety_checks"])

    def test_unchecked_commands_bypass_limits_and_timeouts(self):
        runner = Runner(safety_checks=False)
        state = runner.feedback(0.0)
        frame = hold(
            runner,
            state,
            0,
            q=[12.0] * 29,
            dq=[60.0] * 29,
            kp=[600.0] * 29,
            kd=[60.0] * 29,
            tau=[300.0] * 29,
        )
        self.assertIsNone(runner.command(frame, 1.0))
        np.testing.assert_array_equal(runner.active["tau"], frame["tau"])
        state = runner.feedback(1.0)
        self.assertIsNone(runner.command(hold(runner, state, 1), 1.0))
        runner.step(2.0)
        self.assertIsNone(runner.reason)
        report = runner.report()
        self.assertEqual(report["status"], "completed")
        self.assertFalse(report["safety_checks"])
        self.assertFalse(report["fall_check"])
        self.assertFalse(report["hardware_authorization"])

    def test_checked_action_timeout_still_latches(self):
        runner = Runner()
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        runner.step(0.2)
        self.assertEqual(runner.reason, "action_timeout")

    def test_clipped_ankle_is_sent_and_used_for_response(self):
        runner = Runner()
        # Isolate output diagnostics after startup has completed.
        runner.hardware.adapter.balance_released = True
        runner.startup.command = lambda now, stamp, state, command, **kw: command
        index = runner.plant.joint_names.index("right_ankle_pitch_joint")
        runner.plant.data.qpos[runner.plant.qadr[index]] = -0.86
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        frame = hold(runner, state, 1)
        frame["q"][index] = -0.892688
        self.assertIsNone(runner.command(frame, 0.02))
        self.assertEqual(frame["q"][index], -0.892688)
        self.assertEqual(runner.active["q"][index], -0.87267)
        with patch.object(runner.plant, "step_physics", return_value=np.zeros(29)):
            runner.step(0.02)
        self.assertEqual(runner.plant.data.ctrl[runner.plant.actuators[index]], -0.87267)
        self.assertAlmostEqual(runner.response_error[index], 0.01267)
        report = runner.report()
        self.assertEqual(report["status"], "within_limits")
        warning = report["warnings"]["right_ankle_pitch"]
        self.assertEqual(warning["q_proposed"], -0.892688)
        self.assertEqual(warning["q_commanded"], -0.87267)

    def test_unchecked_observation_has_no_hidden_age_limit(self):
        runner = Runner(safety_checks=False)
        first = runner.feedback(0.0)
        for step in range(600):
            runner.feedback(step * 0.02)
        self.assertIsNone(runner.command(hold(runner, first, 0), 12.0))
        self.assertEqual(
            runner.command(hold(runner, first, 1, observation_id=10000), 12.0),
            "stale_observation",
        )

    def test_unchecked_fall_and_overspeed_do_not_latch(self):
        from unittest.mock import patch

        runner = Runner(safety_checks=False)
        runner.plant.data.qpos[runner.plant.base_qadr + 2] = 0.1
        runner.plant.data.qvel[runner.plant.base_vadr + 3] = 100.0
        mujoco.mj_forward(runner.plant.model, runner.plant.data)
        with patch.object(
            runner.plant, "step_physics", return_value=np.full(29, 100.0)
        ):
            for step in range(2):
                runner.step(step * 0.02)
        self.assertIsNone(runner.reason)

    def test_unchecked_run_still_rejects_invalid_vectors(self):
        for bad in ([0.0] * 28, [float("nan")] * 29, [float("inf")] * 29, ["bad"] * 29):
            with self.subTest(bad=bad[0]):
                runner = Runner(safety_checks=False)
                state = runner.feedback(0.0)
                reason = runner.command(hold(runner, state, 0, q=bad), 0.0)
                self.assertEqual(reason, "invalid_command")
                self.assertIsNone(runner.active)
                self.assertEqual(runner.report()["status"], "error")

    def test_unchecked_run_still_requires_session_and_sequence(self):
        runner = Runner(safety_checks=False)
        state = runner.feedback(0.0)
        self.assertEqual(
            runner.command(hold(runner, state, 0, session_id="other"), 0.0),
            "session_mismatch",
        )
        runner = Runner(safety_checks=False)
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        self.assertEqual(runner.command(hold(runner, state, 0), 0.0), "sequence")

    def test_unchecked_run_without_commands_is_not_completed(self):
        self.assertEqual(Runner(safety_checks=False).report()["status"], "no_command")

    def test_hold_stays_up(self):
        runner = Runner()
        target = None
        for step in range(50):
            state = runner.feedback(step * 0.02)
            if target is None:
                target = list(state["q"])
            reason = runner.command(hold(runner, state, step, q=target), step * 0.02)
            self.assertIsNone(reason)
            runner.step(step * 0.02 + 0.01)
        report = runner.report()
        self.assertEqual(report["status"], "within_limits")
        self.assertGreater(runner.plant.state()["height"], 0.5)
        self.assertFalse(report["hardware_authorization"])

    def test_policy_launches_only_after_the_operator_period(self):
        runner = Runner()
        q = runner.plant.q().tolist()
        for step in range(round((LAUNCH_SECONDS + 1) / 0.02)):
            time = step * 0.02
            state = runner.feedback(time)
            runner.command({
                "session_id": runner.session_id, "sequence": step,
                "observation_id": state["observation_id"], "q": q, "dq": [0.0] * 29,
                "kp": [40.0] * 29, "kd": [2.0] * 29, "tau": [0.0] * 29,
            }, time)
            runner.step(time)
            if runner.sim_time < LAUNCH_SECONDS:
                self.assertIsNone(runner.startup.handoff)
        self.assertIsNotNone(runner.startup.handoff)
        self.assertGreaterEqual(runner.startup.handoff, LAUNCH_SECONDS)

    def test_circle_walks_without_falling(self):
        model = ROOT / "containers/examples/policies/walker/walker.onnx"
        if not model.is_file():
            self.skipTest("cached walker weights are not on this machine")
        runner = Runner()
        start = None
        for step in range(round((LAUNCH_SECONDS + 3) / 0.02)):
            result = runner.circle(step * 0.02)
            self.assertIsNone(result.get("reason"))
            position = runner.plant.state()["position"]
            if start is None:
                start = position
        end = runner.plant.state()
        moved = sum((end["position"][i] - start[i]) ** 2 for i in range(2)) ** 0.5
        self.assertGreater(end["height"], 0.5)
        self.assertGreater(end["upright"], 0.8)
        self.assertGreater(moved, 0.3)
        self.assertIsNone(runner.thrash_detector.event)

    def test_circle_reports_partial_checks_in_report_and_standalone_replay(self):
        import json
        import os
        from unittest.mock import patch

        from view import export_replay, report_summary

        runner = Runner(record_trace=True)
        runner.circle(0.0)
        with tempfile.TemporaryDirectory() as tmp:
            trace = Path(tmp) / "trace.npz"
            with patch.dict(os.environ, {"G1_SIM_TRACE": str(trace)}):
                report = runner.report()
            self.assertEqual(report["status"], "completed")
            self.assertEqual(report["control_path"], "direct")
            self.assertFalse(report["checks"]["command_limits"])
            self.assertFalse(report["checks"]["command_protocol"])
            self.assertFalse(report["checks"]["observation_freshness"])
            self.assertTrue(report["checks"]["joint_speed"])
            self.assertTrue(report["checks"]["fall"])
            self.assertIn("PARTIAL", report_summary(report))
            dest = export_replay(trace, Path(tmp) / "view")
            motion = json.loads((dest / "motion.json").read_text())
            self.assertIn("PARTIAL", motion["summary"])
            self.assertIn("submission checks bypassed", motion["summary"])

    def test_circle_still_checks_falls_and_can_disable_runtime_checks(self):
        for safety_checks in (True, False):
            with self.subTest(safety_checks=safety_checks):
                runner = Runner(safety_checks=safety_checks)
                runner.circle(0.0)
                runner.plant.data.qpos[runner.plant.base_qadr + 2] = 0.1
                mujoco.mj_forward(runner.plant.model, runner.plant.data)
                runner.circle(0.02)
                report = runner.report()
                self.assertEqual(report["checks"]["fall"], safety_checks)
                self.assertEqual(report["checks"]["joint_speed"], safety_checks)
                self.assertEqual(
                    report["status"], "violation" if safety_checks else "completed"
                )
                if safety_checks:
                    self.assertEqual(report["reason"], "fall")

    def test_torque_limit_warns_and_a_later_error_still_stops(self):
        runner = Runner()
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        q = list(state["q"])
        q[0] = q[0] + 2.3
        self.assertIsNone(runner.command(hold(runner, state, 1, q=q), 0.02))
        self.assertIsNotNone(runner.active)
        demand = next(item for item in runner.position_warnings["left_hip_pitch"]["limits"]
                      if item["torque_kind"] == "PD demand")
        self.assertEqual(demand["reason"], "joint_torque_limit")
        self.assertGreater(demand["value"], demand["limit"])
        self.assertIsNone(runner.command(hold(runner, state, 2), 0.03))
        runner.handle({"op": "abort", "reason": "later_fault"})
        self.assertEqual(runner.reason, "later_fault")
        self.assertIsNone(runner.active)
        with patch.object(runner.plant, "step_physics", return_value=np.zeros(29)):
            runner.step(0.04)
        np.testing.assert_array_equal(runner.plant.model.actuator_gainprm[runner.plant.actuators, 0], 0)

    def test_response_diagnostics_sample_after_a_complete_control_interval(self):
        for follows_target in (True, False):
            with self.subTest(follows_target=follows_target):
                runner = Runner()
                # Isolate output diagnostics after startup has completed.
                runner.hardware.adapter.balance_released = True
                runner.startup.command = lambda now, stamp, state, command, **kw: command
                index = 15
                state = runner.feedback(0.0)
                self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
                state = runner.feedback(0.02)
                target = list(state["q"])
                target[index] += 0.02
                self.assertIsNone(runner.command(hold(runner, state, 1, q=target), 0.02))
                ticks = 0

                def move_on_last_tick(
                    follows_target=follows_target, runner=runner,
                    index=index, target=target,
                ):
                    nonlocal ticks
                    ticks += 1
                    if follows_target and ticks == runner.plant.substeps:
                        runner.plant.data.qpos[runner.plant.qadr[index]] = target[index]
                    return runner.plant.dq()

                with patch.object(runner.plant, "step_physics", side_effect=move_on_last_tick):
                    runner.step(0.02)
                self.assertEqual(ticks, runner.plant.substeps)
                self.assertAlmostEqual(runner.sim_time, 0.02)
                self.assertIsNone(runner.reason)
                expected_error = 0.0 if follows_target else -0.02
                self.assertAlmostEqual(runner.response_error[index], expected_error)

    def test_direct_command_limits_torque_target_without_stopping(self):
        runner = Runner()
        # Isolate output diagnostics after startup has completed.
        runner.hardware.adapter.balance_released = True
        runner.startup.command = lambda now, stamp, state, command, **kw: command
        runner.control_path = "direct"
        runner.commands = 1
        runner.last_command_time = 0.0
        q = runner.plant.q()
        q[20] += 0.5
        runner.active = {
            "q": q,
            "dq": np.zeros(29),
            "kp": np.full(29, 60.0),
            "kd": np.full(29, 1.5),
            "tau": np.zeros(29),
        }
        runner.submitted = runner.active
        with patch.object(runner.plant, "step_physics", return_value=np.zeros(29)):
            result = runner.step(0.02)
        self.assertIsNone(result["reason"])
        self.assertIsNotNone(runner.active)
        warning = runner.position_warnings["left_wrist_pitch"]
        clipping = warning["torque_target_clipping"]
        self.assertTrue(clipping["active"])
        self.assertAlmostEqual(clipping["requested_torque_nm"], 30.0)
        self.assertAlmostEqual(clipping["command_budget_nm"], 4.5)
        self.assertAlmostEqual(clipping["hard_limit_nm"], 5.0)
        self.assertAlmostEqual(runner.active["q"][20] - runner.plant.q()[20], 0.075)
        self.assertAlmostEqual(clipping["q_commanded"], runner.active["q"][20])
        demand = runner.plant.demand(
            runner.active["q"], runner.active["dq"], runner.active["kp"],
            runner.active["kd"], runner.active["tau"], runner.plant.q(), runner.plant.dq())
        self.assertAlmostEqual(demand[20], 4.5)
        self.assertEqual(
            [item for item in warning.get("limits", []) if item["torque_kind"] == "PD demand"],
            [],
        )

    def test_measured_joint_speed_and_base_spin_trigger_hardware_damping(self):
        for base_spin in (False, True):
            with self.subTest(base_spin=base_spin):
                runner = Runner()
                state = runner.feedback(0.0)
                self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
                runner.step(0.0)
                if base_spin:
                    runner.plant.data.qvel[runner.plant.base_vadr + 3] = 7
                    expected = "base_angular_speed_limit"
                else:
                    index, _, limits = runner.rules.entries[3]
                    runner.plant.data.qvel[runner.plant.vadr[index]] = limits.max_velocity_rad_secs + 1
                    expected = "joint_velocity_limit"
                mujoco.mj_forward(runner.plant.model, runner.plant.data)
                with patch.object(runner.plant, "step_physics", return_value=np.zeros(29)):
                    runner.step(0.02)
                self.assertIn(expected, runner.reason)
                self.assertEqual(runner.hardware.snapshot()["phase"], "damping")

    def test_feedforward_torque_is_a_warning(self):
        from view import report_summary

        runner = Runner()
        state = runner.feedback(0.0)
        tau = [0.0] * 29
        tau[0] = 89.0
        self.assertIsNone(runner.command(hold(runner, state, 0, tau=tau), 0.0))
        self.assertIsNotNone(runner.active)
        report = runner.report()
        self.assertEqual(report["status"], "within_limits")
        self.assertIsNone(report["reason"])
        feedforward = next(item for item in report["warnings"]["left_hip_pitch"]["limits"]
                           if item["reason"] == "joint_feedforward_torque_limit")
        self.assertEqual(feedforward["value"], 89.0)
        self.assertEqual(feedforward["limit"], 88.0)
        self.assertIn("Run continued", report_summary(report))

    def test_summary_flags_a_stop_during_a_host_stall(self):
        from view import report_summary

        report = {"status": "violation", "reason": "observation_timeout", "stop_time_seconds": 0.16}
        self.assertNotIn("Inconclusive", report_summary(report))
        report["host_stall"] = {"longest_gap_secs": 0.42}
        self.assertIn("Inconclusive: the simulator was stalled for 0.42 s", report_summary(report))

    def test_initial_pose_gap_does_not_reject_a_command(self):
        runner = Runner()
        state = runner.feedback(0.0)
        q = list(state["q"])
        q[0] = q[0] + 0.05
        reason = runner.command(hold(runner, state, 0, q=q), 0.0)
        self.assertIsNone(reason)
        self.assertAlmostEqual(runner.active["q"][0], q[0])

    def test_wrist_torque_is_a_warning(self):
        runner = Runner()
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        q = list(state["q"])
        q[20] = q[20] + 0.5
        self.assertIsNone(runner.command(hold(runner, state, 1, q=q, kp=[0.0] * 29), 0.02))
        demand = next(item for item in runner.position_warnings["left_wrist_pitch"]["limits"]
                      if item["torque_kind"] == "PD demand")
        self.assertEqual(demand["reason"], "joint_torque_limit")
        self.assertGreater(demand["value"], demand["limit"])
        self.assertIsNone(runner.reason)

    def test_checked_targets_match_hardware_for_every_joint(self):
        for index in range(29):
            for case in ("gains", "clip", "velocity", "feedforward", "demand", "initial"):
                with self.subTest(index=index, case=case):
                    runner = Runner()
                    hardware = G1HardwareAdapter(HardwareProfile.load(runner.rules.path))
                    limits = runner.rules.entries[index][2]
                    if case == "clip":
                        runner.plant.data.qpos[runner.plant.qadr[index]] = limits.position_max_rad
                    state = runner.feedback(0.0)
                    initial = case == "initial"
                    if not initial:
                        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
                    frame = hold(runner, state, 1, kp=[500.0] * 29, kd=[50.0] * 29)
                    if case == "clip":
                        frame["q"][index] = limits.position_max_rad + 0.01
                    elif case == "velocity":
                        frame["dq"][index] = limits.max_velocity_rad_secs + 0.01
                    elif case == "feedforward":
                        frame["tau"][index] = limits.max_torque_nm + 0.01
                    elif case == "demand":
                        # Feedforward alone passes, but PD error crosses the torque bound.
                        frame["q"][index] += 0.01
                        frame["tau"][index] = limits.max_torque_nm
                    elif case == "initial":
                        frame["q"][index] += 0.04
                    hardware.command = frame
                    hardware.feedback = SimpleNamespace(q=runner.plant.q(), dq=runner.plant.dq())
                    hardware.last_output_secs = None if initial else 0.0
                    try:
                        expected = hardware.checked_targets(0.02)
                    except ValueError as exc:
                        if exc.reason in ("joint_torque_limit", "joint_feedforward_torque_limit"):
                            self.assertIsNone(runner.command(frame, 0.02))
                            self.assertIsNotNone(runner.active)
                            self.assertEqual(runner.active["kp"][index], limits.kp)
                            self.assertEqual(runner.active["kd"][index], limits.kd)
                            noted = runner.position_warnings[runner.rules.sdk_names[index]]["limits"]
                            self.assertTrue(any(item["value"] > item["limit"] for item in noted))
                        else:
                            reason = runner.command(frame, 0.02)
                            self.assertEqual(reason, exc.reason)
                            self.assertEqual(runner.detail, exc.details)
                            self.assertIsNone(runner.active)
                    else:
                        self.assertIsNone(runner.command(frame, 0.02))
                        for joint in UPPER_BODY_INDICES:
                            for field, value in expected[joint].items():
                                self.assertEqual(runner.active[field][joint], value)
                        self.assertEqual(runner.active["kp"][index], limits.kp)
                        self.assertEqual(runner.active["kd"][index], limits.kd)

    def test_protocol_rejections_match_supervisor(self):
        cases = (
            {"session_id": "wrong"}, {"sequence": True}, {"sequence": 0},
            {"observation_id": True}, {"observation_id": 900},
            {"q": [True] * 29}, {"q": ["0"] * 29}, {"q": [0.0] * 28},
            {"q": [float("nan")] * 29}, {"kp": [-1.0] * 29},
        )
        for overrides in cases:
            with self.subTest(overrides=overrides):
                runner = Runner()
                state = runner.feedback(0.0)
                supervisor = Supervisor(runner.config)
                supervisor.state(state, False, 0.0)
                supervisor.control("arm", 0.0)
                for sequence, changes in ((0, {}), (1, overrides)):
                    frame = hold(runner, state, sequence)
                    frame.update(changes)
                    message = dict(frame, session=(supervisor.policy_session_id
                                   if frame["session_id"] == runner.session_id else "wrong"),
                                   seq=frame["sequence"], state_id=frame["observation_id"])
                    accepted = supervisor.accept(message, 0.02)
                    self.assertEqual(runner.command(frame, 0.02) is None, accepted)
                self.assertIsNone(runner.active)
                self.assertIsNone(supervisor.accepted_policy_command)

    def test_deadlines_match_supervisor(self):
        for field, reason in (("action_timeout", "action_timeout"),
                              ("robot_timeout", "robot_state_timeout"),
                              ("observation_timeout", "observation_timeout"),
                              ("loop_timeout", "loop_progress")):
            with self.subTest(field=field):
                config = replace(Config(action_timeout=1, robot_timeout=1,
                                        observation_timeout=1, loop_timeout=1), **{field: 0.2})
                runner = Runner(config=config)
                state = runner.feedback(0.0)
                supervisor = Supervisor(config)
                supervisor.state(state, False, 0.0)
                supervisor.control("arm", 0.0)
                frame = hold(runner, state, 0)
                self.assertIsNone(runner.command(frame, 0.0))
                self.assertTrue(supervisor.accept(dict(frame, session=supervisor.policy_session_id,
                                                       seq=0, state_id=1), 0.0))
                supervisor.cycle(0.0)
                with patch.object(runner.plant, "step_physics", return_value=np.zeros(29)):
                    runner.step(0.0)
                    runner.step(0.2)
                supervisor.cycle(0.2)
                self.assertEqual(runner.reason, reason)
                self.assertEqual(supervisor.stop_reason, reason)
                self.assertIsNone(runner.active)
                self.assertIsNone(supervisor.accepted_policy_command)

    def test_late_replacement_cannot_refresh_expired_action(self):
        runner = Runner()
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        state = runner.feedback(0.15)
        self.assertEqual(runner.command(hold(runner, state, 1), 0.2), "action_timeout")
        self.assertEqual(runner.last_command_time, 0.0)
        self.assertIsNone(runner.active)

    def test_bad_shape_session_and_sequence(self):
        runner = Runner()
        state = runner.feedback(0.0)
        short = hold(runner, state, 0)
        short["q"] = short["q"][:-1]
        self.assertEqual(runner.command(short, 0.0), "invalid_command")
        runner = Runner()
        state = runner.feedback(0.0)
        wrong = hold(runner, state, 0, session_id="nope")
        self.assertEqual(runner.command(wrong, 0.0), "session_mismatch")
        runner = Runner()
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        self.assertEqual(runner.command(hold(runner, state, 0), 0.02), "sequence")

    def test_stale_observation_rejected(self):
        runner = Runner()
        state = runner.feedback(0.0)
        self.assertEqual(
            runner.command(hold(runner, state, 0), 0.2), "stale_observation"
        )

    def test_missing_command_is_not_a_pass(self):
        runner = Runner()
        runner.feedback(0.0)
        runner.step(0.01)
        self.assertEqual(runner.report()["status"], "no_command")

    def test_report_only_run_does_not_retain_poses(self):
        runner = Runner(record_trace=False)
        runner.circle(0.0)
        self.assertEqual(runner.trace_qpos, [])
        self.assertEqual(runner.trace_time, [])
        report = runner.report()
        self.assertIsNone(report["trace"])
        self.assertGreater(report["seconds"], 0)
        self.assertIn("minimum_pelvis_height", report["motion"])

    def test_combined_step_issues_observation_after_physics(self):
        runner = Runner(safety_checks=False)
        before = runner.feedback(1.0)
        runner.command(hold(runner, before, 0), 1.0)
        with patch("runner.time.monotonic", side_effect=[10.04, 10.05]):
            state = runner.handle(
                {"op": "step", "time": 1.02, "feedback": True, "clock_origin": 9.0}
            )
        self.assertTrue(state["ok"])
        self.assertEqual(state["observation_id"], before["observation_id"] + 1)
        self.assertAlmostEqual(runner.observations[state["observation_id"]], 1.05)
        np.testing.assert_array_equal(state["q"], runner.plant.q())
        self.assertAlmostEqual(runner.sim_time, 0.02)
        self.assertIsNone(runner.command(hold(runner, state, 1), 1.06))

    def test_trace_records_each_step(self):
        runner = Runner(record_trace=True)
        self.assertEqual(len(runner.trace_qpos), 1)
        runner.step(0.01)
        self.assertEqual(len(runner.trace_qpos), 2)
        self.assertEqual(runner.trace_qpos[-1].shape, (runner.plant.model.nq,))

    def test_live_file_follows_each_step(self):
        import json
        import os
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "live.jsonl"
            os.environ["G1_SIM_LIVE"] = str(path)
            try:
                runner = Runner()
                runner.step(0.01)
                self.assertEqual(runner.trace_qpos, [])
                runner.report()
            finally:
                os.environ.pop("G1_SIM_LIVE", None)
            lines = path.read_text().splitlines()
        self.assertEqual(len(lines), 2)
        frame = json.loads(lines[-1])
        self.assertEqual(len(frame["qpos"]), runner.plant.model.nq)
        self.assertGreater(frame["t"], 0)

    def test_replay_page_includes_the_g1(self):
        import os
        import tempfile

        from view import export_replay

        runner = Runner(record_trace=True)
        runner.step(0.01)
        with tempfile.TemporaryDirectory() as tmp:
            trace = Path(tmp) / "trace.npz"
            os.environ["G1_SIM_TRACE"] = str(trace)
            try:
                runner.report()
                dest = export_replay(trace, Path(tmp) / "view")
                self.assertTrue((dest / "index.html").is_file())
                self.assertTrue((dest / "motion.json").is_file())
                self.assertTrue(any((dest / "meshes").glob("*.stl")))
            finally:
                os.environ.pop("G1_SIM_TRACE", None)

    def test_fallen_pose_detected(self):
        runner = Runner()
        runner.plant.data.qpos[2] = 0.1
        runner.plant.data.qpos[3:7] = [0.0, 1.0, 0.0, 0.0]
        mujoco.mj_forward(runner.plant.model, runner.plant.data)
        self.assertTrue(runner.plant.fallen())
        self.assertEqual(runner.report()["pose"], "standing")
        self.assertFalse(runner.report()["fall_check"])
        self.assertTrue(runner.report()["checks"]["standing_posture"])

    def test_unknown_pose_rejected(self):
        with self.assertRaises(ValueError):
            Plant("ceiling")

    def test_counterfactual_poses_hold_without_a_fall(self):
        for pose in POSES:
            if pose == "standing":
                continue
            runner = Runner(pose)
            plant = runner.plant
            self.assertFalse(plant.check_fall)
            self.assertFalse(plant.fallen())
            self.assertTrue(all(c.dist >= -0.005 for c in plant.data.contact))
            q = plant.q()
            low, high = plant.model.jnt_range[plant.joints].T
            self.assertTrue(((q >= low) & (q <= high)).all())
            if pose in ("left_side", "right_side"):
                side = plant.data.xmat[plant.base].reshape(3, 3)[2, 1]
                self.assertAlmostEqual(
                    side, -1.0 if pose == "left_side" else 1.0, places=3
                )
            target = None
            for step in range(25):
                state = runner.feedback(step * 0.02)
                if target is None:
                    target = list(state["q"])
                reason = runner.command(
                    hold(runner, state, step, q=target), step * 0.02
                )
                if step == 0:
                    self.assertIsNone(reason, pose)
                if reason:
                    self.assertNotEqual(reason, "fall", pose)
                    break
                stepped = runner.step(step * 0.02 + 0.01)
                self.assertNotEqual(stepped["reason"], "fall", pose)
            report = runner.report()
            self.assertEqual(report["pose"], pose)
            self.assertFalse(report["fall_check"])
            self.assertNotEqual(report["reason"], "fall")

    def test_push_is_applied_only_inside_its_window(self):
        plant = Plant("standing")
        plant.set_push(
            {"body": "pelvis", "force": [10.0, 0.0, 0.0], "start": 0.02, "duration": 0.02}
        )
        plant.step_physics()
        self.assertEqual(float(plant.data.xfrc_applied[plant.base, 0]), 0.0)
        plant.data.time = 0.02
        plant.step_physics()
        self.assertEqual(float(plant.data.xfrc_applied[plant.base, 0]), 10.0)
        plant.data.time = 0.04
        plant.step_physics()
        self.assertEqual(float(plant.data.xfrc_applied[plant.base, 0]), 0.0)

    def test_named_push_writes_one_report_case(self):
        from battery import run_battery

        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(SystemExit):
                run_battery(tmp, poses=["standing"], push="sideways")
            summary = run_battery(tmp, 0.2, poses=["seated"], push="none")
            self.assertTrue((Path(tmp) / "report.json").is_file())
        self.assertEqual(len(summary["cases"]), 1)
        self.assertEqual(summary["cases"][0]["pose"], "seated")
        self.assertEqual(summary["cases"][0]["push"], "none")
        self.assertEqual(summary["status"], summary["cases"][0]["status"])

    def test_example_command_files(self):
        from circle import load_command

        stand = load_command(ROOT / "containers/examples/commands/walker/stand.yaml")
        circle = load_command(ROOT / "containers/examples/commands/walker/circle.yaml")
        self.assertTrue(np.allclose(stand, [0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(circle, [0.45, 0.0, 0.9]))

    def test_forward_stop_command_holds_then_zeros(self):
        from circle import load_schedule

        schedule = load_schedule(ROOT / "containers/examples/commands/walker/forward_stop.yaml")
        self.assertTrue(np.allclose(schedule.at(0.0), [0.50, 0.0, 0.0]))
        self.assertTrue(np.allclose(schedule.at(1.99), [0.50, 0.0, 0.0]))
        for step, command in enumerate((
            [0.40, 0.0, 0.0],
            [0.30, 0.0, 0.0],
            [0.20, 0.0, 0.0],
            [0.10, 0.0, 0.0],
        )):
            self.assertTrue(np.allclose(schedule.at(2.0 + step), command))
        self.assertTrue(np.allclose(schedule.at(6.0), [0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(schedule.at(7.0), [0.0, 0.0, 0.0]))

    def test_circle_battery_uses_the_walker_gait(self):
        model = ROOT / "containers/examples/policies/walker/walker.onnx"
        if not model.is_file():
            self.skipTest("cached walker weights are not on this machine")
        from battery import run_case

        report = run_case("standing", None, 0.2, circle=True)
        self.assertEqual(report["status"], "completed")
        self.assertFalse(report["checks"]["command_limits"])
        self.assertGreaterEqual(report["commands"], 5)
        self.assertGreater(report["motion"]["final_position"][2], 0.5)

    def test_flat_ground_pose_push_uses_the_plant(self):
        from battery import run_case

        report = run_case(
            "standing",
            {"body": "pelvis", "force": [10.0, 0.0, 0.0], "start": 0.02, "duration": 0.02},
            0.2,
        )
        self.assertEqual(report["pose"], "standing")
        self.assertGreaterEqual(report["commands"], 1)
        self.assertIn(report["status"], ("within_limits", "violation"))

    def test_standing_fall_still_latches(self):
        runner = Runner("standing")
        state = runner.feedback(0.0)
        self.assertIsNone(runner.command(hold(runner, state, 0), 0.0))
        runner.plant.data.qpos[2] = 0.1
        mujoco.mj_forward(runner.plant.model, runner.plant.data)
        runner.step(0.01)
        self.assertIsNotNone(runner.reason)
        self.assertEqual(runner.hardware.snapshot()["phase"], "damping")

    def test_torn_live_line_resyncs_instead_of_raising(self):
        from live import LiveSession

        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            live = directory / "live.jsonl"
            session = LiveSession(
                directory, live, directory / "start", directory / "report.json",
                "standing", None,
            )
            zeros = ",".join(["0"] * session.model.nq)
            live.write_text('{"t": 0.0, "qpos": [' + zeros + "]}\n, \"qpos\"\n")
            body = session.snapshot(0)
            self.assertTrue(body["reset"])
            self.assertEqual(body["frames"], [])
            live.write_text('{"t": 0.02, "qpos": [' + zeros + "]}\n")
            body = session.snapshot(0)
            self.assertEqual(len(body["frames"]), 1)
            self.assertAlmostEqual(body["frames"][0]["time"], 0.02)


if __name__ == "__main__":
    unittest.main()
