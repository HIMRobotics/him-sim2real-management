"""Offline checks for passive telemetry capture metadata."""

import io
import json
import os
import signal
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from tools.capture_telemetry import (
    copy_diagnostic_metadata,
    copy_trace_window,
    diagnostic_preflight,
    lowstate_preflight,
    main,
    ros_worker,
    start_ros_observer,
    trace_cursor,
)


class CaptureTelemetryTests(unittest.TestCase):
    def test_ros_observer_isolates_inherited_cyclone_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / 'ros-observer.json').write_text(json.dumps({
                'ready': True, 'counts': {'/safety/hardware_status': 1}, 'error': None,
            }))
            args = SimpleNamespace(ros_domain=97, ros_python='/usr/bin/python3', seconds=1)
            with patch.dict(os.environ, RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                            CYCLONEDDS_URI='incompatible-sdk-config'), \
                    patch('tools.capture_telemetry.subprocess.Popen') as launch:
                result = start_ros_observer(args, output)
                self.assertIs(result, launch.return_value)
                env = launch.call_args.kwargs['env']
                self.assertEqual(env['RMW_IMPLEMENTATION'], 'rmw_fastrtps_cpp')
                self.assertNotIn('CYCLONEDDS_URI', env)
                self.assertEqual(os.environ['CYCLONEDDS_URI'], 'incompatible-sdk-config')

    def test_ros_startup_error_is_reported_directly(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / 'ros-observer.json').write_text(json.dumps({
                'ready': False, 'counts': {}, 'error': 'RCLError: domain creation failed',
            }))
            args = SimpleNamespace(ros_domain=97, ros_python='/usr/bin/python3', seconds=1)
            with patch('tools.capture_telemetry.subprocess.Popen'), \
                    self.assertRaisesRegex(RuntimeError, 'RCLError: domain creation failed'):
                start_ros_observer(args, output)

    def test_trace_copy_recovers_single_rotation_in_order(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            path.write_text("before-capture\n")
            cursor = trace_cursor(path)
            with path.open("a") as stream:
                stream.write("captured-before-rotation\n")
            path.rename(Path(str(path) + ".1"))
            path.write_text("captured-after-rotation\n")
            destination = Path(directory) / "copy.jsonl"
            result = copy_trace_window(cursor, destination)
            self.assertEqual(destination.read_text(), "captured-before-rotation\ncaptured-after-rotation\n")
            self.assertTrue(result["recovered_single_rotation"])
            self.assertFalse(result["missing_prior_bytes"])
            with patch("tools.capture_telemetry.TRACE_BYTE_LIMIT", 28):
                bounded = copy_trace_window(cursor, destination)
            self.assertEqual(destination.stat().st_size, 28)
            self.assertTrue(bounded["byte_limit_exceeded"])

    def test_lowstate_preflight_waits_for_valid_sample(self):
        batches = [[], [(b"", SimpleNamespace(valid_data=False))],
                   [(b"state", SimpleNamespace(valid_data=True))]]
        with patch("tools.capture_telemetry.time.sleep"):
            result = lowstate_preflight(SimpleNamespace(_ref="state"), lambda ref, depth: batches.pop(0))
        self.assertEqual(result["valid_samples"], 1)

    def test_lowstate_preflight_rejects_absent_feedback(self):
        with patch("tools.capture_telemetry.time.monotonic", side_effect=[0.0, 0.0, 4.0]), \
                patch("tools.capture_telemetry.time.sleep"), \
                self.assertRaisesRegex(RuntimeError, "No live rt/lowstate"):
            lowstate_preflight(SimpleNamespace(_ref="state"), lambda ref, depth: [])

    def test_reader_failure_saves_partial_samples_and_publisher_intervals(self):
        takes = {}
        motor = SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0, temperature=[25, 25], mode=1,
                                kp=20.0, kd=1.0, tau=0.0)
        sample = SimpleNamespace(tick=100, mode_machine=5, motor_state=[motor] * 29,
                                 motor_cmd=[motor] * 29, imu_state=SimpleNamespace(
                                     gyroscope=[0.0] * 3, rpy=[0.0] * 3,
                                     quaternion=[1.0, 0.0, 0.0, 0.0]))
        sample_type = SimpleNamespace(deserialize=lambda payload: sample)

        def take(topic, depth):
            takes[topic] = takes.get(topic, 0) + 1
            if takes[topic] == 1:
                return []
            if takes[topic] == 3:
                return -1
            return [(b"sample", SimpleNamespace(valid_data=True, source_timestamp=stamp,
                                                 publication_handle=handle))
                    for stamp, handle in ((10_000_000, 1), (12_000_000, 1), (200_000_000, 2))]

        modules = {
            "robot_safety.sdk_latest": SimpleNamespace(_new_reader=lambda topic, kind, depth: SimpleNamespace(_ref=topic)),
            "cyclonedds.sub": SimpleNamespace(ddspy_take=take),
            "unitree_sdk2py.core.channel": SimpleNamespace(ChannelFactoryInitialize=lambda domain, interface: None),
            "unitree_sdk2py.idl.unitree_hg.msg.dds_": SimpleNamespace(LowCmd_=sample_type, LowState_=sample_type),
        }
        with tempfile.TemporaryDirectory() as directory, patch.dict("sys.modules", modules):
            output = Path(directory) / "capture"
            with patch("sys.argv", ["capture", "--seconds", "1", "--output", str(output)]), \
                    patch("tools.capture_telemetry.os.nice"), patch("tools.capture_telemetry.time.sleep"), \
                    patch("sys.stdout", new_callable=io.StringIO), \
                    self.assertRaisesRegex(SystemExit, "DDS take returned error code"):
                main()
            rows = [json.loads(line) for line in (output / "samples.jsonl").read_text().splitlines()]
            summary = json.loads((output / "summary.json").read_text())
            self.assertEqual(len(rows), 6)
            self.assertEqual(rows[-1]["observer_latest_lowstate_tick"], 100)
            publisher = summary["topics"]["rt/lowcmd"]["per_publisher"]["1"]
            self.assertEqual(publisher["source_interval_ms"]["median"], 2.0)

    def test_diagnostic_metadata_survives_trace_rotation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            path.write_text('{"event":"heartbeat"}\n')
            path.with_suffix(".meta.json").write_text('{"event":"diagnostic_start","model_hash":"abc"}\n')
            destination = Path(directory) / "metadata.json"
            result = copy_diagnostic_metadata(path, destination)
            self.assertEqual(json.loads(destination.read_text())["model_hash"], "abc")
            self.assertEqual(result["output"], "metadata.json")

    def test_trace_copy_excludes_previous_run(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            path.write_text('{"old":1}\n')
            cursor = trace_cursor(path)
            with path.open("a") as stream:
                stream.write('{"new":2}\n')
            destination = Path(directory) / "copy.jsonl"
            result = copy_trace_window(cursor, destination)
            self.assertEqual(destination.read_text(), '{"new":2}\n')
            self.assertFalse(result["reset_or_created"])

    def test_rotation_and_byte_limit_are_disclosed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            path.write_text("old\n")
            cursor = trace_cursor(path)
            path.rename(path.with_suffix(".old"))
            path.write_text("replacement\n")
            destination = Path(directory) / "copy.jsonl"
            with patch("tools.capture_telemetry.TRACE_BYTE_LIMIT", 5):
                result = copy_trace_window(cursor, destination)
            self.assertEqual(destination.read_text(), "repla")
            self.assertTrue(result["reset_or_created"])
            self.assertTrue(result["byte_limit_exceeded"])
            self.assertTrue(result["missing_prior_bytes"])

    def test_preflight_rejects_missing_empty_and_stale_trace(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            for candidate in (None, path):
                with self.assertRaises(ValueError):
                    diagnostic_preflight(candidate, 5)
            path.touch()
            with self.assertRaises(ValueError):
                diagnostic_preflight(path, 5)
            path.write_text('{"event":"heartbeat"}\n')
            self.assertGreater(diagnostic_preflight(path, 5)["size_bytes"], 0)
            old = time.time() - 60
            os.utime(path, (old, old))
            with self.assertRaises(ValueError):
                diagnostic_preflight(path, 5)

    def test_ros_observer_records_status_and_abort_without_robot_apis(self):
        callbacks = {}
        spun = []
        node = SimpleNamespace(
            create_subscription=lambda message_type, topic, callback, qos: callbacks.setdefault(topic, callback),
            destroy_node=lambda: None,
        )

        def spin_once(node, timeout_sec):
            callbacks["/safety/hardware_status"](SimpleNamespace(data='{"automatic_soft_reset":true}'))
            callbacks["/safety/abort"](SimpleNamespace(source="checker", reason="joint_limit"))
            spun.append(True)

        modules = {
            "rclpy": SimpleNamespace(init=lambda args: None, create_node=lambda *args, **kwargs: node,
                                     ok=lambda: not spun, spin_once=spin_once, shutdown=lambda: None),
            "rclpy.qos": SimpleNamespace(
                DurabilityPolicy=SimpleNamespace(VOLATILE=0, TRANSIENT_LOCAL=1),
                ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=0),
                QoSProfile=lambda **kwargs: kwargs),
            "std_msgs.msg": SimpleNamespace(String=object),
            "him_safety_interfaces.msg": SimpleNamespace(Abort=object),
        }
        with tempfile.TemporaryDirectory() as directory, patch.dict("sys.modules", modules):
            old_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                self.assertEqual(ros_worker(Path(directory), 1), 0)
            finally:
                for sig, handler in old_handlers.items():
                    signal.signal(sig, handler)
            rows = [json.loads(line) for line in (Path(directory) / "ros-events.jsonl").read_text().splitlines()]
            metadata = json.loads((Path(directory) / "ros-observer.json").read_text())
            self.assertTrue(rows[0]["status"]["automatic_soft_reset"])
            self.assertEqual(rows[1]["reason"], "joint_limit")
            self.assertGreater(rows[0]["receive_monotonic_ns"], 0)
            self.assertEqual(metadata["counts"]["/safety/hardware_status"], 1)


if __name__ == "__main__":
    unittest.main()
