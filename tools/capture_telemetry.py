"""Passively capture serialized DDS samples without motor publishers."""

import argparse
import json
import os
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

TRACE_BYTE_LIMIT = 64 * 1024 * 1024


def trace_cursor(path):
    """Remember the existing trace without reading its old contents."""
    if path is None:
        return None
    try:
        state = path.stat()
        return {"path": str(path.resolve()), "inode": state.st_ino,
                "offset": state.st_size, "mtime_ns": state.st_mtime_ns}
    except FileNotFoundError:
        return {"path": str(path.resolve()), "inode": None, "offset": 0}


def copy_trace_window(cursor, destination):
    """Copy bounded appended bytes and disclose rotation or truncation."""
    if cursor is None:
        return None
    result = dict(cursor)
    path = Path(cursor["path"])
    try:
        with path.open("rb") as source:
            state = os.fstat(source.fileno())
            reset = state.st_ino != cursor["inode"] or state.st_size < cursor["offset"]
            prefix = b""
            prior_available = 0
            recovered_rotation = False
            missing_prior_bytes = reset and cursor["inode"] is not None
            if state.st_ino != cursor["inode"] and cursor["inode"] is not None:
                backup = Path(str(path) + ".1")
                try:
                    with backup.open("rb") as previous:
                        old = os.fstat(previous.fileno())
                        if old.st_ino == cursor["inode"] and old.st_size >= cursor["offset"]:
                            prior_available = old.st_size - cursor["offset"]
                            previous.seek(cursor["offset"])
                            prefix = previous.read(min(prior_available, TRACE_BYTE_LIMIT))
                            recovered_rotation = True
                            missing_prior_bytes = False
                except FileNotFoundError:
                    pass
            offset = 0 if reset else cursor["offset"]
            available = state.st_size - offset
            source.seek(offset)
            current = source.read(min(available, TRACE_BYTE_LIMIT - len(prefix)))
            data = prefix + current
        destination.write_bytes(data)
        result.update(copied_bytes=len(data), end_offset=offset + len(current),
                      recovered_single_rotation=recovered_rotation,
                      recovered_rotated_bytes=len(prefix), missing_prior_bytes=missing_prior_bytes,
                      reset_or_created=reset,
                      byte_limit_exceeded=prior_available + available > TRACE_BYTE_LIMIT,
                      output=destination.name)
    except OSError as error:
        result["error"] = str(error)
    return result


def lowstate_preflight(reader, take, timeout=3.0):
    """Require newly arriving valid state samples before the test starts."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        batch = take(reader._ref, 256)
        if isinstance(batch, int):
            raise RuntimeError(f"Lowstate preflight DDS error code {batch}")  # noqa: TRY004
        count = sum(bool(info.valid_data) for _, info in batch)
        if count:
            return {"valid_samples": count, "observed_monotonic_ns": time.monotonic_ns()}
        time.sleep(0.01)
    raise RuntimeError("No live rt/lowstate received within 3s. Check interface and DDS domain before testing.")


def diagnostic_preflight(path, max_age):
    """Reject missing or stale diagnostic traces before an instrumented test."""
    if path is None:
        raise ValueError("--require-diagnostics needs --diagnostic-trace PATH")
    try:
        state = path.stat()
    except OSError as error:
        raise ValueError("Diagnostic trace is unavailable: " + str(error)) from error
    age = (time.time_ns() - state.st_mtime_ns) / 1e9
    if state.st_size == 0 or age < 0 or age > max_age:
        raise ValueError(f"Diagnostic trace is empty or stale ({age:.1f}s). "
                         "Enable tracing in the running harness before this test.")
    return {"path": str(path.resolve()), "age_s": age, "size_bytes": state.st_size}


def copy_diagnostic_metadata(path, destination):
    """Preserve startup attribution even when the trace has rotated."""
    if path is None:
        return None
    candidates = (path.with_suffix(".meta.json"), Path(str(path) + ".meta.json"))
    for candidate in candidates:
        if candidate.exists():
            try:
                with candidate.open("rb") as stream:
                    data = stream.read(1024 * 1024 + 1)
                if len(data) > 1024 * 1024:
                    return {"error": "Diagnostic metadata exceeds 1 MiB", "path": str(candidate)}
                json.loads(data)
                destination.write_bytes(data)
                return {"path": str(candidate.resolve()), "output": destination.name}
            except (OSError, ValueError) as error:
                return {"error": str(error), "path": str(candidate)}
    try:
        with path.open("rb") as stream:
            first_line = stream.readline(1024 * 1024)
        first = json.loads(first_line)
        if "diagnostic_start" in (first.get("event"), first.get("kind"), first.get("type")):
            destination.write_bytes(first_line)
            return {"path": str(path.resolve()), "output": destination.name, "from_first_record": True}
    except (OSError, ValueError):
        pass
    return {"error": "Diagnostic startup metadata unavailable"}


def ros_worker(output, seconds):
    """Record status and abort receipts in an isolated ROS observer process."""
    metadata = {"ready": False, "counts": {}, "error": None}
    metadata_path = output / "ros-observer.json"
    node = None
    try:
        import rclpy
        from him_safety_interfaces.msg import Abort
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        rclpy.init(args=[])
        node = rclpy.create_node("passive_telemetry_" + str(os.getpid()),
                                 enable_rosout=False, start_parameter_services=False)
        stopped = False

        def stop(signum, frame):
            nonlocal stopped
            stopped = True

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        with (output / "ros-events.jsonl").open("w") as stream:
            def callback(topic, message):
                row = {"topic": topic, "receive_monotonic_ns": time.monotonic_ns(),
                       "receive_wall_ns": time.time_ns()}
                if topic == "/safety/hardware_status":
                    try:
                        row["status"] = json.loads(message.data)
                    except ValueError:
                        row["raw_data"] = message.data
                else:
                    row.update(source=message.source, reason=message.reason)
                stream.write(json.dumps(row, separators=(",", ":")) + "\n")
                stream.flush()
                metadata["counts"][topic] = metadata["counts"].get(topic, 0) + 1
                if metadata["counts"][topic] == 1:
                    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")

            subscriptions = []
            def subscriber_callback(topic):
                def receive(message):
                    callback(topic, message)
                return receive

            for topic, message_type, durability in (
                ("/safety/hardware_status", String, DurabilityPolicy.VOLATILE),
                ("/safety/abort", Abort, DurabilityPolicy.TRANSIENT_LOCAL),
            ):
                qos = QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=durability)
                subscriptions.append(node.create_subscription(
                    message_type, topic, subscriber_callback(topic), qos))
            metadata.update(ready=True, start_monotonic_ns=time.monotonic_ns(),
                            domain=os.environ.get("ROS_DOMAIN_ID"),
                            method="Passive best-effort subscriptions. Callback receipt times are not transition times. Abort receipts may be retained or repeated old faults.")
            metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
            deadline = time.monotonic() + seconds
            while not stopped and rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as error:  # noqa: BLE001 - preserve observer startup failures
        metadata["error"] = type(error).__name__ + ": " + str(error)
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        metadata["end_monotonic_ns"] = time.monotonic_ns()
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
    return 1 if metadata["error"] else 0


def start_ros_observer(args, output):
    """Start the optional observer and fail before capture if imports fail."""
    if args.ros_domain is None:
        return None
    # ROS status uses Fast DDS, independently of the Unitree CycloneDDS reader.
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.ros_domain), ROS_LOCALHOST_ONLY="1",
               RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    env.pop("CYCLONEDDS_URI", None)
    command = [args.ros_python, str(Path(__file__).resolve()), "--ros-worker",
               "--output", str(output.resolve()), "--seconds", str(args.seconds + 15)]
    with (output / "ros-observer.stderr").open("w") as stderr:
        process = subprocess.Popen(command, env=env, stdout=subprocess.DEVNULL, stderr=stderr)
    deadline = time.monotonic() + 10
    observer_error = None
    while time.monotonic() < deadline:
        metadata_path = output / "ros-observer.json"
        if metadata_path.exists():
            try:
                metadata = json.loads(metadata_path.read_text())
            except ValueError:
                continue
            if (metadata.get("ready") and not metadata.get("error")
                    and metadata["counts"].get("/safety/hardware_status", 0)):
                return process
            if metadata.get("error"):
                observer_error = metadata["error"]
                break
        if process.poll() is not None:
            break
        time.sleep(0.05)
    stop_ros_observer(process)
    if observer_error:
        raise RuntimeError("ROS observer failed: " + observer_error
                           + ". See " + str(output / "ros-observer.stderr"))
    raise RuntimeError("ROS observer did not receive hardware status. Check the domain and source "
                       "the ROS and safety install setup.bash files, then inspect "
                       + str(output / "ros-observer.json"))


def stop_ros_observer(process):
    """Stop the observer without signalling the robot processes."""
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=1)


def stats(values):
    values = sorted(values)
    if not values:
        return None
    return {
        "min": values[0], "median": statistics.median(values),
        "p95": values[int((len(values) - 1) * .95)],
        "p99": values[int((len(values) - 1) * .99)], "max": values[-1],
    }


def main():
    """Run the existing passive topic capture with CLI options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30, help="Capture duration, 1 to 60 seconds")
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--label", default="holosoma-balance", help="Operator description, not verified controller state")
    parser.add_argument("--output", type=Path, help="New output directory, must not exist")
    parser.add_argument("--ros-domain", type=int, help="Also observe ROS status/abort, hardware service uses 97")
    parser.add_argument("--ros-python", default="/usr/bin/python3", help="Python with ROS and safety messages installed")
    parser.add_argument("--control-trace", type=Path, help="Copy appended existing control trace")
    parser.add_argument("--diagnostic-trace", type=Path, help="Copy appended detailed harness trace")
    parser.add_argument("--require-diagnostics", action="store_true", help="Refuse capture without a fresh detailed trace")
    parser.add_argument("--diagnostic-max-age", type=float, default=5, help="Maximum diagnostic trace age in seconds")
    parser.add_argument("--ros-worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.ros_worker:
        if args.output is None or not 1 <= args.seconds <= 75:
            parser.error("Invalid internal observer arguments")
        sys.exit(ros_worker(args.output, args.seconds))
    if not 1 <= args.seconds <= 60:
        parser.error("--seconds must be between 1 and 60 to bound memory use")
    if args.diagnostic_max_age <= 0:
        parser.error("--diagnostic-max-age must be positive")
    diagnostic_ready = None
    if args.require_diagnostics:
        try:
            diagnostic_ready = diagnostic_preflight(args.diagnostic_trace, args.diagnostic_max_age)
        except ValueError as error:
            parser.error(str(error))
    from cyclonedds.sub import ddspy_take
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

    from robot_safety.sdk_latest import _new_reader

    os.nice(10)
    out = args.output or (Path(__file__).resolve().parents[1] / ".cache/topic-profile" /
                          datetime.now(timezone.utc).strftime("live-%Y%m%dT%H%M%S-%fZ"))
    out.mkdir(parents=True, exist_ok=False)
    (out / "context.json").write_text(json.dumps({
        "operator_label": args.label, "requested_seconds": args.seconds,
        "interface": args.interface, "domain": args.domain,
        "host_boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        "ros_domain": args.ros_domain, "diagnostic_preflight": diagnostic_ready,
        "clock_notes": "Host monotonic clocks correlate local processes only. DDS source clocks are not verified synchronized. ROS status receipts can lag transitions. Lowstate tick association is observer-side, not the controller input.",
        "source_recorder": ".cache/topic-profile/soft-reset-20261021T072827Z/capture.py",
        "method": "Passive DDS readers only. No motor commands or RPCs.",
    }, indent=2) + "\n")
    print("Output: " + str(out.resolve()), flush=True)
    ChannelFactoryInitialize(args.domain, args.interface)
    readers = {
        "rt/lowstate": (_new_reader("rt/lowstate", LowState_, 256), LowState_),
        "rt/lowcmd": (_new_reader("rt/lowcmd", LowCmd_, 256), LowCmd_),
    }
    time.sleep(1)
    ros_process = start_ros_observer(args, out)
    cursors = {"control": trace_cursor(args.control_trace),
               "diagnostic": trace_cursor(args.diagnostic_trace)}
    diagnostic_metadata = copy_diagnostic_metadata(args.diagnostic_trace, out / "diagnostic-metadata.json")
    for reader, _ in readers.values():
        ddspy_take(reader._ref, 256)
    preflight = None
    if args.require_diagnostics:
        try:
            preflight = {"lowstate": lowstate_preflight(readers["rt/lowstate"][0], ddspy_take),
                         "diagnostics": diagnostic_preflight(args.diagnostic_trace, args.diagnostic_max_age)}
        except (RuntimeError, ValueError) as error:
            stop_ros_observer(ros_process)
            parser.error(str(error))
        (out / "preflight.json").write_text(json.dumps(preflight, indent=2) + "\n")
    records = []
    counts = {topic: 0 for topic in readers}
    previous_counts = counts.copy()
    interrupted = False

    def stop_capture(signum, frame):
        nonlocal interrupted
        interrupted = True

    old_handler = signal.signal(signal.SIGINT, stop_capture)
    print(f"CAPTURING for {args.seconds:g}s. Ctrl+C ends capture and saves. Decode runs afterward.", flush=True)
    start = time.monotonic_ns()
    start_wall = time.time_ns()
    start_cpu = time.process_time()
    last_report = start
    capture_error = None
    try:
        while not interrupted and time.monotonic_ns() - start < args.seconds * 1e9:
            for topic, (reader, _) in readers.items():
                batch = ddspy_take(reader._ref, 256)
                mono, wall = time.monotonic_ns(), time.time_ns()
                if isinstance(batch, int):
                    raise RuntimeError(f"DDS take returned error code {batch}")  # noqa: TRY004
                for payload, info in batch:
                    if info.valid_data:
                        records.append((topic, mono, wall, payload, info))
                        counts[topic] += 1
            now = time.monotonic_ns()
            if now - last_report >= 1_000_000_000:
                elapsed = (now - last_report) / 1e9
                rates = [f"{topic}: {(counts[topic] - previous_counts[topic]) / elapsed:.0f} msg/s ({counts[topic]} total)"
                    for topic in readers]
                print("{:5.1f}s | {}".format((now - start) / 1e9, " | ".join(rates)), flush=True)
                previous_counts = counts.copy()
                last_report = now
            time.sleep(.001)
    except Exception as error:  # noqa: BLE001 - save partial evidence after reader failures
        capture_error = type(error).__name__ + ": " + str(error)
    end = time.monotonic_ns()
    end_wall = time.time_ns()
    capture_cpu = time.process_time() - start_cpu
    stop_ros_observer(ros_process)
    trace_flush_grace = 1.1 if any(cursors.values()) else 0
    if trace_flush_grace:
        print("DDS capture stopped. Allowing 1.1s for asynchronous diagnostic writes.", flush=True)
        time.sleep(trace_flush_grace)
    trace_copies = {name: copy_trace_window(cursor, out / (name + "-trace.jsonl"))
                    for name, cursor in cursors.items()}
    trace_copy_end = time.monotonic_ns()
    print(f"Capture stopped. Decoding and saving {len(records)} samples. Please wait.", flush=True)
    rows = []
    last_state = None
    for topic, mono, wall, payload, info in records:
        sample = readers[topic][1].deserialize(payload)
        row = {
            "topic": topic, "take_monotonic_ns": mono, "take_wall_ns": wall,
            "source_timestamp_ns": info.source_timestamp,
            "publication_handle": info.publication_handle,
        }
        if topic == "rt/lowstate":
            row["tick"] = sample.tick
            row["mode_machine"] = sample.mode_machine
            row["q"] = [m.q for m in sample.motor_state[:29]]
            row["dq"] = [m.dq for m in sample.motor_state[:29]]
            row["tau_est"] = [m.tau_est for m in sample.motor_state[:29]]
            row["motor_temperature"] = [m.temperature for m in sample.motor_state[:29]]
            row["gyro"] = list(sample.imu_state.gyroscope)
            acceleration = getattr(sample.imu_state, "accelerometer", None)
            row["accelerometer"] = None if acceleration is None else list(acceleration)
            row["rpy"] = list(sample.imu_state.rpy)
            row["quaternion"] = list(sample.imu_state.quaternion)
            row["motor_mode"] = [m.mode for m in sample.motor_state[:29]]
            last_state = (sample.tick, mono)
        else:
            row["q"] = [m.q for m in sample.motor_cmd[:29]]
            row["kp"] = [m.kp for m in sample.motor_cmd[:29]]
            row["kd"] = [m.kd for m in sample.motor_cmd[:29]]
            row["tau"] = [m.tau for m in sample.motor_cmd[:29]]
            row["dq"] = [m.dq for m in sample.motor_cmd[:29]]
            row["observer_latest_lowstate_tick"] = None if last_state is None else last_state[0]
            row["observer_lowstate_take_age_ms"] = None if last_state is None else (mono - last_state[1]) / 1e6
        rows.append(row)
    with (out / "samples.jsonl").open("w") as stream:
        for row in rows:
            stream.write(json.dumps(row, separators=(",", ":")) + "\n")
    summary = {
        "start_wall_ns": start_wall, "duration_s": (end - start) / 1e9,
        "end_wall_ns": end_wall, "start_monotonic_ns": start, "end_monotonic_ns": end,
        "capture_error": capture_error, "trace_copies": trace_copies,
        "trace_flush_grace_s": trace_flush_grace, "trace_copy_end_monotonic_ns": trace_copy_end,
        "trace_window_note": "Appended traces include observer setup and post-capture flush grace. Filter their timestamps against start/end_monotonic_ns for the DDS capture window.",
        "diagnostic_metadata": diagnostic_metadata,
        "preflight": preflight,
        "capture_cpu_s": capture_cpu, "interrupted": interrupted, "operator_label": args.label,
        "method": "Independent DDS readers, history depth 256, raw serialized take at about 1 ms. Decode after capture. No publishers. Receive times are userspace take completion, not NIC arrival. Source clocks may be unsynchronized.",
        "topics": {},
    }
    if args.ros_domain is not None:
        summary["ros_observer"] = json.loads((out / "ros-observer.json").read_text())
    for topic in readers:
        group = [r for r in rows if r["topic"] == topic]
        source = [r["source_timestamp_ns"] for r in group]
        times = [r["take_monotonic_ns"] for r in group]
        result = {
            "samples": len(group), "rate_per_window_hz": len(group) / summary["duration_s"],
            "publishers": sorted({r["publication_handle"] for r in group}),
            "source_interval_ms": stats([(b - a) / 1e6 for a, b in zip(source, source[1:])]),
            "take_interval_ms": stats([(b - a) / 1e6 for a, b in zip(times, times[1:])]),
            "take_wall_minus_source_ms_unvalidated_clock": stats([(r["take_wall_ns"] - r["source_timestamp_ns"]) / 1e6 for r in group]),
        }
        result["per_publisher"] = {}
        for publisher in result["publishers"]:
            samples = [row for row in group if row["publication_handle"] == publisher]
            source_times = [row["source_timestamp_ns"] for row in samples]
            result["per_publisher"][str(publisher)] = {
                "samples": len(samples),
                "first_take_monotonic_ns": samples[0]["take_monotonic_ns"],
                "last_take_monotonic_ns": samples[-1]["take_monotonic_ns"],
                "source_interval_ms": stats([(b - a) / 1e6 for a, b in zip(source_times, source_times[1:])]),
            }
        if group and topic == "rt/lowstate":
            ticks = [r["tick"] for r in group]
            result["tick_increment"] = stats([b - a for a, b in zip(ticks, ticks[1:])])
            result["missing_ticks_if_1_per_sample"] = sum(max(0, b - a - 1) for a, b in zip(ticks, ticks[1:]))
        if group and topic == "rt/lowcmd":
            result["command_change_counts"] = {
                key: sum(a[key] != b[key] for a, b in zip(group, group[1:]))
                for key in ("q", "dq", "kp", "kd", "tau")
            }
            result["knee_kp_median"] = {
                name: statistics.median(r["kp"][index] for r in group)
                for name, index in (("left", 3), ("right", 9))
            }
        if group and topic == "rt/lowstate":
            result["knees"] = {
                name: {
                    "tau_est_median_Nm": statistics.median(r["tau_est"][index] for r in group),
                    "tau_est_abs_max_Nm": max(abs(r["tau_est"][index]) for r in group),
                    "temperature_start_C": group[0]["motor_temperature"][index],
                    "temperature_end_C": group[-1]["motor_temperature"][index],
                } for name, index in (("left", 3), ("right", 9))
            }
        summary["topics"][topic] = result
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)

    signal.signal(signal.SIGINT, old_handler)
    print("Saved samples.jsonl, summary.json and context.json in " + str(out.resolve()), flush=True)
    if capture_error:
        sys.exit("Capture ended early: " + capture_error)


if __name__ == "__main__":
    main()
