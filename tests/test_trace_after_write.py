"""The control-trace line follows the SDK write without changing what it records."""

import json

from robot_safety.control_trace import ControlTrace
from robot_safety.hardware_adapter import G1HardwareAdapter
from tests.test_robot_safety_hardware import FakeTransport, load_profile, profile_data


def make_adapter(tmp_path):
    clock = {"now": 10.0}
    adapter = G1HardwareAdapter(load_profile(profile_data()), lambda: clock["now"])
    adapter.transport = FakeTransport()
    adapter.receive_battery(type("B", (), {"soc": 80})())
    adapter.trace = ControlTrace(tmp_path / "trace.jsonl")
    adapter.trace.open()
    adapter.receive_state(type("S", (), {
        "motor_state": [type("M", (), {"q": 0.0, "dq": 0.0, "temperature": (25, 25),
                                       "tau_est": 0.0})() for _ in range(35)],
        "mode_machine": 5,
        "imu_state": type("I", (), {"quaternion": (1.0, 0.0, 0.0, 0.0),
                                    "gyroscope": (0.0, 0.0, 0.0)})(),
        "wireless_remote": [0] * 40,
    })())
    assert adapter.check_ready_mode() and adapter.arm()
    adapter.submit({name: [0.0] * 29 for name in ("q", "dq", "kp", "kd", "tau")}, clock["now"] + 0.1)
    return adapter


def lines(trace):
    trace.close()
    return [json.loads(line) for line in trace.path.read_text().splitlines()]


def test_remembered_step_is_the_record_an_abort_logs(tmp_path):
    trace = ControlTrace(tmp_path / "trace.jsonl")
    trace.open()
    trace.note_step({"now": 1.0, "phase": "running"})
    trace.remember({"now": 1.02, "phase": "running", "gap_rad": 0.3})
    trace.emit_abort("hardware_output_worker_stalled", 1.2)
    written = lines(trace)
    assert [line["now"] for line in written] == [1.0, 1.2]
    assert written[1]["gap_rad"] == 0.3 and written[1]["abort"] == "hardware_output_worker_stalled"


def test_line_is_written_after_the_write_and_its_bookkeeping(tmp_path):
    adapter = make_adapter(tmp_path)
    seen = {}
    note_step = adapter.trace.note_step

    def checked_note_step(record):
        seen["writes"] = len(adapter.transport.writes)
        seen["q_applied"] = adapter.q_applied
        seen["last_output_secs"] = adapter.last_output_secs
        note_step(record)

    adapter.trace.note_step = checked_note_step
    adapter.step()
    assert seen["writes"] == 1
    assert seen["last_output_secs"] == 10.0
    written = lines(adapter.trace)
    assert len(written) == 1 and written[0]["now"] == 10.0


def test_abort_during_the_write_logs_the_step_being_written(tmp_path):
    adapter = make_adapter(tmp_path)
    write_arm = adapter.transport.write_arm

    def stalled_write(targets, weight):
        adapter.abort("hardware_output_worker_stalled")      # the watchdog thread, mid-write
        return write_arm(targets, weight)

    adapter.transport.write_arm = stalled_write
    adapter.step()
    written = lines(adapter.trace)
    assert written[0]["abort"] == "hardware_output_worker_stalled"
    assert written[0]["now"] == 10.0 and "gap_joint" in written[0]


def test_failed_write_logs_the_step_then_the_abort(tmp_path):
    adapter = make_adapter(tmp_path)
    adapter.transport.write_success = False
    adapter.step()
    assert adapter.abort_reason == "hardware_write_failed"
    written = lines(adapter.trace)
    assert len(written) == 2
    assert "abort" not in written[0] and written[1]["abort"] == "hardware_write_failed"
