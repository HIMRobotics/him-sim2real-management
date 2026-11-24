"""Opt-in command-driven output: the single worker steps when a new command arrives."""

import statistics
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest

import tests.test_robot_safety_hardware as hardware
from robot_safety.hardware_adapter import (
    COMMAND_FALLBACK_SLACK_SECS,
    COMMAND_STEP_MIN_SECS,
    G1HardwareAdapter,
)
from robot_safety.hardware_profile import HARDWARE_PERIOD_SECS
from tests.test_startup_hardware import handoff_policy, start
from tests.test_startup_hardware import rig as rig  # noqa: PLC0414


def run_command_output(arrivals, steps, duration=0.002, stop_policy_after=None):
    """Run the worker on a virtual clock. ``arrivals`` are new-command times."""
    clock = 0.0
    starts = []
    pending = sorted(arrivals)
    adapter = SimpleNamespace(
        _ensure_balance_policy=lambda: None, _ensure_soft_reset_policy=lambda: None,
        _ensure_startup_policy=lambda: None, transport=None, soft_reset_policy=None,
        balance_policy=None, whole_body=False, output_on_command=True,
        _command_wakes=0, policy_launched=True, abort_reason=None,
        stop_requested=SimpleNamespace(is_set=lambda: False),
        last_worker_cycle_secs=123.0,
    )

    def arrive(until):
        while pending and pending[0] <= until:
            pending.pop(0)
            adapter._command_wakes += 1

    def wait(delay):
        nonlocal clock
        target = clock + delay
        if pending and pending[0] <= target:
            clock = max(clock, pending[0])
            arrive(clock)
            return True
        clock = target
        return False

    def step():
        nonlocal clock
        starts.append(clock)
        if stop_policy_after is not None and len(starts) == stop_policy_after:
            adapter.abort_reason = "test_stop"
        clock += duration
        arrive(clock)

    adapter.step = step
    adapter.output_wake = SimpleNamespace(wait=wait, clear=lambda: None, set=lambda: None)
    adapter.shutdown_requested = SimpleNamespace(is_set=lambda: len(starts) >= steps)
    trace = SimpleNamespace(open=lambda: None, diagnostic_enabled=False, timing_profiler=None)

    def thread(target, name, daemon):
        return SimpleNamespace(run=target, name=name, start=lambda: None)

    with patch('robot_safety.hardware_adapter.time.monotonic', side_effect=lambda: clock), \
            patch('robot_safety.hardware_adapter.sys.setswitchinterval'), \
            patch('robot_safety.hardware_adapter.ControlTrace', return_value=trace), \
            patch('robot_safety.hardware_adapter.Thread', side_effect=thread):
        G1HardwareAdapter.start_workers(adapter)
        next(worker for worker in adapter.workers if worker.name == 'policy_output').run()
    assert adapter.last_worker_cycle_secs == 123.0
    return starts


def test_constants_keep_the_existing_period_and_cap_the_step_rate():
    assert HARDWARE_PERIOD_SECS == pytest.approx(0.02)
    assert COMMAND_STEP_MIN_SECS == pytest.approx(0.015)
    assert COMMAND_FALLBACK_SLACK_SECS == pytest.approx(0.005)


def test_without_commands_the_worker_keeps_twenty_millisecond_ticks():
    assert run_command_output([], 3) == pytest.approx([.020, .040, .060])


def test_new_commands_step_on_arrival_instead_of_the_next_tick():
    arrivals = [.005, .0258, .0466, .0674]
    assert run_command_output(arrivals, 4) == pytest.approx(arrivals)


def test_commands_closer_than_the_floor_wait_for_it():
    assert run_command_output([.005, .008], 2) == pytest.approx([.005, .020])


def test_a_flooding_policy_is_held_to_the_floor():
    arrivals = [.005 + .001 * i for i in range(60)]
    assert run_command_output(arrivals, 4) == pytest.approx([.005, .020, .035, .050])


def test_a_late_command_gets_one_timer_rewrite_then_the_normal_period():
    assert run_command_output([.005], 4) == pytest.approx([.005, .030, .050, .070])


def test_a_late_command_after_the_timer_rewrite_is_still_written_on_arrival():
    assert run_command_output([.005, .032], 3) == pytest.approx([.005, .030, .032])


def test_a_stop_returns_the_worker_to_the_plain_period():
    starts = run_command_output([.005, .060], 4, stop_policy_after=1)
    assert starts[:3] == pytest.approx([.005, .025, .045])


def test_shutdown_prevents_another_update():
    assert run_command_output([.005], 0) == []


def test_default_adapter_uses_the_existing_timer_loop():
    adapter = G1HardwareAdapter(hardware.load_profile(hardware.profile_data()))
    assert adapter.output_on_command is False


@pytest.mark.parametrize('kwargs, error', [
    ({'output_on_command': 'yes'}, TypeError), ({'output_on_command': 1}, TypeError),
])
def test_invalid_options_are_rejected(kwargs, error):
    with pytest.raises(error):
        G1HardwareAdapter(hardware.load_profile(hardware.profile_data()), **kwargs)


def make_arm_rig(enabled=True):
    rig = hardware.HardwareAdapterTests()
    rig.setUp()
    rig.adapter.output_on_command = enabled
    assert rig.adapter.arm()
    return rig


def test_only_a_new_sequence_from_a_running_policy_wakes_the_worker():
    rig = make_arm_rig()
    adapter = rig.adapter
    adapter.submit(dict(rig.command, seq=1), rig.now + 0.1)
    assert adapter._command_wakes == 1 and adapter.output_wake.is_set()
    adapter.output_wake.clear()
    adapter.submit(dict(rig.command, seq=1), rig.now + 0.1)   # the gate's 5 ms resubmission
    assert adapter._command_wakes == 1 and not adapter.output_wake.is_set()
    adapter.submit(dict(rig.command, seq=2), rig.now + 0.1)
    assert adapter._command_wakes == 2
    adapter.submit(dict(rig.command), rig.now + 0.1)            # no sequence: never wakes
    assert adapter._command_wakes == 2
    adapter.policy_launched = False
    adapter.submit(dict(rig.command, seq=3), rig.now + 0.1)
    assert adapter._command_wakes == 2


def test_disabled_option_never_wakes_the_worker():
    rig = make_arm_rig(enabled=False)
    rig.adapter.submit(dict(rig.command, seq=1), rig.now + 0.1)
    assert rig.adapter._command_wakes == 0 and not rig.adapter.output_wake.is_set()


def test_rejected_submission_never_wakes_the_worker():
    rig = make_arm_rig()
    rig.adapter.abort("test_stop")
    rig.adapter.output_wake.clear()
    rig.adapter.submit(dict(rig.command, seq=1), rig.now + 0.1)
    assert rig.adapter._command_wakes == 0 and not rig.adapter.output_wake.is_set()


def test_first_fault_wakes_the_worker_to_drop_the_fallback_allowance():
    rig = make_arm_rig()
    rig.adapter.abort("test_stop")
    assert rig.adapter.output_wake.is_set()


def test_whole_body_wakes_only_after_the_policy_handoff(rig):
    rig.adapter.output_on_command = True
    start(rig)
    rig.adapter.submit(dict(rig.command, seq=1), rig.now + 0.1)
    assert rig.adapter._command_wakes == 0
    handoff_policy(rig)
    rig.adapter.submit(dict(rig.command, seq=2), rig.now + 0.1)
    assert rig.adapter._command_wakes == 1


@pytest.mark.skipif(sys.platform == "win32", reason="Windows timers are too coarse for this check")
def test_real_worker_writes_each_new_command_promptly(tmp_path, monkeypatch):
    """The real threads: a 50 Hz policy, the gate's 5 ms resubmission and a 5 ms feed."""
    monkeypatch.setenv("HIM_CONTROL_TRACE", str(tmp_path / "trace.jsonl"))
    data = hardware.profile_data()
    data["max_publish_gap_secs"] = 0.1
    adapter = G1HardwareAdapter(hardware.load_profile(data), output_on_command=True)
    adapter.balance_policy = adapter.soft_reset_policy = adapter.startup_policy = SimpleNamespace(
        reset=lambda: None)
    rig = hardware.HardwareAdapterTests()
    writes, lock = [], threading.Lock()
    transport = hardware.FakeTransport()

    def write_arm(targets, weight):
        with lock:
            writes.append((time.monotonic(), targets[15]["q"]))
        return True

    transport.write_arm = write_arm
    adapter.transport = transport
    rig.adapter = adapter
    adapter.receive_battery(SimpleNamespace(soc=100))
    rig.state()
    assert adapter.check_ready_mode() and adapter.arm()
    stop = threading.Event()
    arrivals, held = {}, {}

    def feeder():
        while not stop.wait(0.005):
            rig.state()
            adapter.receive_battery(SimpleNamespace(soc=100))

    def policy():
        for seq in range(1, 41):
            if stop.wait(0.02):
                return
            command = {name: [0.0] * 29 for name in ("q", "dq", "kp", "kd", "tau")}
            command["q"][15] = round(seq * 1e-4, 6)
            command["seq"] = seq
            arrivals[command["q"][15]] = time.monotonic()
            held["command"] = command
            adapter.submit(command, time.monotonic() + 0.19)

    def gate_timer():
        while not stop.wait(0.005):
            if "command" in held:
                adapter.submit(held["command"], time.monotonic() + 0.19)

    switch = sys.getswitchinterval()
    threads = [threading.Thread(target=f, daemon=True) for f in (feeder, gate_timer)]
    try:
        adapter.start_workers()
        for thread in threads:
            thread.start()
        policy()
        time.sleep(0.05)
    finally:
        stop.set()
        for thread in threads:
            thread.join(1)
        adapter.abort("test_end")
        adapter.shutdown_requested.set()
        adapter.output_wake.set()
        for worker in adapter.workers:
            worker.join(1)
        adapter.trace.close()
        sys.setswitchinterval(switch)
    assert adapter.abort_reason == "test_end"
    first = {}
    for stamp, q in writes:
        first.setdefault(round(q, 6), stamp)
    delays = [first[q] - arrivals[q] for q in arrivals if q in first]
    assert len(arrivals) == 40 and len(delays) >= 38   # a loaded runner may replace one or two
    assert statistics.median(delays) < 0.005            # not the 0-25 ms of the timers
    stamps = [stamp for stamp, _ in writes]
    assert len(stamps) <= 1.1 * 40 + 5                  # no extra step rate from the gate's resubmits
