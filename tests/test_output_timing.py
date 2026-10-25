"""Exercise the actual output worker with a virtual clock and no hardware."""

from types import SimpleNamespace
from unittest.mock import patch

import pytest

from robot_safety.hardware_adapter import G1HardwareAdapter


def run_output(durations, wake_delays=(), stopped=False):
    clock = 0.0
    starts, waits = [], []

    def wait(delay):
        nonlocal clock
        waits.append(delay)
        if stopped or len(starts) == len(durations):
            return True
        oversleep = wake_delays[len(starts)] if len(starts) < len(wake_delays) else 0
        clock += delay + oversleep
        return False

    def step():
        nonlocal clock
        starts.append(clock)
        clock += durations[len(starts) - 1]

    adapter = SimpleNamespace(
        _ensure_balance_policy=lambda: None, _ensure_soft_reset_policy=lambda: None,
        _ensure_startup_policy=lambda: None, transport=None, soft_reset_policy=None,
        balance_policy=None, whole_body=False,
        shutdown_requested=SimpleNamespace(wait=wait), step=step,
        last_worker_cycle_secs=123.0,
    )
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
    return starts, waits


def test_fast_updates_keep_twenty_millisecond_start_spacing():
    starts, waits = run_output([.008, .008, .008])
    assert starts == pytest.approx([.020, .040, .060])
    assert waits[:3] == pytest.approx([.020, .012, .012])


def test_late_update_runs_immediately_then_resumes_normal_spacing():
    starts, waits = run_output([.029, .005, .005])
    assert starts == pytest.approx([.020, .049, .069])
    assert waits[:3] == pytest.approx([.020, 0, .015])


def test_multiple_missed_periods_do_not_create_catch_up_updates():
    starts, waits = run_output([.085, .001, .001])
    assert starts == pytest.approx([.020, .105, .125])
    assert waits[:3] == pytest.approx([.020, 0, .019])


def test_late_wake_rebases_the_following_deadline():
    starts, waits = run_output([.005, .005, .005], wake_delays=[.009])
    assert starts == pytest.approx([.029, .049, .069])
    assert waits[:3] == pytest.approx([.020, .015, .015])


def test_sustained_overruns_do_not_queue_or_overlap_updates():
    starts, waits = run_output([.029] * 3)
    assert starts == pytest.approx([.020, .049, .078])
    assert waits[:3] == pytest.approx([.020, 0, 0])


def test_shutdown_prevents_another_update():
    starts, _ = run_output([.005], stopped=True)
    assert starts == []
