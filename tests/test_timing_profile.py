"""Timing instrumentation preserves control behavior and distinguishes waits."""

import hashlib
import json
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from robot_safety.control_trace import ControlTrace
from robot_safety.timing_profile import TimingProfiler, profiled, timed_lock, timed_wait
from tests.test_recovery_diagnostics import recovery_rig
from tests.test_transport_diagnostics import targets, transport
from tools.analyze_recovery_telemetry import summarize_timing
from tools.capture_telemetry import timing_preflight


def test_waiting_is_elapsed_time_not_thread_cpu_and_threads_are_attributed():
    records = []
    profiler = TimingProfiler(records.append)

    def worker():
        with profiler.scope("blocking_operation"):
            threading.Event().wait(.025)
        profiler.flush()

    thread = threading.Thread(target=worker, name="offline_waiter")
    thread.start()
    thread.join(2)
    assert not thread.is_alive()
    row = records[0]
    stage = row['stages']['blocking_operation']
    assert stage['wall_s'] >= .020
    assert stage['cpu_s'] < stage['wall_s']
    assert row['tid'] != threading.get_native_id()
    assert row['thread'] == 'offline_waiter'
    assert sum(stage['wall_histogram']) == 1


def test_lock_release_and_method_exception_survive_broken_profiler_sink():
    def broken(_):
        raise OSError("fake logging failure")

    class Worker:
        timing_profiler = TimingProfiler(broken, period=0)
        lock = threading.RLock()

        @profiled("work")
        def run(self):
            with timed_lock(self, self.lock, "lock"):
                with timed_lock(self, self.lock, "lock"):
                    raise ValueError("original failure")

    worker = Worker()
    with pytest.raises(ValueError, match="original failure"):
        worker.run()
    acquired = []

    def other_thread():
        acquired.append(worker.lock.acquire(timeout=.1))
        if acquired[-1]:
            worker.lock.release()

    thread = threading.Thread(target=other_thread)
    thread.start()
    thread.join(1)
    assert acquired == [True]
    assert worker.timing_profiler.errors > 0
    assert worker.timing_profiler.state()['stack'] == []


def test_timers_keep_requested_delay_and_report_overrun_and_wake_lateness():
    records = []
    owner = SimpleNamespace(timing_profiler=TimingProfiler(records.append, period=100))
    owner.timing_profiler.state()
    delays = []
    event = SimpleNamespace(wait=lambda delay: delays.append(delay) or False)
    with patch('robot_safety.timing_profile.time.monotonic', side_effect=[1.003, 1.030, 1.030, 1.030]), \
            patch('robot_safety.timing_profile.time.thread_time', side_effect=[.1, .101]):
        assert timed_wait(owner, event, .020, 1.000, "timer") is False
    owner.timing_profiler.flush()
    stage = records[0]['stages']['timer']
    assert delays == [.020]
    assert stage['deadline_missed_sum'] == 1
    assert stage['deadline_late_s_sum'] == pytest.approx(.003)
    assert stage['wake_late_s_sum'] == pytest.approx(.007)


@pytest.mark.parametrize('detail', [True, False])
def test_profiled_recovery_preserves_targets_and_counts_repeated_faults(detail):
    baseline, traced = recovery_rig(), recovery_rig([])
    records = []
    traced.adapter.timing_profiler = TimingProfiler(records.append)
    traced.adapter.trace.diagnostic_detail = detail
    for rig in (baseline, traced):
        rig.adapter.abort('thrash_detected')
        rig.adapter.step()
        rig.adapter.abort('thrash_detected')
        rig.adapter.step()
    traced.adapter.timing_profiler.flush()
    assert baseline.transport.body_commands == traced.transport.body_commands
    assert traced.adapter.automatic_soft_reset
    assert not traced.adapter.hold_failed
    stages = records[0]['stages']
    assert stages['abort_notifications']['repeated_reason_sum'] == 1
    assert any('check_health' in name for name in stages)
    assert any('recovery_target_checks' in name for name in stages)


def test_profiled_transport_still_resends_and_clears_the_same_command():
    instance = transport()
    records = []
    instance.timing_profiler = TimingProfiler(records.append)
    assert instance.write_body(targets(), 5)
    message = instance.lowcmd.Write.call_args.args[0]
    assert instance.repeat_body(5)
    assert instance.lowcmd.Write.call_args.args[0] is message
    instance.clear_body()
    assert instance.repeat_body(5) is None
    instance.timing_profiler.flush()
    assert any('command_crc' in name for name in records[0]['stages'])
    assert any('dds_command_write' in name for name in records[0]['stages'])


def test_async_logging_profiles_its_own_encoding_without_recursion(tmp_path, monkeypatch):
    monkeypatch.setenv('HIM_TIMING_PROFILE', '1')
    trace = ControlTrace(tmp_path / 'control.jsonl', diagnostic_path=tmp_path / 'detail.jsonl')
    trace.open()
    trace.note_diagnostic({'event': 'sample'})
    time.sleep(1.1)
    trace.note_diagnostic({'event': 'sample'})
    trace.close()
    rows = [json.loads(line) for line in (tmp_path / 'detail.jsonl').read_text().splitlines()]
    stages = [row['stages'] for row in rows if row.get('event') == 'timing_profile']
    assert any('diagnostic_json_encode' in row for row in stages)
    assert len(rows) < 30
    assert trace.diagnostic_dropped == trace.diagnostic_errors == 0


def test_timing_preflight_rejects_stale_code_and_missing_live_measurements(tmp_path):
    sources = Path(__file__).resolve().parents[1] / 'robot_safety'
    names = ('timing_profile.py', 'hardware_adapter.py', 'hardware_node.py', 'control_trace.py',
             'holosoma_policy.py', 'unitree_transport.py', 'sdk_latest.py')
    metadata = dict(timing_profile_version=1, pid=123,
                    source_sha256={name: hashlib.sha256((sources / name).read_bytes()).hexdigest()
                                   for name in names})
    meta = tmp_path / 'meta.json'
    trace = tmp_path / 'trace.jsonl'
    meta.write_text(json.dumps(metadata))
    trace.write_text(json.dumps(dict(event='timing_profile', version=1, pid=123, tid=123,
                                    mono_s=time.monotonic())) + '\n')
    assert timing_preflight(trace, meta, 5)['pid'] == 123
    metadata['source_sha256']['timing_profile.py'] = 'old'
    meta.write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match='different profiling code'):
        timing_preflight(trace, meta, 5)
    metadata['source_sha256']['timing_profile.py'] = hashlib.sha256(
        (sources / 'timing_profile.py').read_bytes()).hexdigest()
    meta.write_text(json.dumps(metadata))
    trace.write_text(json.dumps(dict(event='timing_profile', version=1, pid=123, tid=123,
                                    mono_s=time.monotonic() - 60)) + '\n')
    with pytest.raises(ValueError, match='No recent timing records'):
        timing_preflight(trace, meta, 5)
    metadata['timing_profile_version'] = None
    meta.write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match='not enabled'):
        timing_preflight(trace, meta, 5)


def test_summary_excludes_boundary_windows_and_keeps_nested_stages_separate(tmp_path):
    records = []
    profiler = TimingProfiler(records.append)
    with profiler.scope('outer'):
        with profiler.scope('inner'):
            pass
    profiler.flush()
    row = records[0]
    row.update(window_start_s=10, mono_s=11)
    path = tmp_path / 'trace.jsonl'
    path.write_text(json.dumps(row) + '\n' + json.dumps(dict(row, mono_s=12)) + '\n')
    result = summarize_timing(path, 10, 11)
    assert result['records'] == result['excluded_boundary_records'] == 1
    stages = next(iter(result['threads'].values()))['stages']
    assert stages['outer']['count'] == stages['outer/inner']['count'] == 1
    assert stages['outer']['mean_wall_ms'] >= stages['outer/inner']['mean_wall_ms']
