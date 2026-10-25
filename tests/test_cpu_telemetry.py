"""CPU profiling tests use this test process, never robot control."""

import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.capture_telemetry import start_cpu_observer, stop_ros_observer
from tools.cpu_telemetry import proc_stat, thread_snapshot


def test_stat_name_with_spaces_and_parentheses():
    fields = ['0'] * 39
    fields[0] = 'S'
    for index, value in ((11, 12), (12, 34), (15, 20), (16, 10), (19, 999), (36, 3), (37, 0), (38, 0)):
        fields[index] = str(value)
    row = proc_stat('123 (worker (one)) ' + ' '.join(fields))
    assert row['name'] == 'worker (one)'
    assert row['user_ticks'] == 12
    assert row['system_ticks'] == 34
    assert row['start_ticks'] == 999
    assert row['last_cpu'] == 3


def test_thread_snapshot_reports_unavailable_scheduler_stats(tmp_path):
    path = tmp_path / str(os.getpid())
    path.mkdir()
    (path / 'stat').write_text(Path('/proc/self/stat').read_text())
    row = thread_snapshot(path)
    assert row['schedstat'] is None
    assert row['voluntary_ctxt_switches'] is None


def test_cpu_observer_records_target_threads_and_host_counters(tmp_path):
    args = SimpleNamespace(cpu_profile=True, cpu_pid=os.getpid(), cpu_hz=5, seconds=1)
    worker = start_cpu_observer(args, tmp_path)
    stop_ros_observer(worker)
    status = json.loads((tmp_path / 'cpu-observer.json').read_text())
    rows = [json.loads(line) for line in (tmp_path / 'cpu-samples.jsonl').read_text().splitlines()]
    assert status['ready'] and status['error'] is None
    assert status['pid'] == os.getpid()
    assert rows[0]['threads']
    assert any(row['tid'] == os.getpid() for row in rows[0]['threads'])
    assert 'cpu_stat' in rows[0]['host']
    assert rows[0]['collection_duration_s'] >= 0
    assert rows[0]['monotonic_ns'] > 0


def test_cpu_auto_pid_requires_controller_identity(tmp_path):
    (tmp_path / 'diagnostic-metadata.json').write_text(json.dumps({'pid': os.getpid()}))
    args = SimpleNamespace(cpu_profile=True, cpu_pid=None, cpu_hz=5, seconds=1)
    with pytest.raises(RuntimeError, match='not the live hardware gate'):
        start_cpu_observer(args, tmp_path)


def test_cpu_worker_reports_missing_pid(tmp_path):
    result = subprocess.run([
        sys.executable, 'tools/capture_telemetry.py', '--cpu-worker',
        '--output', str(tmp_path), '--seconds', '1', '--cpu-pid', '2147483647',
    ], check=False, timeout=5)
    status = json.loads((tmp_path / 'cpu-observer.json').read_text())
    assert result.returncode != 0
    assert not status['ready']
    assert 'FileNotFoundError' in status['error']
