"""Recovery tracing preserves commands and selected input identity."""

import json
from dataclasses import replace
from types import SimpleNamespace

from robot_safety.control_trace import ControlTrace
from tests import test_robot_safety_hardware as hardware


def recovery_rig(records=None):
    rig = hardware.HardwareAdapterTests()
    rig.setUp()
    adapter = rig.adapter
    adapter.whole_body = True
    adapter.armed = True
    adapter.policy_launched = True
    adapter.ownership_taken = True
    adapter.balance_released = True
    adapter.soft_reset_policy.diagnostic_observation = [0.0] * 100
    if records is not None:
        adapter.trace = SimpleNamespace(
            diagnostic_enabled=True, note_diagnostic=records.append,
            emit_abort=lambda *args: None,
        )
    rig.state()
    return rig


def test_automatic_recovery_trace_matches_written_targets_and_selected_input():
    records = []
    rig = recovery_rig(records)
    adapter = rig.adapter
    selected = adapter.feedback
    selected_info = adapter._feedback_diagnostic[1]
    adapter.abort('thrash_detected')

    def infer(*args):
        rig.now += .001
        adapter.feedback = replace(selected, received_at_secs=rig.now)
        adapter._feedback_diagnostic = (adapter.feedback, {'tick': 999})
        return [2.0] * 29

    adapter.soft_reset_policy.update.side_effect = infer
    adapter.step()
    command = next(row for row in records if row['event'] == 'control_command')
    assert command['automatic']
    assert command['phase'] == 'recovery'
    assert command['input'] is selected_info
    assert command['feedback_received_at_s'] == selected.received_at_secs
    assert command['raw_q'] == [2.0] * 29
    assert command['actor_obs'] == [0.0] * 100
    targets, _ = rig.transport.body_commands[-1]
    assert command['applied'] == {str(i): value for i, value in targets.items()}
    assert command['applied']['0']['q'] != command['raw_q'][0]
    assert command['write_ok']
    assert command['write_finished_s'] >= command['write_started_s']
    assert command['infer_s'] >= 0
    assert adapter.automatic_soft_reset
    assert not adapter.hold_failed


def test_diagnostic_sink_failure_does_not_change_recovery_output():
    baseline = recovery_rig()
    traced = recovery_rig([])

    def broken(record):
        raise OSError('offline logger failure')

    traced.adapter.trace.note_diagnostic = broken
    for rig in (baseline, traced):
        rig.adapter.abort('thrash_detected')
        rig.adapter.step()
    assert baseline.transport.body_commands == traced.transport.body_commands
    assert traced.adapter.automatic_soft_reset
    assert not traced.adapter.hold_failed


def test_async_trace_keeps_full_rate_and_reports_invalid_records(tmp_path):
    path = tmp_path / 'diagnostics.jsonl'
    trace = ControlTrace(tmp_path / 'control.jsonl', diagnostic_path=path)
    trace.open()
    for index in range(100):
        trace.note_diagnostic({'event': 'sample', 'sequence': index})
    trace.note_diagnostic({'event': 'bad', 'value': float('nan')})
    trace.close()
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert [row['sequence'] for row in rows if row['event'] == 'sample'] == list(range(100))
    assert any(row['event'] == 'diagnostic_heartbeat' for row in rows)
    assert trace.diagnostic_errors == 1


def test_full_queue_drops_diagnostics_without_blocking(tmp_path):
    trace = ControlTrace(tmp_path / 'control.jsonl', diagnostic_path=tmp_path / 'detail.jsonl')
    for index in range(5000):
        trace.note_diagnostic({'sequence': index})
    assert trace._diagnostics.qsize() == 4096
    assert trace.diagnostic_dropped == 904


def test_invalid_diagnostic_path_does_not_break_existing_trace(tmp_path):
    path = tmp_path / 'control.jsonl'
    trace = ControlTrace(path, diagnostic_path=path)
    trace.open()
    assert not trace.diagnostic_enabled
    trace.note_step({'now': 1.0, 'phase': 'waiting'})
    trace.close()
    assert json.loads(path.read_text())['phase'] == 'waiting'
