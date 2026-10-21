"""Offline analysis of small synthetic telemetry captures."""

import importlib.util
import json
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location(
    'recovery_analyzer', Path(__file__).resolve().parents[1] / 'tools/analyze_recovery_telemetry.py')
ANALYZER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYZER)


def save(directory, name, records):
    (directory / name).write_text(''.join(json.dumps(row) + '\n' for row in records))


def command(stamp, phase='recovery', automatic=True, generation=1):
    return dict(event='control_command', phase=phase, automatic=automatic,
                startup_phase='running' if phase == 'submission' else 'fault',
                mono_s=stamp + .02, feedback_age_s=.02,
                infer_started_s=stamp + .005, infer_finished_s=stamp + .015, infer_s=.01,
                write_started_s=stamp + .02, write_finished_s=stamp + .022,
                input=dict(callback_entered_s=stamp + .001, callback_lock_wait_s=.003,
                           poll_sample=dict(take_started_s=stamp, take_duration_s=.001)),
                raw_q=[.5, .2], applied={'0': {'q': .3}, '1': {'q': .2}},
                transport=dict(body=dict(generation=generation, write_count=generation * 10,
                                         success_count=generation * 10, fresh_count=generation,
                                         repeat_count=generation * 9, clear_count=generation)))


def sample(stamp, publisher=1, source=None):
    return dict(topic='rt/lowcmd', take_monotonic_ns=int(stamp * 1e9),
                publication_handle=publisher,
                source_timestamp_ns=int((stamp if source is None else source) * 1e9))


def test_selected_input_ages_corrections_and_counters(tmp_path):
    save(tmp_path, 'diagnostic-trace.jsonl', [
        command(10., 'submission'),
        dict(event='abort_decision', mono_s=10.025, first_fault=True, automatic=True,
             soft_balance=True, reason='joint_torque_limit'),
        dict(event='recovery_enter', mono_s=10.03, automatic=True),
        command(10.04, generation=2),
        command(10.06, generation=3),
    ])
    save(tmp_path, 'samples.jsonl', [sample(10.025), sample(10.035), sample(10.055), sample(10.075)])
    result = ANALYZER.analyze(tmp_path)
    assert result['recovery_entries_observed'] == dict(automatic=1, manual=0)
    assert result['command_records_by_phase']['submitted_policy'] == 1
    assert result['command_records_by_phase']['automatic_recovery'] == 2
    timing = result['timing_ms_by_phase']['automatic_recovery']
    assert timing['poll_take_start_to_infer_started_s_ms']['max'] == pytest.approx(5.)
    assert timing['poll_take_start_to_write_finished_s_ms']['max'] == pytest.approx(22.)
    assert timing['callback_entry_to_write_finished_s_ms']['max'] == pytest.approx(21.)
    assert timing['adapter_received_to_write_finished_s_ms']['max'] == pytest.approx(22.)
    corrections = result['raw_to_applied']['automatic_recovery']
    assert corrections['corrected_command_count'] == 2
    assert corrections['joints'][0] == dict(index=0, count=2, max_abs_rad=.2)
    assert corrections['joints'][1]['count'] == 0
    assert result['transport']['deltas']['clear_count'] == 2
    assert result['publication_intervals_ms']['rt/lowcmd:1/phase_boundary']['writer_source_interval_ms']['count'] == 1
    assert (tmp_path / 'diagnostic-summary.json').exists()


def test_old_capture_missing_diagnostics_and_independent_publishers(tmp_path):
    save(tmp_path, 'samples.jsonl', [sample(1., 1, 100.), sample(1.005, 2, 900.),
                                    sample(1.02, 1, 100.02), sample(1.025, 2, 900.02)])
    with (tmp_path / 'samples.jsonl').open('a') as stream:
        stream.write('{truncated\n')
    result = ANALYZER.analyze(tmp_path)
    assert result['no_evidence_flags']['no_harness_diagnostics']
    assert result['no_evidence_flags']['no_automatic_recovery_commands']
    assert result['no_evidence_flags']['no_motor_latency_measurement']
    assert result['malformed_lines'] == {'samples.jsonl': 1}
    assert result['transport']['deltas'] is None
    for publisher in (1, 2):
        interval = result['publication_intervals_ms']['rt/lowcmd:{}/unknown'.format(publisher)]
        assert interval['writer_source_interval_ms']['max'] == pytest.approx(20.)
        assert interval['writer_source_interval_ms']['count'] == 1


def test_ros_fallback_does_not_invent_recovery_entry_or_latency(tmp_path):
    save(tmp_path, 'ros-events.jsonl', [
        dict(topic='/safety/hardware_status', receive_monotonic_ns=1000000000,
             status=dict(walk_stop=True, automatic_soft_reset=True)),
        dict(topic='/safety/abort', receive_monotonic_ns=1100000000, reason='old_fault'),
    ])
    save(tmp_path, 'samples.jsonl', [sample(1.1), sample(1.12)])
    result = ANALYZER.analyze(tmp_path)
    assert result['phase_source'] == 'ROS_status_receipt'
    assert result['recovery_entries_observed']['automatic'] == 0
    assert result['ros_abort_receipts'][0]['reason'] == 'old_fault'
    assert 'rt/lowcmd:1/automatic_recovery' in result['publication_intervals_ms']


def test_reservoir_is_bounded_and_tracks_exact_extremes():
    values = ANALYZER.Distribution(capacity=10)
    for value in range(100):
        values.add(value)
    summary = values.summary()
    assert summary['count'] == 100
    assert summary['min'] == 0
    assert summary['max'] == 99
    assert summary['mean'] == 49.5
    assert summary['quantile_sample_count'] == 10
    assert summary['quantiles_approximate']


def test_inference_without_write_is_reported_without_inventing_commands(tmp_path):
    save(tmp_path, 'diagnostic-trace.jsonl', [
        dict(event='recovery_enter', mono_s=1., automatic=True),
        dict(event='recovery_inference_start', mono_s=1., infer_started_s=1., automatic=True),
        dict(event='recovery_inference_end', mono_s=1.12, infer_started_s=1., infer_s=.12),
        dict(event='control_skipped', mono_s=1.13, reason='recovery_stopped_before_write'),
    ])
    result = ANALYZER.analyze(tmp_path)
    assert result['completed_inference_ms_including_unwritten']['automatic_recovery']['max'] == 120.
    assert result['diagnostic_events']['control_skipped'] == 1
    assert result['command_records_by_phase'] == {}
    assert result['no_evidence_flags']['no_automatic_recovery_commands']
