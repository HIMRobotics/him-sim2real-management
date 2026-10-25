"""Summarize passive captures and optional harness diagnostic traces."""

import argparse
import bisect
import json
import math
import random
from collections import Counter, defaultdict
from pathlib import Path


class Distribution:
    """Keep exact totals and a bounded deterministic quantile sample."""

    def __init__(self, capacity=8192):
        self.capacity = capacity
        self.values = []
        self.count = 0
        self.total = 0.
        self.minimum = self.maximum = None
        self.random = random.Random(712)

    def add(self, value):
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            return
        self.count += 1
        self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)
        if len(self.values) < self.capacity:
            self.values.append(value)
        else:
            index = self.random.randrange(self.count)
            if index < self.capacity:
                self.values[index] = value

    def summary(self):
        values = sorted(self.values)

        def percentile(fraction):
            if not values:
                return None
            position = fraction * (len(values) - 1)
            lower = int(position)
            return values[lower] + (values[min(lower + 1, len(values) - 1)] - values[lower]) * (position - lower)

        return dict(count=self.count, min=self.minimum, max=self.maximum,
                    mean=self.total / self.count if self.count else None,
                    median=percentile(.5), p95=percentile(.95), p99=percentile(.99),
                    quantile_sample_count=len(values), quantiles_approximate=self.count > len(values))


def rows(path, errors):
    if not path.exists():
        return
    with path.open() as stream:
        for number, line in enumerate(stream, 1):
            try:
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise ValueError('Expected object')
                yield value
            except (ValueError, TypeError):
                errors[path.name] += 1


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def difference_ms(later, earlier):
    return 1000 * (later - earlier) if finite(later) and finite(earlier) else None


def event_phase(row):
    if row.get('event') == 'control_command':
        phase = row.get('phase', 'unknown')
        if phase == 'recovery':
            return 'automatic_recovery' if row.get('automatic') else 'manual_recovery'
        if phase == 'submitted' or row.get('startup_phase') == 'running':
            return 'submitted_policy'
        return phase
    if row.get('event') == 'recovery_enter':
        return 'automatic_recovery' if row.get('automatic') else 'manual_recovery'
    if row.get('event') == 'recovery_error':
        return 'recovery_error'
    if row.get('event') == 'abort_decision' and row.get('hold_failed'):
        return 'damping_requested'
    return None


def add_transition(transitions, stamp, phase):
    if finite(stamp) and phase and (not transitions or transitions[-1][1] != phase):
        if len(transitions) < 10000:
            transitions.append((stamp, phase))


def summarize_timing(path, start=None, end=None):
    """Summarize complete timing windows without double-counting nested stages."""
    errors = Counter()
    groups = {}
    records = boundary_records = profiler_errors = 0
    edges = None
    for row in rows(Path(path), errors):
        if row.get("event") != "timing_profile":
            continue
        left, right = row.get("window_start_s"), row.get("mono_s")
        if not finite(left) or not finite(right):
            errors["timing_window"] += 1
            continue
        if (start is not None and left < start) or (end is not None and right > end):
            boundary_records += 1
            continue
        records += 1
        profiler_errors = max(profiler_errors, row.get("profiler_errors", 0))
        edges = row.get("wall_histogram_edges_s")
        identity = str(row.get("pid")) + ":" + str(row.get("tid"))
        group = groups.setdefault(identity, {"thread": row.get("thread"), "windows": 0,
                                            "window_seconds": 0.0, "stages": {}})
        group["windows"] += 1
        group["window_seconds"] += right - left
        for name, stage in row.get("stages", {}).items():
            total = group["stages"].setdefault(name, {})
            for key, value in stage.items():
                if key == "slowest_finished_s":
                    continue
                if key == "wall_histogram":
                    old = total.setdefault(key, [0] * len(value))
                    total[key] = [a + b for a, b in zip(old, value)]
                elif key.endswith("_max") or key == "wall_max_s":
                    total[key] = max(total.get(key, 0), value)
                else:
                    total[key] = total.get(key, 0) + value
    for group in groups.values():
        for stage in group["stages"].values():
            count = stage.get("count", 0)
            if count:
                stage["mean_wall_ms"] = 1000 * stage["wall_s"] / count
                stage["mean_calling_thread_cpu_ms"] = 1000 * stage["cpu_s"] / count
                stage["mean_non_cpu_elapsed_ms"] = 1000 * max(0, stage["wall_s"] - stage["cpu_s"]) / count
                stage["calls_per_window_second"] = count / group["window_seconds"] if group["window_seconds"] else None
    return {"records": records, "excluded_boundary_records": boundary_records,
            "profiler_errors_max": profiler_errors, "parse_errors": dict(errors),
            "wall_histogram_edges_s": edges, "threads": groups,
            "notes": [
                "Stages include nested work. Do not sum parent and child durations.",
                "CPU time belongs to the calling thread, not all ONNX/native helper threads.",
                "Wall minus CPU is not a direct GIL measurement. It includes locks, sleep, I/O and scheduling.",
                "Only complete reporting windows within capture bounds are included. Calls are assigned at completion.",
                "Active threads report about once per second. A blocked call appears only after returning.",
                "Histograms have an overflow bin. Timer interruption counts include shutdown wake-ups.",
            ]}


def analyze(directory, plot=False):
    directory = Path(directory)
    errors = Counter()
    event_counts = Counter()
    recovery_entries = Counter()
    commands = Counter()
    metrics = defaultdict(lambda: defaultdict(Distribution))
    corrections = defaultdict(lambda: [dict(count=0, max_abs_rad=0.) for _ in range(29)])
    corrected_commands = Counter()
    missing_inputs = Counter()
    aborts = []
    transitions = []
    transport_first = transport_last = None
    transport_resets = 0
    diagnostic_points = []
    last_plot_stamp = -math.inf
    dropped = encoding_errors = 0
    inference_starts = {}
    all_inference_metrics = defaultdict(Distribution)
    for row in rows(directory / 'diagnostic-trace.jsonl', errors):
        event = row.get('event', 'unknown')
        event_counts[event] += 1
        add_transition(transitions, row.get('mono_s'), event_phase(row))
        if event == 'diagnostic_heartbeat':
            dropped = max(dropped, row.get('dropped', 0))
            encoding_errors = max(encoding_errors, row.get('encoding_errors', 0))
        if event == 'recovery_enter':
            recovery_entries['automatic' if row.get('automatic') else 'manual'] += 1
        if event == 'recovery_inference_start':
            inference_starts = {row.get('infer_started_s'): row}
        if event == 'recovery_inference_end':
            selected_start = inference_starts.pop(row.get('infer_started_s'), {})
            phase = 'automatic_recovery' if selected_start.get('automatic') else 'manual_or_unpaired_recovery'
            value = row.get('infer_s')
            all_inference_metrics[phase].add(1000 * value if finite(value) else None)
        if event in ('abort_decision', 'recovery_error') and len(aborts) < 200:
            aborts.append({key: row[key] for key in (
                'event', 'mono_s', 'reason', 'latched_reason', 'first_fault', 'automatic',
                'soft_balance', 'hold_failed', 'buffer_preserved', 'error') if key in row})
        if event != 'control_command':
            continue
        phase = event_phase(row)
        commands[phase] += 1
        joint_corrections = corrections[phase]
        selected = row.get('input') or {}
        poll = selected.get('poll_sample') or {}
        if not selected:
            missing_inputs[phase] += 1
        for label, origin in (('poll_take_start', poll.get('take_started_s')),
                              ('callback_entry', selected.get('callback_entered_s')),
                              ('selected_received', selected.get('received_at_s'))):
            for endpoint in ('infer_started_s', 'write_finished_s'):
                metrics[phase][label + '_to_' + endpoint + '_ms'].add(difference_ms(row.get(endpoint), origin))
        for label, value in (
            ('inference_ms', row.get('infer_s')),
            ('prepare_ms', row.get('prepare_s')),
            ('feedback_age_at_preparation_ms', row.get('feedback_age_s')),
            ('callback_lock_wait_ms', selected.get('callback_lock_wait_s')),
            ('take_decode_ms', poll.get('take_duration_s')),
        ):
            metrics[phase][label].add(value * 1000 if finite(value) else None)
        metrics[phase]['write_call_ms'].add(difference_ms(row.get('write_finished_s'), row.get('write_started_s')))
        if finite(row.get('feedback_age_s')) and finite(row.get('mono_s')):
            for endpoint in ('infer_started_s', 'write_finished_s'):
                if finite(row.get(endpoint)):
                    metrics[phase]['adapter_received_to_' + endpoint + '_ms'].add(
                        1000 * (row['feedback_age_s'] + row[endpoint] - row['mono_s']))
        body = (row.get('transport') or {}).get('body') or {}
        if body:
            if transport_first is None:
                transport_first = body
            if transport_last and body.get('write_count', 0) < transport_last.get('write_count', 0):
                transport_resets += 1
            transport_last = body
            for key in ('last_lock_wait_s', 'last_write_duration_s', 'last_lock_held_s'):
                value = body.get(key)
                metrics[phase][key.replace('_s', '_ms')].add(value * 1000 if finite(value) else None)
        maximum = 0.
        for index, raw in enumerate((row.get('raw_q') or [])[:29]):
            applied = (row.get('applied') or {}).get(str(index), {}).get('q')
            if not finite(raw) or not finite(applied):
                continue
            delta = abs(raw - applied)
            maximum = max(maximum, delta)
            if delta > 1e-8:
                entry = joint_corrections[index]
                entry['count'] += 1
                entry['max_abs_rad'] = max(entry['max_abs_rad'], delta)
        corrected_commands[phase] += int(maximum > 1e-8)
        stamp = row.get('mono_s')
        if finite(stamp) and stamp - last_plot_stamp >= .02 and len(diagnostic_points) < 10000:
            diagnostic_points.append((stamp, row.get('infer_s'),
                                      difference_ms(row.get('write_finished_s'), poll.get('take_started_s')), maximum))
            last_plot_stamp = stamp

    ros_counts = Counter()
    ros_transitions = []
    ros_aborts = []
    for row in rows(directory / 'ros-events.jsonl', errors):
        topic = row.get('topic', 'unknown')
        ros_counts[topic] += 1
        stamp = row.get('receive_monotonic_ns')
        stamp = stamp / 1e9 if finite(stamp) else None
        status = row.get('status') or {}
        if status:
            phase = ('automatic_recovery' if status.get('automatic_soft_reset') else 'manual_recovery') if status.get('walk_stop') else (
                'submitted_policy' if status.get('policy_launched') and not status.get('abort_reason') else 'unknown')
            add_transition(ros_transitions, stamp, phase)
        elif row.get('reason') and len(ros_aborts) < 200:
            ros_aborts.append(dict(mono_s=stamp, reason=row['reason'], source=row.get('source')))
    phase_source = 'harness_diagnostic_events' if transitions else 'ROS_status_receipt' if ros_transitions else 'unavailable'
    transitions = sorted(transitions or ros_transitions)
    transition_times = [value[0] for value in transitions]

    def phase_at(stamp):
        index = bisect.bisect_right(transition_times, stamp) - 1
        return transitions[index][1] if index >= 0 else 'unknown'

    intervals = defaultdict(lambda: defaultdict(Distribution))
    publisher_counts = Counter()
    previous = {}
    state_points = []
    last_state_plot = -math.inf
    start = end = None
    for row in rows(directory / 'samples.jsonl', errors):
        take_ns = row.get('take_monotonic_ns')
        if not finite(take_ns):
            continue
        stamp = take_ns / 1e9
        start = stamp if start is None else min(start, stamp)
        end = stamp if end is None else max(end, stamp)
        topic = row.get('topic', 'unknown')
        publisher = str(row.get('publication_handle', 'unknown'))
        key = topic + ':' + publisher
        publisher_counts[key] += 1
        phase = phase_at(stamp)
        old = previous.get(key)
        source_ns = row.get('source_timestamp_ns')
        if old:
            group = phase if phase == old[2] else 'phase_boundary'
            intervals[key + '/' + group]['observer_take_interval_ms'].add((take_ns - old[0]) / 1e6)
            if finite(source_ns) and finite(old[1]):
                intervals[key + '/' + group]['writer_source_interval_ms'].add((source_ns - old[1]) / 1e6)
        previous[key] = (take_ns, source_ns, phase)
        if topic == 'rt/lowstate' and stamp - last_state_plot >= .02 and len(state_points) < 10000:
            quaternion = row.get('quaternion') or []
            tilt = None
            if len(quaternion) == 4 and all(finite(v) for v in quaternion):
                norm = sum(v * v for v in quaternion)
                if norm:
                    upright = 1 - 2 * (quaternion[1] ** 2 + quaternion[2] ** 2) / norm
                    tilt = math.degrees(math.acos(max(-1., min(1., upright))))
            torque = max((abs(v) for v in row.get('tau_est', []) if finite(v)), default=None)
            state_points.append((stamp, tilt, torque))
            last_state_plot = stamp

    capture_summary = {}
    try:
        capture_summary = json.loads((directory / 'summary.json').read_text())
    except (OSError, ValueError):
        pass
    result = {
        'method': 'Streaming analysis with bounded deterministic reservoir quantiles. Local monotonic timestamps align harness and observer on the same host. Writer-source intervals use differences within one publisher only. No source-to-local clock subtraction.',
        'limitations': [
            'No command acknowledgement or motor actuation timestamps, so this does not measure motor latency or establish causation.',
            'Poll take start precedes decode, but is not NIC arrival or robot sampling time. Callback timing excludes earlier transport delay.',
            'Publication phase labels use local observer receipt and may lag controller transitions. ROS abort receipts can be retained or repeated old faults.',
            'Adapter-received endpoint ages extrapolate the preparation age using local elapsed time. They assume adapter and monotonic clocks advance together.',
            'Raw-to-applied differences include all harness corrections and do not identify a particular clipping rule.',
            'Transport deltas cover first-to-last command snapshots only. Counters may include repeats between phases.',
        ],
        'capture_label': capture_summary.get('operator_label'),
        'capture_monotonic_start_s': start, 'capture_monotonic_end_s': end,
        'diagnostic_events': dict(event_counts),
        'recovery_entries_observed': dict(automatic=recovery_entries['automatic'], manual=recovery_entries['manual']),
        'command_records_by_phase': dict(commands),
        'completed_inference_ms_including_unwritten': {phase: values.summary() for phase, values in all_inference_metrics.items()},
        'missing_selected_input_by_phase': dict(missing_inputs),
        'abort_events': aborts, 'abort_events_truncated': sum(event_counts[e] for e in ('abort_decision', 'recovery_error')) > len(aborts),
        'ros_event_counts': dict(ros_counts), 'ros_abort_receipts': ros_aborts,
        'phase_source': phase_source, 'phase_transitions': transitions,
        'timing_ms_by_phase': {phase: {name: values.summary() for name, values in fields.items()} for phase, fields in metrics.items()},
        'raw_to_applied': {phase: dict(corrected_command_count=corrected_commands[phase],
                                      joints=[dict(index=i, **entry) for i, entry in enumerate(entries)])
                           for phase, entries in corrections.items()},
        'publisher_sample_counts': dict(publisher_counts),
        'publication_intervals_ms': {key: {name: values.summary() for name, values in fields.items()} for key, fields in intervals.items()},
        'transport': dict(first=transport_first, last=transport_last, counter_resets_observed=transport_resets,
                          deltas={key: transport_last.get(key, 0) - transport_first.get(key, 0)
                                  for key in ('write_count', 'success_count', 'fresh_count', 'repeat_count', 'clear_count')}
                          if transport_first and transport_last and not transport_resets else None),
        'diagnostic_dropped_max': dropped, 'diagnostic_encoding_errors_max': encoding_errors,
        'malformed_lines': dict(errors),
        'no_evidence_flags': {
            'no_harness_diagnostics': not event_counts,
            'no_automatic_recovery_entry': not recovery_entries['automatic'],
            'no_automatic_recovery_commands': not commands['automatic_recovery'],
            'no_submitted_policy_commands': not commands['submitted_policy'],
            'no_ROS_status': not ros_counts['/safety/hardware_status'],
            'no_motor_latency_measurement': True,
        },
    }
    if plot:
        try:
            make_plot(directory, diagnostic_points, state_points, transitions, start)
            result['plot'] = 'diagnostic-timeline.png'
        except ImportError:
            result['plot_unavailable'] = 'matplotlib is not installed'
    timing = summarize_timing(
        directory / 'diagnostic-trace.jsonl',
        capture_summary.get('start_monotonic_ns', 0) / 1e9 or start,
        capture_summary.get('end_monotonic_ns', 0) / 1e9 or end)
    (directory / 'timing-summary.json').write_text(json.dumps(timing, indent=2) + '\n')
    result['timing_summary'] = 'timing-summary.json'
    (directory / 'diagnostic-summary.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def make_plot(directory, diagnostics, states, transitions, start):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    start = start if start is not None else diagnostics[0][0] if diagnostics else 0.
    figure, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    if diagnostics:
        x = [r[0] - start for r in diagnostics]
        axes[0].plot(x, [1000 * r[1] if finite(r[1]) else math.nan for r in diagnostics], label='Inference')
        axes[0].plot(x, [r[2] if finite(r[2]) else math.nan for r in diagnostics], label='Selected poll start to write end')
        axes[3].plot(x, [r[3] for r in diagnostics])
        axes[0].legend()
    else:
        for axis in (axes[0], axes[3]):
            axis.text(.5, .5, 'No harness diagnostic records in this capture',
                      transform=axis.transAxes, ha='center')
    if states:
        x = [r[0] - start for r in states]
        axes[1].plot(x, [r[1] if finite(r[1]) else math.nan for r in states])
        axes[2].plot(x, [r[2] if finite(r[2]) else math.nan for r in states])
    for axis, label in zip(axes, ('Local timing (ms)', 'Tilt (degrees)', 'Max |torque estimate| (Nm)', 'Max target correction (rad)')):
        axis.set_ylabel(label)
        axis.grid(alpha=.25)
        for stamp, phase in transitions:
            if 'recovery' in phase:
                axis.axvline(stamp - start, color='orange', alpha=.5)
    axes[-1].set_xlabel('Seconds from first observed DDS sample')
    figure.suptitle('Passive capture and harness diagnostics — no motor latency or causal inference')
    figure.tight_layout(rect=(0, 0, 1, .96))
    figure.savefig(directory / 'diagnostic-timeline.png', dpi=150)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    if not args.capture.is_dir():
        parser.error('capture must be an existing directory')
    result = analyze(args.capture, args.plot)
    print(json.dumps(dict(output=str(args.capture / 'diagnostic-summary.json'),
                          recovery_entries=result['recovery_entries_observed'],
                          no_evidence_flags=result['no_evidence_flags']), indent=2))


if __name__ == '__main__':
    main()
