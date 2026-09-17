#!/usr/bin/env python3
"""Read-only G1 session collection. Python 3.9+, SDK required only for live DDS."""
import argparse
import collections
import datetime as dt
import hashlib
import html
import importlib.metadata
import json
import math
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import socket
import subprocess
import sys
import tarfile
import threading
import time
import uuid

VERSION = '1.2.1'
ROOT = Path(__file__).resolve().parent
OUTCOMES = ('success', 'partial', 'failure', 'invalid', 'aborted', 'unknown')
MOTOR_FIELDS = ('mode', 'q', 'dq', 'ddq', 'tau_est', 'temperature', 'vol', 'sensor', 'motorstate', 'lost')
COMMAND_FIELDS = ('mode', 'q', 'dq', 'tau', 'kp', 'kd')
IMU_FIELDS = ('quaternion', 'gyroscope', 'accelerometer', 'rpy', 'temperature')


def utc():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def atomic_json(path, data):
    path = Path(path)
    tmp = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.tmp')
    with tmp.open('x', encoding='utf-8') as f:
        json.dump(data, f, indent=2, allow_nan=False)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def safe_value(value):
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (list, tuple, bytes, bytearray)) or hasattr(value, '__iter__'):
        return [safe_value(x) for x in value]
    if hasattr(value, 'item'):
        return safe_value(value.item())
    raise TypeError('Unsupported telemetry value: ' + type(value).__name__)


def fields(obj, names):
    return {name: safe_value(getattr(obj, name, None)) for name in names}


def state_payload(msg):
    motors = list(msg.motor_state)
    if len(motors) < 29:
        raise ValueError('Expected G1 HG state with at least 29 motor slots')
    for m in motors[:29]:
        if not math.isfinite(float(m.q)) or not math.isfinite(float(m.dq)):
            raise ValueError('Nonfinite motor position/velocity')
    return {
        **fields(msg, ('tick', 'version', 'mode_pr', 'mode_machine', 'wireless_remote', 'crc')),
        'motors': [fields(m, MOTOR_FIELDS) for m in motors],
        'imu': fields(getattr(msg, 'imu_state', None), IMU_FIELDS),
    }


def command_payload(msg):
    return {**fields(msg, ('mode_pr', 'mode_machine', 'crc')),
            'motors': [fields(m, COMMAND_FIELDS) for m in msg.motor_cmd]}


def json_lines(path, issues=None):
    if not path.exists():
        return
    with path.open(encoding='utf-8') as f:
        for i, line in enumerate(f, 1):
            try:
                yield json.loads(line)
            except (ValueError, UnicodeError):
                if issues is not None:
                    issues.append('{}: invalid JSON at line {}'.format(path.name, i))


class Receiver:
    """DDS callbacks never write files; bounded queue with visible overflow accounting."""
    def __init__(self, hz=50, capacity=2000):
        self.queue = queue.Queue(capacity)
        self.hz = hz
        self.lock = threading.Lock()
        self.stats = collections.defaultdict(lambda: {'received_callbacks': 0, 'sampled': 0,
            'rate_limited': 0, 'queue_dropped': 0, 'decode_errors': 0, 'last_error': None})
        self.last_sample = {}
        self.last_state = None
        self.ready = threading.Event()
        self.enabled = True

    def callback(self, topic, kind):
        def receive(msg):
            now = time.monotonic()
            with self.lock:
                if not self.enabled:
                    return
                s = self.stats[topic]
                s['received_callbacks'] += 1
                if now - self.last_sample.get(topic, -1e20) < 1.0 / self.hz:
                    s['rate_limited'] += 1
                    return
                self.last_sample[topic] = now
                try:
                    payload = state_payload(msg) if kind == 'state' else command_payload(msg)
                    row = {'schema': 'g1-session-sample.v1', 'kind': kind, 'topic': topic,
                           'host_monotonic_s': now, 'host_unix_ns': time.time_ns(), 'data': payload}
                    self.queue.put_nowait(row)
                    s['sampled'] += 1
                    if kind == 'state':
                        self.last_state = now
                        self.ready.set()
                except queue.Full:
                    s['queue_dropped'] += 1
                except Exception as exc:
                    s['decode_errors'] += 1
                    s['last_error'] = str(exc)
        return receive

    def snapshot(self):
        with self.lock:
            return {k: dict(v) for k, v in self.stats.items()}


def connect(receiver, interface, domain, commands=True):
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
    except ImportError as exc:
        raise RuntimeError('Unitree SDK / Cyclone DDS unavailable in this Python. Use the robot\'s '
                           'existing SDK Python: PYTHON=/path/to/python ./start.sh. See TROUBLESHOOTING.md.') from exc
    ChannelFactoryInitialize(domain, None if interface == 'auto' else interface)
    subscribers = []
    warnings = []
    specs = [('rt/lowstate', 'state', LowState_)]
    if commands:
        specs += [('rt/lowcmd', 'command', LowCmd_), ('rt/arm_sdk', 'command', LowCmd_)]
    for topic, kind, message_type in specs:
        try:
            sub = ChannelSubscriber(topic, message_type)
            # Direct lightweight callback avoids the SDK's uncounted bounded callback queue.
            sub.Init(receiver.callback(topic, kind), 0)
            subscribers.append(sub)
        except Exception as exc:
            if kind == 'state':
                raise RuntimeError('Cannot subscribe to rt/lowstate: ' + str(exc)) from exc
            warnings.append('Optional {} subscription failed: {}'.format(topic, exc))
    return subscribers, warnings


def demo_source(receiver, stop):
    from types import SimpleNamespace as NS
    tick = 0
    callback = receiver.callback('rt/lowstate', 'state')
    command = receiver.callback('rt/arm_sdk', 'command')
    while not stop.is_set():
        tick += 20
        motors = [NS(q=math.sin(tick / 1000) * .1, dq=.1, tau_est=.2,
                     temperature=[32, 33], mode=1) for _ in range(35)]
        callback(NS(motor_state=motors, tick=tick, imu_state=NS(rpy=[0., .01, 0.])))
        command(NS(motor_cmd=[NS(q=m.q, dq=.1, tau=0., kp=10., kd=1.) for m in motors]))
        stop.wait(.02)


def new_run(root, name, run_id=None):
    slug = re.sub(r'[^a-zA-Z0-9_-]+', '-', name).strip('-')[:70]
    if not slug:
        raise ValueError('Run name must contain a letter or number')
    root.mkdir(parents=True, exist_ok=True)
    if run_id is not None and not re.fullmatch(r'[A-Za-z0-9_-]{1,150}', run_id):
        raise ValueError('Invalid run ID')
    path = root / (run_id or (dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + slug + '_' + uuid.uuid4().hex[:6]))
    path.mkdir()
    for d in ('telemetry', 'events', 'evidence', 'policy'):
        (path / d).mkdir()
    return path.resolve()


def copy_evidence(run, source, role='evidence'):
    source = Path(source).expanduser().resolve()
    if not source.is_file():
        raise ValueError('Evidence must be an existing file: ' + str(source))
    destination = run / 'evidence' / (uuid.uuid4().hex[:10] + '_' + source.name)
    if shutil.disk_usage(run).free < source.stat().st_size + 256 * 1024**2:
        raise ValueError('Insufficient disk space to copy evidence')
    shutil.copyfile(source, destination)
    before = digest(source)
    after = digest(destination)
    if before != after:
        destination.unlink()
        raise ValueError('Source changed while copying; stop its writer and retry')
    event = {'schema': 'g1-session-event.v1', 'kind': 'evidence', 'role': role,
             'path': str(destination.relative_to(run)), 'sha256': after, 'utc': utc(),
             'host_monotonic_s': time.monotonic(), 'original_name': source.name}
    atomic_json(run / 'events' / (str(time.time_ns()) + '_' + uuid.uuid4().hex + '.json'), event)
    return destination


def collect(args):
    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    if command and args.seconds:
        raise ValueError('--seconds cannot be combined with a wrapped policy; stop the policy normally')
    if args.demo and command:
        raise ValueError('Demo cannot launch a policy')
    if args.reserve_gib < .1 or args.hz <= 0 or args.hz > 500 or args.seconds < 0 or args.timeout <= 0:
        raise ValueError('Use reserve >= 0.1 GiB, rate 0 < Hz <= 500, seconds >= 0, timeout > 0')
    run = new_run(args.root, args.name, args.run_id)
    receiver = Receiver(args.hz)
    stop = threading.Event()
    subscribers = []
    child = None
    old_handlers = {}
    reason = 'startup_failed'
    error = None
    warnings = []
    manifest = {'schema': 'g1-session.v1', 'kit_version': VERSION, 'name': args.name,
                'source': 'synthetic_demo' if args.demo else 'physical_dds', 'started_utc': utc(),
                'start_monotonic_s': time.monotonic(), 'host': socket.gethostname(),
                'python': sys.version, 'platform': sys.platform, 'operator': args.operator,
                'policy_label': args.policy, 'robot_label': args.robot, 'notes': args.notes,
                'interface': args.interface, 'domain': args.domain, 'sample_rate_limit_hz': args.hz,
                'reserve_gib': args.reserve_gib, 'policy_command': command,
                'policy_working_directory': str(Path.cwd()), 'collector_sha256': digest(__file__),
                'clock': 'host receipt monotonic seconds; UTC anchor via host Unix ns; NOT sensor exposure',
                'motor_order': 'raw HG DDS slots; first 29 are G1 joints; remaining slots retained separately',
                'task_metrics': 'operator annotations only; no automatic task adjudication',
                'status': 'recording'}
    try:
        manifest['unitree_sdk_version'] = importlib.metadata.version('unitree-sdk2py')
    except importlib.metadata.PackageNotFoundError:
        manifest['unitree_sdk_version'] = None
    atomic_json(run / 'manifest.json', manifest)
    atomic_json(run / 'ACTIVE.json', {'pid': os.getpid(), 'host': socket.gethostname()})
    print('\nRUN DIRECTORY: {}\n'.format(run), flush=True)
    print('Read-only recorder. Robot stop procedures remain with the policy operator.', flush=True)

    def interrupted(signum, _frame):
        stop.set()
        if child is not None and child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    for sig in (signal.SIGINT, signal.SIGTERM):
        old_handlers[sig] = signal.signal(sig, interrupted)
    counts = collections.Counter()
    try:
        if shutil.disk_usage(run).free < args.reserve_gib * 1024**3:
            raise RuntimeError('Disk free space is below the configured reserve')
        for artifact in args.artifact:
            copy_evidence(run, artifact, 'policy_or_calibration_artifact')
        if args.demo:
            threading.Thread(target=demo_source, args=(receiver, stop), daemon=True).start()
        else:
            subscribers, warnings = connect(receiver, args.interface, args.domain, not args.state_only)
        print('Waiting for valid G1 state (up to {} s)...'.format(args.timeout), flush=True)
        deadline = time.monotonic() + args.timeout
        while not receiver.ready.wait(.1):
            if stop.is_set():
                raise RuntimeError('Stopped before a valid state arrived')
            if time.monotonic() >= deadline:
                raise RuntimeError('No valid rt/lowstate. Check interface/domain and SDK. No policy was launched.')
        if stop.is_set():
            raise RuntimeError('Stopped during startup; no policy was launched')
        atomic_json(run / 'READY.json', {'utc': utc(), 'meaning': 'valid lowstate received; not robot safety qualification'})
        print('RECORDING READY — {}\nStart the policy in its usual terminal. Ctrl+C here finishes the recording.'.format(args.name), flush=True)
        if args.demo:
            print('SYNTHETIC DEMO ONLY — not measured G1 data.', flush=True)
        if command:
            env = os.environ.copy()
            env['G1_RUN_DIR'] = str(run)
            env['G1_POLICY_LOG'] = str(run / 'policy' / 'events.jsonl')
            child = subprocess.Popen(command, env=env, start_new_session=True)
            manifest['policy_pid'] = child.pid
            print('Wrapped policy PID {}. Its terminal output stays visible.'.format(child.pid), flush=True)
        handles = {kind: (run / 'telemetry' / (kind + '.jsonl')).open('a', encoding='utf-8')
                   for kind in ('state', 'command')}
        begin = time.monotonic()
        last_status = begin
        stale_since = None
        reason = 'operator_stopped'
        try:
            while not stop.is_set():
                now = time.monotonic()
                if args.seconds and now - begin >= args.seconds:
                    reason = 'duration_reached'
                    break
                if child is not None and child.poll() is not None:
                    reason = 'policy_exited'
                    break
                try:
                    row = receiver.queue.get(timeout=.1)
                    handles[row['kind']].write(json.dumps(row, allow_nan=False) + '\n')
                    counts[row['kind']] += 1
                except queue.Empty:
                    pass
                if now - last_status >= 2:
                    for handle in handles.values():
                        handle.flush()
                    last_status = now
                    age = now - (receiver.last_state or begin)
                    if age > 2 and stale_since is None:
                        stale_since = now
                        warnings.append('Live state stream went stale at monotonic {:.3f}'.format(now))
                    elif age <= 2:
                        stale_since = None
                    status = {'utc': utc(), 'state_samples': counts['state'], 'command_samples': counts['command'],
                              'last_state_age_s': max(0, age), 'stream_stale': age > 2}
                    atomic_json(run / 'status.json', status)
                    print('{} | state {} | commands {} | last state {:.2f}s ago'.format(
                        'STALE: check robot/network' if age > 2 else 'Recording',
                        counts['state'], counts['command'], max(0, age)), flush=True)
                    if shutil.disk_usage(run).free < args.reserve_gib * 1024**3:
                        raise RuntimeError('Disk reserve reached; recording stopped. Check the policy separately.')
        finally:
            with receiver.lock:
                receiver.enabled = False
            try:
                while True:
                    row = receiver.queue.get_nowait()
                    handles[row['kind']].write(json.dumps(row, allow_nan=False) + '\n')
                    counts[row['kind']] += 1
            except queue.Empty:
                pass
            finally:
                for handle in handles.values():
                    handle.flush()
                    os.fsync(handle.fileno())
                    handle.close()
    except Exception as exc:
        error = str(exc)
        reason = 'error'
        print('COLLECTION ERROR: ' + error, file=sys.stderr, flush=True)
    finally:
        stop.set()
        with receiver.lock:
            receiver.enabled = False
        for sub in subscribers:
            try:
                sub.Close()
            except Exception as exc:
                warnings.append('Subscriber close: ' + str(exc))
        if child is not None:
            if reason == 'operator_stopped' and child.poll() is None:
                try:
                    child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
            if child.poll() not in (None, 0):
                warnings.append('Wrapped policy exited with code {}'.format(child.returncode))
            # Never assume killing a generic controller is a safe robot stop.
            if child.poll() is None:
                warnings.append('Policy PID {} was still running when recording ended. Operator must stop it using its normal procedure.'.format(child.pid))
            manifest['policy_returncode'] = child.poll()
        manifest.update({'ended_utc': utc(), 'end_monotonic_s': time.monotonic(),
                         'status': 'closed_with_error' if error else 'closed', 'stop_reason': reason,
                         'error': error, 'warnings': warnings, 'receiver_stats': receiver.snapshot(),
                         'written_samples': dict(counts)})
        atomic_json(run / 'manifest.json', manifest)
        (run / 'ACTIVE.json').unlink(missing_ok=True)
        atomic_json(run / 'CLOSED.json', {'utc': utc(), 'status': manifest['status']})
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    for warning in warnings:
        print('NOTE: ' + warning, flush=True)
    report(run)
    print('Run saved: {}\nAdd labels/evidence, then: ./kit.sh export "{}"'.format(run, run), flush=True)
    return 1 if error or (child is not None and child.returncode not in (None, 0)) else 0


def percentile(values, p):
    if not values:
        return None
    v = sorted(values)
    return v[round((len(v) - 1) * p)]


def numeric(values):
    return [x for x in values if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)]


def report(run):
    run = Path(run).resolve()
    manifest = json.loads((run / 'manifest.json').read_text())
    issues = []
    count = 0
    first = last = None
    gaps = []
    tick_resets = 0
    last_tick = None
    motor_stats = [{} for _ in range(29)]
    imu_max = [None, None]
    for row in json_lines(run / 'telemetry' / 'state.jsonl', issues):
        t = row['host_monotonic_s']
        if last is not None:
            gaps.append(t - last)
        first = t if first is None else first
        last = t
        count += 1
        data = row['data']
        tick = data.get('tick')
        if tick is not None and last_tick is not None and tick < last_tick:
            tick_resets += 1
        last_tick = tick
        for i, m in enumerate(data['motors'][:29]):
            for key in ('q', 'dq', 'tau_est', 'temperature'):
                value = m.get(key)
                vals = numeric(value if isinstance(value, list) else [value])
                if vals:
                    record = motor_stats[i].setdefault(key, {'min': min(vals), 'max': max(vals), 'max_abs': max(map(abs, vals))})
                    record['min'] = min(record['min'], min(vals))
                    record['max'] = max(record['max'], max(vals))
                    record['max_abs'] = max(record['max_abs'], max(map(abs, vals)))
        rpy = data.get('imu', {}).get('rpy') or []
        for i in range(min(2, len(rpy))):
            if numeric([rpy[i]]):
                imu_max[i] = max(imu_max[i] or 0, abs(rpy[i]))
    command_counts = collections.Counter(row['topic'] for row in json_lines(run / 'telemetry' / 'command.jsonl', issues))
    events = []
    for path in sorted((run / 'events').glob('*.json')):
        events.append(json.loads(path.read_text()))
    trials = {}
    notes = []
    for event in events:
        if event.get('kind') == 'label':
            trials.setdefault(event['trial'], {}).update(event)
        elif event.get('kind') == 'trial-start':
            trials.setdefault(event['trial'], {'trial': event['trial'], 'outcome': 'unknown'})
        elif event.get('kind') == 'note':
            notes.append(event)
    outcomes = collections.Counter(e.get('outcome', 'unknown') for e in trials.values())
    eligible = sum(outcomes[k] for k in ('success', 'partial', 'failure', 'aborted'))
    duration = (last - first) if count > 1 else 0
    policy_rows = sum(1 for _ in json_lines(run / 'policy' / 'events.jsonl', issues))
    warnings = list(manifest.get('warnings', [])) + issues
    if count == 0:
        warnings.append('NO ROBOT STATE RECORDED')
    if not command_counts:
        warnings.append('No command messages captured; policy may use another interface or may not have run.')
    if manifest.get('error'):
        warnings.append(manifest['error'])
    if (run / 'ACTIVE.json').exists():
        warnings.append('Run is still active or was interrupted without finalization. This report is provisional.')
    drops = sum(s['queue_dropped'] for s in manifest.get('receiver_stats', {}).values())
    decode_errors = sum(s['decode_errors'] for s in manifest.get('receiver_stats', {}).values())
    if drops or decode_errors:
        warnings.append('Collector queue drops: {}; decode errors: {}'.format(drops, decode_errors))
    if gaps and max(gaps) > .2:
        warnings.append('At least one saved state gap exceeded 200 ms. Inspect continuity before analysis.')
    summary = {'schema': 'g1-session-report.v1', 'name': manifest['name'], 'source': manifest['source'],
               'policy_label': manifest['policy_label'], 'status': manifest['status'], 'warnings': warnings,
               'state_samples': count, 'sampled_duration_s': duration,
               'effective_saved_state_hz': (count - 1) / duration if duration else None,
               'saved_state_gap_p95_ms': percentile(gaps, .95) * 1000 if gaps else None,
               'saved_state_gap_max_ms': max(gaps) * 1000 if gaps else None,
               'saved_state_gaps_over_200ms': sum(x > .2 for x in gaps),
               'tick_decreases_or_wraps': tick_resets, 'collector_queue_drops': drops,
               'decode_errors': decode_errors, 'command_samples_by_topic': dict(command_counts),
               'max_abs_imu_roll_rad': imu_max[0], 'max_abs_imu_pitch_rad': imu_max[1],
               'motors_first_29': motor_stats, 'policy_hook_rows': policy_rows,
               'operator_labeled_outcomes': dict(outcomes), 'annotated_trials': len(trials),
               'operator_success_fraction_among_resolved_valid_attempts': outcomes['success'] / eligible if eligible else None,
               'operator_success_denominator': eligible,
               'task_score_limit': 'Manual exploratory labels. Denominator includes success, partial, failure, aborted; invalid and unknown excluded and shown. Unmarked attempts are not detectable.',
               'unavailable': ['automatic task/contact/landing scoring', 'camera/video unless separately attached',
                               'sensor-to-action latency', 'true applied motor commands', 'calibrated ground truth',
                               'automatic Arena download or publication'],
               'clock_limit': 'All sample timestamps are collector host receipt times. Saved gaps are not end-to-end latency.',
               'evidence': [e for e in events if e.get('kind') == 'evidence'],
               'trials': list(trials.values()), 'notes': notes}
    atomic_json(run / 'summary.json', summary)
    shown = lambda x: 'not available' if x is None else str(round(x, 3)) if isinstance(x, float) else str(x)
    rows = [('Policy', manifest['policy_label']), ('Source', manifest['source']), ('Run status', manifest['status']),
            ('Saved state samples', count), ('Covered seconds', duration),
            ('Effective saved Hz', summary['effective_saved_state_hz']),
            ('Largest saved state gap (ms)', summary['saved_state_gap_max_ms']),
            ('Command samples', dict(command_counts)), ('Collector queue drops', drops),
            ('Decode errors', decode_errors), ('Policy hook rows', policy_rows),
            ('Annotated outcomes', dict(outcomes)), ('Resolved valid attempts', eligible),
            ('Operator-labeled success fraction', summary['operator_success_fraction_among_resolved_valid_attempts'])]
    title = ('DEMO — ' if manifest['source'] == 'synthetic_demo' else '') + manifest['name']
    motor_rows = ''.join('<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>'.format(i,
        shown(m.get('dq', {}).get('max_abs')), shown(m.get('tau_est', {}).get('max_abs')),
        shown(m.get('temperature', {}).get('max'))) for i, m in enumerate(motor_stats))
    page = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>{title}</title><style>body{{font:16px system-ui;max-width:1000px;margin:40px auto;padding:0 20px;color:#142536;background:#f6f8fa}}h1{{font-size:32px}}table{{border-collapse:collapse;width:100%;background:white}}td,th{{padding:10px;text-align:left;border-bottom:1px solid #ddd}}.notice{{padding:16px;background:#fff0c2;border-radius:8px}}small{{color:#526372}}pre{{white-space:pre-wrap}}</style>
<h1>{title}</h1><p>G1 session report · kit {version}</p>
<p class="notice">This is a telemetry and operator-label report. It is not an independently verified policy benchmark. No automatic task scoring or camera capture is performed.</p>
<h2>Run overview</h2><table>{rows}</table><h2>Data quality notes</h2><ul>{warnings}</ul>
<h2>Observed motor ranges</h2><p>Raw DDS slots 0–28. These values are measurements, not pass/fail thresholds. Torque is estimated. Temperature retains both sensors in raw data; maximum shown here.</p>
<table><tr><th>Slot</th><th>Max |velocity| rad/s</th><th>Max |estimated torque| Nm</th><th>Max temperature °C</th></tr>{motors}</table>
<h2>Labels and notes</h2><pre>{notes}</pre><h2>Interpretation</h2><p>{limit}</p><p>Timing uses collector receipt timestamps. Gaps describe saved sampling continuity; they do not measure policy inference or sensor-to-action latency. Commands are observed DDS requests, not proof of motor application.</p>
<p>Read summary.json and manifest.json for full provenance, quality counters and missing measurements. Raw messages are under telemetry/; attachments under evidence/.</p></html>'''.format(title=html.escape(title), version=VERSION,
        rows=''.join('<tr><th>{}</th><td>{}</td></tr>'.format(html.escape(k), html.escape(shown(v))) for k, v in rows),
        warnings=''.join('<li>{}</li>'.format(html.escape(w)) for w in warnings) or '<li>No collector warnings recorded; hardware qualification is still separate.</li>',
        motors=motor_rows, notes=html.escape(json.dumps({'trials': summary['trials'], 'notes': notes}, indent=2)),
        limit=html.escape(summary['task_score_limit']))
    (run / 'report.html').write_text(page, encoding='utf-8')
    return summary


def ensure_closed(run):
    manifest = json.loads((run / 'manifest.json').read_text())
    if manifest.get('policy_pid') and manifest.get('policy_returncode') is None and manifest.get('host') == socket.gethostname():
        try:
            os.kill(manifest['policy_pid'], 0)
        except ProcessLookupError:
            pass
        else:
            raise ValueError('Wrapped policy PID still exists. Stop its writers before exporting or attaching.')
    if (run / 'ACTIVE.json').exists() or not (run / 'CLOSED.json').exists():
        raise ValueError('Run is active or unfinalized. Stop recorder normally. After a crash use recover only once recorder AND policy logging have stopped.')


def seal(run):
    paths = sorted(p for p in run.rglob('*') if p.is_file() and p.name != 'checksums.json')
    if any(p.is_symlink() for p in run.rglob('*')):
        raise ValueError('Symlinks are not allowed inside a return bundle')
    atomic_json(run / 'checksums.json', {p.relative_to(run).as_posix(): digest(p) for p in paths})


def verify(run):
    expected = json.loads((run / 'checksums.json').read_text())
    actual = {p.relative_to(run).as_posix() for p in run.rglob('*') if p.is_file() and p.name != 'checksums.json'}
    if actual != set(expected):
        raise ValueError('Bundle file list differs from checksums')
    for rel, checksum in expected.items():
        path = run / rel
        if path.is_symlink() or not path.resolve().is_relative_to(run.resolve()):
            raise ValueError('Unsafe bundle path')
        if digest(path) != checksum:
            raise ValueError('Checksum mismatch: ' + rel)
    return len(expected)


def export(run, destination):
    ensure_closed(run)
    report(run)
    seal(run)
    verify(run)
    destination.mkdir(parents=True, exist_ok=True)
    archive = destination / (run.name + '_' + uuid.uuid4().hex[:6] + '.tar.gz')
    temp = archive.with_suffix('.partial')
    try:
        with tarfile.open(temp, 'w:gz', dereference=False) as tf:
            tf.add(run, arcname=run.name, recursive=True)
        # Reject changes from concurrent evidence/policy writers rather than handing back a bad snapshot.
        verify(run)
        with tarfile.open(temp, 'r:gz') as tf:
            expected = json.loads((run / 'checksums.json').read_text())
            members = {m.name: m for m in tf.getmembers() if m.isfile()}
            wanted = {run.name + '/' + p for p in expected} | {run.name + '/checksums.json'}
            if set(members) != wanted:
                raise ValueError('Files changed during export; stop writers and retry')
            for rel, checksum in expected.items():
                h = hashlib.sha256()
                f = tf.extractfile(members[run.name + '/' + rel])
                for block in iter(lambda: f.read(1024 * 1024), b''):
                    h.update(block)
                if h.hexdigest() != checksum:
                    raise ValueError('File changed during export: ' + rel)
        os.replace(temp, archive)
    finally:
        temp.unlink(missing_ok=True)
    sidecar = archive.with_name(archive.name + '.sha256')
    sidecar.write_text(digest(archive) + '  ' + archive.name + '\n')
    print('SEND BOTH FILES TO ARENA:\n{}\n{}'.format(archive, sidecar))
    return archive


def mark(args):
    run = args.run.resolve()
    if not (run / 'manifest.json').exists():
        raise ValueError('Not a run folder')
    kind = args.kind
    if kind in ('trial-start', 'label') and not args.trial:
        raise ValueError('--trial is required')
    if kind == 'label' and not args.outcome:
        raise ValueError('--outcome is required for label')
    event = {'schema': 'g1-session-event.v1', 'kind': kind, 'trial': args.trial,
             'outcome': args.outcome, 'notes': args.notes, 'operator': args.operator,
             'utc': utc(), 'host_monotonic_s': time.monotonic(), 'host': socket.gethostname()}
    atomic_json(run / 'events' / (str(time.time_ns()) + '_' + uuid.uuid4().hex + '.json'), event)
    print('Saved {} for trial {}'.format(kind, args.trial or '(session)'))


def doctor(args):
    if args.timeout <= 0:
        raise ValueError('Timeout must be positive')
    print('Python: {}\nInterfaces: {}'.format(sys.executable, ', '.join(name for _, name in socket.if_nameindex())))
    print('Checking READ-ONLY live G1 state. No policy or motor commands are started.')
    receiver = Receiver(20)
    subscribers, warnings = connect(receiver, args.interface, args.domain)
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            time.sleep(.1)
        if not receiver.ready.is_set() or time.monotonic() - receiver.last_state > 2:
            raise RuntimeError('No fresh valid rt/lowstate. Try the robot network interface explicitly; confirm DDS domain and HG G1 SDK.')
        sample = None
        while not receiver.queue.empty():
            row = receiver.queue.get_nowait()
            if row['kind'] == 'state':
                sample = row
        print('PASS: live G1 state received; {} motor slots.\n{}'.format(len(sample['data']['motors']), json.dumps(receiver.snapshot(), indent=2)))
        for warning in warnings:
            print(warning)
    finally:
        for sub in subscribers:
            sub.Close()
    return 0


def recover(run):
    if (run / 'ACTIVE.json').exists():
        active = json.loads((run / 'ACTIVE.json').read_text())
        if active['host'] == socket.gethostname():
            try:
                os.kill(active['pid'], 0)
            except ProcessLookupError:
                pass
            else:
                raise ValueError('Recorder PID still exists. Stop it normally first.')
    manifest = json.loads((run / 'manifest.json').read_text())
    manifest.update(status='recovered_incomplete', stop_reason='crash_recovery', ended_utc=utc())
    manifest.setdefault('warnings', []).append('Recovered after unclean shutdown; tail lines may be incomplete. No raw data repaired or discarded.')
    atomic_json(run / 'manifest.json', manifest)
    (run / 'ACTIVE.json').unlink(missing_ok=True)
    atomic_json(run / 'CLOSED.json', {'utc': utc(), 'status': 'recovered_incomplete'})
    report(run)
    print('Recovered as INCOMPLETE. Raw files preserved. Inspect report then export.')


def parser():
    p = argparse.ArgumentParser(description='Passive G1 collection and portable session reports; never publishes robot commands.')
    sub = p.add_subparsers(dest='action', required=True)
    for action in ('doctor', 'record'):
        s = sub.add_parser(action)
        s.add_argument('--interface', default='auto')
        s.add_argument('--domain', type=int, default=0)
        s.add_argument('--timeout', type=float, default=10)
        if action == 'record':
            s.add_argument('--name', required=True)
            s.add_argument('--policy', default='unspecified')
            s.add_argument('--robot', default='unspecified')
            s.add_argument('--operator', default='unspecified')
            s.add_argument('--notes', default='')
            s.add_argument('--root', type=Path, default=ROOT / 'runs')
            s.add_argument('--run-id', help=argparse.SUPPRESS)
            s.add_argument('--hz', type=float, default=50)
            s.add_argument('--reserve-gib', type=float, default=2)
            s.add_argument('--seconds', type=float, default=0)
            s.add_argument('--state-only', action='store_true')
            s.add_argument('--demo', action='store_true')
            s.add_argument('--artifact', type=Path, action='append', default=[])
            s.add_argument('command', nargs=argparse.REMAINDER)
    s = sub.add_parser('mark')
    s.add_argument('run', type=Path)
    s.add_argument('--kind', choices=('trial-start', 'label', 'note'), default='note')
    s.add_argument('--trial')
    s.add_argument('--outcome', choices=OUTCOMES)
    s.add_argument('--operator', required=True)
    s.add_argument('--notes', default='')
    for action in ('report', 'export', 'verify', 'recover', 'attach'):
        s = sub.add_parser(action)
        s.add_argument('run', type=Path)
        if action == 'export':
            s.add_argument('--output', type=Path, default=ROOT / 'exports')
        if action == 'recover':
            s.add_argument('--confirm-writers-stopped', action='store_true', required=True)
        if action == 'attach':
            s.add_argument('file', type=Path)
            s.add_argument('--role', default='evidence')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.action == 'record':
            return collect(args)
        if args.action == 'doctor':
            return doctor(args)
        if args.action == 'mark':
            mark(args)
        elif args.action == 'report':
            report(args.run)
            print(args.run.resolve() / 'report.html')
        elif args.action == 'export':
            export(args.run.resolve(), args.output.resolve())
        elif args.action == 'verify':
            print('VERIFIED: {} files'.format(verify(args.run.resolve())))
        elif args.action == 'attach':
            ensure_closed(args.run)
            print(copy_evidence(args.run.resolve(), args.file, args.role))
        elif args.action == 'recover':
            recover(args.run.resolve())
        return 0
    except Exception as exc:
        print('ERROR: ' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
