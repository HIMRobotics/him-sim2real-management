"""Read Linux resource counters for the passive telemetry recorder."""

import json
import os
import signal
import time
from pathlib import Path


def proc_stat(text):
    """Parse stat without splitting a process name containing spaces."""
    left, right = text.index('('), text.rindex(')')
    fields = text[right + 2:].split()
    return {
        'name': text[left + 1:right], 'state': fields[0],
        'user_ticks': int(fields[11]), 'system_ticks': int(fields[12]),
        'priority': int(fields[15]), 'nice': int(fields[16]),
        'start_ticks': int(fields[19]), 'last_cpu': int(fields[36]),
        'rt_priority': int(fields[37]), 'policy': int(fields[38]),
    }


def read_text(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def thread_snapshot(path):
    """Read cumulative CPU, scheduling and context-switch counters."""
    stat = proc_stat((path / 'stat').read_text())
    stat['tid'] = int(path.name)
    sched = read_text(path / 'schedstat')
    stat['schedstat'] = None if sched is None else [int(value) for value in sched.split()]
    status = read_text(path / 'status')
    fields = dict(line.split(':', 1) for line in (status or '').splitlines() if ':' in line)
    for key in ('voluntary_ctxt_switches', 'nonvoluntary_ctxt_switches'):
        value = fields.get(key)
        stat[key] = int(value) if value is not None else None
    stat['allowed_cpus'] = fields.get('Cpus_allowed_list', '').strip()
    stat['wchan'] = read_text(path / 'wchan')
    return stat


def machine_snapshot():
    """Read host load, frequency, thermal sensors and process CPU totals."""
    processes = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            value = proc_stat((path / 'stat').read_text())
        except (OSError, ValueError, IndexError):
            continue
        processes.append(dict(pid=int(path.name), **value))
    frequencies = {str(path): read_text(path) for path in
                   Path('/sys/devices/system/cpu').glob('cpu*/cpufreq/scaling_cur_freq')}
    thermal = [{'zone': path.name, 'type': read_text(path / 'type'),
                'temperature_millidegrees_c': read_text(path / 'temp')}
               for path in Path('/sys/class/thermal').glob('thermal_zone*')]
    throttling = {str(path): read_text(path) for path in
                  Path('/sys/devices/system/cpu').glob('cpu*/thermal_throttle/*_throttle_count')}
    return {'cpu_stat': read_text('/proc/stat'), 'loadavg': read_text('/proc/loadavg'),
            'cpu_pressure': read_text('/proc/pressure/cpu'), 'frequencies_khz': frequencies,
            'thermal': thermal, 'throttle_counters': throttling, 'processes': processes}


def cpu_worker(output, pid, seconds, hz):
    """Sample in a separate process without attaching to the controller."""
    output = Path(output)
    status_path = output / 'cpu-observer.json'
    process = Path('/proc') / str(pid)
    status = {'ready': False, 'pid': pid, 'requested_hz': hz, 'samples': 0, 'error': None,
              'clock_ticks_per_second': os.sysconf('SC_CLK_TCK'),
              'sched_schedstats': read_text('/proc/sys/kernel/sched_schedstats'),
              'observer_pid': os.getpid(), 'method': 'Read-only /proc and sysfs. No ptrace, signals to target, priority changes or affinity changes. Thread counters at requested Hz, host counters at 1 Hz. No direct GIL measurement.'}
    stopped = False

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        identity = proc_stat((process / 'stat').read_text())
        status['start_ticks'] = identity['start_ticks']
        status['cgroup'] = read_text(process / 'cgroup')
        status['cgroup_cpu_max_root'] = read_text('/sys/fs/cgroup/cpu.max')
        status['cgroup_cpu_stat_root'] = read_text('/sys/fs/cgroup/cpu.stat')
        status['cgroup_note'] = 'Root cgroup counters may differ from the target cgroup.'
        started = time.monotonic()
        deadline = started + seconds
        next_host = started
        next_tick = started
        with (output / 'cpu-samples.jsonl').open('w') as stream:
            while not stopped and time.monotonic() < deadline:
                begin = time.monotonic_ns()
                current = proc_stat((process / 'stat').read_text())
                if current['start_ticks'] != identity['start_ticks']:
                    raise RuntimeError('Target PID was reused, ending CPU capture')
                threads, unavailable = [], []
                for path in (process / 'task').iterdir():
                    try:
                        threads.append(thread_snapshot(path))
                    except (OSError, ValueError, IndexError):
                        unavailable.append(path.name)
                if not threads:
                    raise RuntimeError('Target thread counters are unreadable')
                row = {'monotonic_ns': begin, 'wall_ns': time.time_ns(), 'pid': pid,
                       'process': current, 'threads': threads, 'unavailable_tids': unavailable}
                if time.monotonic() >= next_host:
                    row['host'] = machine_snapshot()
                    next_host = time.monotonic() + 1.0
                row['collection_duration_s'] = (time.monotonic_ns() - begin) / 1e9
                stream.write(json.dumps(row, separators=(',', ':')) + '\n')
                stream.flush()
                status['samples'] += 1
                if not status['ready']:
                    status['ready'] = True
                    status_path.write_text(json.dumps(status, indent=2) + '\n')
                next_tick += 1 / hz
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
    except Exception as error:
        status['error'] = type(error).__name__ + ': ' + str(error)
    finally:
        status['end_monotonic_ns'] = time.monotonic_ns()
        status_path.write_text(json.dumps(status, indent=2) + '\n')
    return 1 if status['error'] else 0
