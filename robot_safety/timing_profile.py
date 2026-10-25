"""Optional per-thread timing aggregates, with no file I/O on control workers."""

import os
import threading
import time
from bisect import bisect_right
from contextlib import nullcontext
from functools import wraps

WALL_BINS_S = (0.001, 0.002, 0.005, 0.010, 0.020, 0.040, 0.080)


class TimingProfiler:
    """Aggregate inclusive wall and calling-thread CPU time once per second."""

    def __init__(self, emit, period=1.0):
        self.emit = emit
        self.period = period
        self.local = threading.local()
        self.errors = 0

    def state(self):
        if not hasattr(self.local, "data"):
            self.local.data = {"start": time.monotonic(), "rows": {}, "stack": []}
        return self.local.data

    def record(self, name, wall, cpu, **metrics):
        try:
            state = self.state()
            row = state["rows"].get(name)
            if row is None:
                row = {"count": 0, "wall_s": 0.0, "cpu_s": 0.0, "wall_max_s": 0.0,
                       "wall_histogram": [0] * (len(WALL_BINS_S) + 1)}
                state["rows"][name] = row
            row["count"] += 1
            row["wall_s"] += wall
            row["cpu_s"] += cpu
            row["wall_histogram"][bisect_right(WALL_BINS_S, wall)] += 1
            if wall >= row["wall_max_s"]:
                row["wall_max_s"] = wall
                row["slowest_finished_s"] = time.monotonic()
            for key, value in metrics.items():
                row[key + "_sum"] = row.get(key + "_sum", 0.0) + value
                row[key + "_max"] = max(row.get(key + "_max", 0.0), value)
            if time.monotonic() - state["start"] >= self.period:
                self.flush()
        except Exception:  # Optional diagnostics must not alter control decisions.
            self.errors += 1

    def flush(self):
        state = self.state()
        if not state["rows"]:
            return
        now = time.monotonic()
        record = {"event": "timing_profile", "version": 1, "mono_s": now,
                  "window_start_s": state["start"], "pid": os.getpid(),
                  "tid": threading.get_native_id(),
                  "thread": threading.current_thread().name,
                  "wall_histogram_edges_s": WALL_BINS_S,
                  "stages": state["rows"], "profiler_errors": self.errors}
        state["rows"] = {}
        state["start"] = now
        self.emit(record)

    def scope(self, name):
        return _Scope(self, name)


class _Scope:
    def __init__(self, profiler, name):
        self.profiler = profiler
        self.name = name

    def __enter__(self):
        stack = self.profiler.state()["stack"]
        self.key = (stack[-1] + "/" if stack else "") + self.name
        stack.append(self.key)
        self.wall = time.monotonic()
        self.cpu = time.thread_time()

    def __exit__(self, kind, value, traceback):
        cpu = time.thread_time() - self.cpu
        wall = time.monotonic() - self.wall
        self.profiler.state()["stack"].pop()
        self.profiler.record(self.key, wall, cpu, failures=int(kind is not None))


def scope(owner, name):
    profiler = getattr(owner, "timing_profiler", None)
    return nullcontext() if profiler is None else profiler.scope(name)


def profiled(name):
    def decorate(method):
        @wraps(method)
        def call(self, *args, **kwargs):
            if getattr(self, "timing_profiler", None) is None:
                return method(self, *args, **kwargs)
            with scope(self, name):
                return method(self, *args, **kwargs)
        return call
    return decorate


class _TimedLock:
    def __init__(self, profiler, lock, name):
        self.profiler, self.lock, self.name = profiler, lock, name

    def __enter__(self):
        stack = self.profiler.state()["stack"]
        self.wait_key = (stack[-1] + "/" if stack else "") + self.name + ".wait"
        started, cpu = time.monotonic(), time.thread_time()
        try:
            result = self.lock.__enter__()
        except BaseException:
            self.profiler.record(self.wait_key, time.monotonic() - started,
                                 time.thread_time() - cpu, failures=1)
            raise
        acquired, acquired_cpu = time.monotonic(), time.thread_time()
        self.wait_wall, self.wait_cpu = acquired - started, acquired_cpu - cpu
        self.hold = self.profiler.scope(self.name + ".hold")
        self.hold.__enter__()
        self.hold.wall, self.hold.cpu = acquired, acquired_cpu
        return result

    def __exit__(self, kind, value, traceback):
        # Release the real lock before publishing any aggregate.
        try:
            return self.lock.__exit__(kind, value, traceback)
        finally:
            self.hold.__exit__(kind, value, traceback)
            self.profiler.record(self.wait_key, self.wait_wall, self.wait_cpu)


def timed_lock(owner, lock, name):
    profiler = getattr(owner, "timing_profiler", None)
    return lock if profiler is None else _TimedLock(profiler, lock, name)


def timed_wait(owner, event, delay, deadline, name):
    """Observe the existing wait without changing its requested duration."""
    profiler = getattr(owner, "timing_profiler", None)
    if profiler is None:
        return event.wait(delay)
    started, cpu = time.monotonic(), time.thread_time()
    stopped = event.wait(delay)
    finished = time.monotonic()
    profiler.record(name, finished - started, time.thread_time() - cpu,
                    requested_s=delay,
                    deadline_late_s=max(0.0, started - deadline),
                    deadline_missed=int(started >= deadline),
                    wake_late_s=max(0.0, finished - (started + delay)),
                    interrupted=int(stopped))
    return stopped
