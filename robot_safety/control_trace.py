"""One-line traces of the whole-body command path.

The 2 ms publisher only counts. Lines are written a few times a second, and
once more when a run stops, so logging cannot become the control loop.
"""

import json
import os
import time
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread

TRACE_PERIOD_SECS = 0.2


def default_trace_path():
    """The service runs from the repo. Its imported copy lives in the ROS install."""
    override = os.environ.get("HIM_CONTROL_TRACE")
    if override:
        return Path(override)
    return Path.cwd() / ".cache" / "control-trace.jsonl"


def _round(value):
    if value is None:
        return None
    return round(float(value), 5)


class ControlTrace:
    """Append compact JSON lines for the safety node and a file."""

    def __init__(self, path, period=TRACE_PERIOD_SECS, diagnostic_path=None):
        self.path = Path(path)
        self.period = period
        self.lock = Lock()
        self.file = None
        self.last_emit = None
        self.last = None
        self.repeat_count = 0
        self.repeat_last = None
        self.repeat_gap_max = 0.0
        self.repeat_window = None
        self.gate_last = None
        self.gate_gap = None
        self.step_period_max = None
        self.diagnostic_path = diagnostic_path or os.environ.get("HIM_DIAGNOSTIC_TRACE")
        self.diagnostic_enabled = bool(self.diagnostic_path)
        self.diagnostic_dropped = 0
        self.diagnostic_errors = 0
        self._diagnostics = Queue(maxsize=4096)
        self._diagnostic_stop = Event()
        self._diagnostic_worker = None

    def open(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", encoding="utf-8")
        print("Control trace: {0}".format(self.path), flush=True)
        if self.diagnostic_enabled:
            try:
                path = Path(self.diagnostic_path)
                if path.resolve() == self.path.resolve():
                    raise ValueError("diagnostic trace must differ from control trace")
                path.parent.mkdir(parents=True, exist_ok=True)
                handle = path.open("w", encoding="utf-8")
            except (OSError, ValueError) as error:
                self.diagnostic_enabled = False
                self.diagnostic_errors += 1
                print("Optional diagnostics unavailable: {0}".format(error), flush=True)
                return
            self._diagnostic_worker = Thread(
                target=self._write_diagnostics, args=(handle,),
                name="control_diagnostics", daemon=True,
            )
            self._diagnostic_worker.start()

    def note_diagnostic(self, record):
        """Queue an owned snapshot without waiting for disk or encoding."""
        if not self.diagnostic_enabled:
            return
        try:
            self._diagnostics.put_nowait(record)
        except Full:
            self.diagnostic_dropped += 1

    def _write_diagnostics(self, handle):
        """Bound disk usage and report losses even while control is idle."""
        next_heartbeat = 0.0
        try:
            while not self._diagnostic_stop.is_set() or not self._diagnostics.empty():
                records = []
                try:
                    records.append(self._diagnostics.get(timeout=0.1))
                except Empty:
                    pass
                now = time.monotonic()
                if now >= next_heartbeat:
                    records.append({
                        "event": "diagnostic_heartbeat", "mono_s": now,
                        "wall_ns": time.time_ns(), "pid": os.getpid(),
                        "dropped": self.diagnostic_dropped,
                        "encoding_errors": self.diagnostic_errors,
                    })
                    next_heartbeat = now + 1.0
                for record in records:
                    try:
                        encoded = json.dumps(record, separators=(",", ":"), allow_nan=False)
                    except (ValueError, TypeError):
                        self.diagnostic_errors += 1
                        continue
                    if record.get("event") == "diagnostic_start":
                        Path(str(self.diagnostic_path) + ".meta.json").write_text(encoded + "\n")
                    if handle.tell() >= 128 * 1024 * 1024:
                        handle.close()
                        path = Path(self.diagnostic_path)
                        path.replace(Path(str(path) + ".1"))
                        handle = path.open("w", encoding="utf-8")
                    handle.write(encoded + "\n")
                if records:
                    handle.flush()
        except OSError:
            self.diagnostic_errors += 1
            self.diagnostic_enabled = False
        finally:
            handle.close()

    def note_repeat(self, now):
        """Count one resend. This stays off the policy tick."""
        with self.lock:
            if self.repeat_last is not None:
                self.repeat_gap_max = max(self.repeat_gap_max, now - self.repeat_last)
            self.repeat_last = now
            self.repeat_count += 1
            if self.repeat_window is None:
                self.repeat_window = now

    def note_gate(self, now):
        """Record how long the ROS motor-gate timer waited."""
        with self.lock:
            if self.gate_last is not None:
                self.gate_gap = now - self.gate_last
            self.gate_last = now

    def note_step(self, record):
        with self.lock:
            period = record.get("step_period_s")
            if period is not None:
                self.step_period_max = (
                    period if self.step_period_max is None else max(self.step_period_max, period)
                )
            self.last = record
            self._emit(record, force=False)

    def emit_abort(self, reason, now):
        with self.lock:
            record = dict(self.last or {})
            record["abort"] = reason
            record["now"] = now
            self.last = record
            self._emit(record, force=True)

    def close(self):
        self._diagnostic_stop.set()
        if self._diagnostic_worker is not None:
            self._diagnostic_worker.join(timeout=2.0)
        with self.lock:
            handle = self.file
            self.file = None
        if handle is not None:
            handle.close()

    def _emit(self, record, force):
        now = record.get("now")
        if (
            not force
            and self.last_emit is not None
            and now is not None
            and now - self.last_emit < self.period
        ):
            return
        window = self.repeat_window
        count = self.repeat_count
        gap = self.repeat_gap_max
        elapsed = None if window is None or now is None else now - window
        # Repeat stamps use the wall clock; the policy stamp is the adapter clock.
        # Rate uses the repeat clock only when both ends exist.
        if self.repeat_last is not None and window is not None and self.repeat_last > window:
            elapsed = self.repeat_last - window
        rate = None if not elapsed else count / elapsed
        line = dict(record)
        line["repeat_hz"] = _round(rate)
        line["repeat_gap_s"] = _round(gap)
        line["repeat_count"] = count
        line["gate_gap_s"] = _round(self.gate_gap)
        line["step_period_max_s"] = _round(self.step_period_max)
        self.step_period_max = None
        self.repeat_count = 0
        self.repeat_gap_max = 0.0
        self.repeat_window = self.repeat_last
        self.last_emit = now
        encoded = json.dumps(line, separators=(",", ":"), allow_nan=False)
        print(encoded, flush=True)
        if self.file is None:
            return
        try:
            self.file.write(encoded + "\n")
            self.file.flush()
        except OSError as error:
            print("Control trace write failed: {0}".format(error), flush=True)
            self.file = None
