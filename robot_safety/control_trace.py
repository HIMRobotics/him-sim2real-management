"""One-line traces of the whole-body command path.

The 2 ms publisher only counts. Lines are written a few times a second, and
once more when a run stops, so logging cannot become the control loop.
"""

import json
import os
from pathlib import Path
from threading import Lock


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

    def __init__(self, path, period=TRACE_PERIOD_SECS):
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

    def open(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", encoding="utf-8")
        print("Control trace: {0}".format(self.path), flush=True)

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
