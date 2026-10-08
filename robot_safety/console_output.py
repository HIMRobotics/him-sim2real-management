"""Keep hardware telemetry in the log and show controller notices and errors."""

import json
import sys
import time


class HardwareConsole:
    """Filter policy stdout without changing commands or aborts."""

    def __init__(self, started=None):
        self.started = time.monotonic() if started is None else started
        self.last_count = 0
        self.bundle = False

    def line(self, line):
        """Return a terminal line, or None for routine log records."""
        stripped = line.strip()
        if stripped.startswith("Bundle hashes: "):
            self.bundle = not stripped.endswith("}")
            return None
        if self.bundle:
            if stripped == "}":
                self.bundle = False
            elif not stripped.startswith('"'):
                self.bundle = False
                return line
            return None
        if not stripped or stripped in (
            "Checking hardware readiness before arming",
            "Waiting for stationary hardware readiness before arming",
        ):
            return None
        try:
            record = json.loads(line)
        except ValueError:
            return line
        if (not isinstance(record, dict) or not isinstance(record.get("state"), dict)
                or not isinstance(record.get("command"), dict) or "inference_secs" not in record
                or "error" in record):
            return line
        state = record["state"]
        count, stamp = state.get("left_b_count", 0), state.get("left_b_at_secs")
        if type(count) is not int or count < 0:
            return None
        changed = count > self.last_count
        self.last_count = count
        if changed and isinstance(stamp, (int, float)) and stamp >= self.started:
            return "D-pad Left + B detected\n"
        return None


def main():
    console = HardwareConsole()
    for line in sys.stdin:
        notice = console.line(line)
        if notice is not None:
            print(notice, end="", flush=True)


if __name__ == "__main__":
    main()
