"""Optional policy-side events. Call from a logging worker, not a hard realtime loop."""
import json
import os
import time


def log_event(kind, **fields):
    """Append a bounded JSON event, returning False on failure. No robot SDK dependency.

    The wrapper sets G1_POLICY_LOG. With separate terminals set it explicitly.
    One policy logging writer per run is recommended. This does filesystem I/O.
    """
    path = os.environ.get('G1_POLICY_LOG')
    if not path:
        return False
    fd = None
    try:
        payload = {'schema': 'g1-policy-event.v1', 'kind': kind,
                   'host_monotonic_s': time.monotonic(), 'host_unix_ns': time.time_ns(),
                   'fields': fields}
        raw = (json.dumps(payload, allow_nan=False) + '\n').encode()
        if len(raw) > 32768:
            return False
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
        return os.write(fd, raw) == len(raw)
    except (OSError, TypeError, ValueError):
        return False
    finally:
        if fd is not None:
            os.close(fd)
