"""Evaluation process entry point."""

import json
import sys
import traceback
from pathlib import Path

from .cli import failure_report


def main():
    request = json.loads(Path(sys.argv[1]).read_text())
    try:
        from .evaluation import evaluate

        return evaluate(request)
    except Exception as exc:  # noqa: BLE001 - Worker boundary must persist all failures.
        traceback.print_exc()
        failure_report(Path(request["output"]), f"{type(exc).__name__}: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
