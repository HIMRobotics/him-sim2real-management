"""Preserve partial validation when aggregating run reports."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).with_name("summary.py")


class SummaryTests(unittest.TestCase):
    def test_completed_direct_run_is_not_promoted_to_within_limits(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for pose, status, path in (
                ("standing", "completed", "direct"),
                ("seated", "within_limits", "submission"),
            ):
                directory = root / pose
                directory.mkdir()
                report = {
                    "status": status,
                    "safety_checks": True,
                    "control_path": path,
                    "checks": {"command_limits": path == "submission"},
                }
                (directory / "report.json").write_text(json.dumps(report))
            result = subprocess.run(
                [sys.executable, str(SCRIPT), tmp, "1", "", "standing", "seated"],
                capture_output=True,
                text=True,
                check=False,
                timeout=10,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads((root / "summary.json").read_text())
            self.assertEqual(summary["status"], "completed")
            self.assertEqual(summary["poses"][0]["control_path"], "direct")
            self.assertFalse(summary["poses"][0]["checks"]["command_limits"])
