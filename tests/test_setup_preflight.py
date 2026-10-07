"""Check launcher opt-in with a fake container runtime and no robot processes."""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class SetupTests(unittest.TestCase):
    def test_preflight_and_missing_hardware_enable_never_start_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            runtime, calls = Path(tmp) / "runtime", Path(tmp) / "calls"
            runtime.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$CALLS"\necho fake-image-id\n')
            runtime.chmod(0o700)
            args = [str(ROOT / "setup.sh"), "--image", "test-image", "--profile",
                    str(ROOT / "robot_safety/profiles/g1_type5.template.json")]
            env = {**os.environ, "CONTAINER_RUNTIME": str(runtime), "CALLS": str(calls)}
            result = subprocess.run(args, env=env, capture_output=True, text=True, timeout=10, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("Preflight complete", result.stdout)
            self.assertEqual(calls.read_text().splitlines(), ['image inspect test-image --format {{.Id}}'])
            calls.unlink()
            result = subprocess.run(args + ["--mode", "hardware"], env=env,
                                    capture_output=True, text=True, timeout=10, check=False)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("--enable-hardware-output", result.stderr)
            self.assertFalse(calls.exists())
