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


def test_preflight_preserves_container_help_and_rejects_conflicting_modes(tmp_path):
    runtime = tmp_path / 'runtime'
    calls = tmp_path / 'calls'
    runtime.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$CALLS"\necho fake-image-id\n')
    runtime.chmod(0o700)
    env = {**os.environ, 'CONTAINER_RUNTIME': str(runtime), 'CALLS': str(calls),
           'G1_IMAGE': '', 'G1_COMMAND_FILE': ''}
    args = [str(ROOT / 'setup.sh'), '--image', 'test-policy']
    result = subprocess.run(args + ['--', '--help'], env=env, capture_output=True,
                            text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr
    assert 'Preflight complete' in result.stdout
    assert calls.read_text().splitlines() == ['image inspect test-policy --format {{.Id}}']
    for extra in (['--mode', 'dry-run', '--enable-hardware-output'], ['--example', 'walker']):
        calls.unlink(missing_ok=True)
        result = subprocess.run(args + extra, env=env, capture_output=True,
                                text=True, timeout=10, check=False)
        assert result.returncode == 2
        assert not calls.exists()


def test_example_preflight_preserves_explicit_profile(tmp_path):
    import hashlib
    import json

    profile = tmp_path / 'explicit profile.json'
    data = json.loads((ROOT / 'robot_safety/profiles/g1_type5.template.json').read_text())
    data['joints']['left_hip_pitch']['kp'] = 123.0
    profile.write_text(json.dumps(data))
    runtime = tmp_path / 'runtime'
    calls = tmp_path / 'calls'
    runtime.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$CALLS"\n')
    runtime.chmod(0o700)
    result = subprocess.run(
        [str(ROOT / 'setup.sh'), '--example', 'walker', '--profile', str(profile)],
        env={**os.environ, 'CONTAINER_RUNTIME': str(runtime), 'CALLS': str(calls),
             'G1_IMAGE': '', 'G1_COMMAND_FILE': ''},
        capture_output=True, text=True, timeout=20, check=False,
    )
    assert result.returncode == 0, result.stderr
    assert hashlib.sha256(profile.read_bytes()).hexdigest() in result.stdout
    assert calls.read_text().splitlines() == [
        'build -t g1-example-walker:local ' + str(ROOT / '.cache/sim_examples/walker/container'),
        'image inspect g1-example-walker:local --format {{.Id}}',
    ]


def test_runtime_failure_reports_exit_status(tmp_path):
    runtime = tmp_path / 'runtime'
    runtime.write_text('#!/bin/sh\nexit 42\n')
    runtime.chmod(0o700)
    result = subprocess.run(
        [str(ROOT / 'setup.sh'), '--image', 'test-policy'],
        env={**os.environ, 'CONTAINER_RUNTIME': str(runtime),
             'G1_IMAGE': '', 'G1_COMMAND_FILE': ''},
        capture_output=True, text=True, timeout=10, check=False,
    )
    assert result.returncode == 42
    assert 'setup.sh failed at line' in result.stderr
    assert '(exit 42)' in result.stderr
