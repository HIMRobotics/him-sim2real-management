"""Prepared policy containers must work without imports from the checkout."""

import site
import subprocess
import sys

import pytest

from containers.examples.prepare import prepare


def test_prepared_walker_loads_with_only_packaged_modules(tmp_path):
    pytest.importorskip("onnxruntime")
    context = prepare("walker", tmp_path / "walker") / "container"
    result = subprocess.run(
        [sys.executable, "-I", "-c", """
import sys
from pathlib import Path
context = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(context))
sys.path[1:1] = sys.argv[2:]
from controllers import ExamplePolicy
import robot_safety.timing_profile as timing
assert Path(timing.__file__).parent == context / 'robot_safety'
ExamplePolicy('walker', context / 'project')
""", str(context), site.getusersitepackages()],
        cwd=tmp_path, capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
