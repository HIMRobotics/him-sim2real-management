"""Fixtures for optional simulator tests."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "robot_safety"))


@pytest.fixture(scope="session")
def bundle(tmp_path_factory):
    """Screening bundle prepared from the NVIDIA walker example."""
    pytest.importorskip("mujoco")
    pytest.importorskip("onnxruntime")
    pytest.importorskip("onnx")
    from policy_safety.bundle import prepare_g1

    sample = Path(__file__).resolve().parents[3] / "containers/examples/policies/walker"
    output = tmp_path_factory.mktemp("bundles") / "g1"
    prepare_g1(sample, output)
    return output
