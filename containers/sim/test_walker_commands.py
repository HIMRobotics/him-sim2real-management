"""Velocity command parsing shared by both walker submission paths."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))
from command import load_schedule


def test_named_velocities_and_timed_transition(tmp_path):
    path = tmp_path / "command.yaml"
    path.write_text("forward_velocity: 0.3\nsideways_velocity: -0.1\nyaw_rate: 0.2\n")
    np.testing.assert_allclose(load_schedule(path).at(1), [0.3, -0.1, 0.2])
    path.write_text(
        "segments:\n  - seconds: 1\n    forward_velocity: 0.3\n"
        "    sideways_velocity: 0\n    yaw_rate: 0\n  - command: [0,0,0]\n"
    )
    np.testing.assert_allclose(load_schedule(path).at(0.5), [0.3, 0, 0])
    np.testing.assert_array_equal(load_schedule(path).at(1), [0, 0, 0])


@pytest.mark.parametrize(
    "contents",
    [
        "forward_velocity: 0.3",
        "command: [0,0,0]\nforward_velocity: 0.3\nsideways_velocity: 0\nyaw_rate: 0",
        "forward_velocity: .nan\nsideways_velocity: 0\nyaw_rate: 0",
    ],
)
def test_invalid_named_command_rejected(tmp_path, contents):
    path = tmp_path / "command.yaml"
    path.write_text(contents)
    with pytest.raises(ValueError):
        load_schedule(path)
