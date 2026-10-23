"""Check standing geometry against the robot model and known postures."""

import math
from pathlib import Path

import numpy as np
import pytest

from robot_safety.balance_policy import DEFAULT_JOINT_POSITIONS
from robot_safety.hardware_constants import (
    G1_29_JOINT_NAMES,
    G1_STANDING_PELVIS_HEIGHT_M,
    STANDING_HEIGHT_MIN_FRACTION,
    STANDING_HEIGHT_MIN_M,
)
from robot_safety.standing import pelvis_to_sole_heights, standing_height


def squat_pose(knee=2.2):
    return tuple([-knee / 2, 0, 0, knee, -knee / 2, 0] * 2 + [0] * 17)


def pose_at_height(height):
    low, high = 0.0, 2.8
    for _ in range(55):
        knee = (low + high) / 2
        if standing_height(squat_pose(knee), (1, 0, 0, 0)) > height:
            low = knee
        else:
            high = knee
    return squat_pose((low + high) / 2)


def test_reference_and_cutoff():
    assert G1_STANDING_PELVIS_HEIGHT_M == 0.74
    assert STANDING_HEIGHT_MIN_FRACTION == 0.66
    assert STANDING_HEIGHT_MIN_M == pytest.approx(0.4884)


@pytest.mark.parametrize('q, expected', [
    (np.zeros(29), 0.7918637524),
    (DEFAULT_JOINT_POSITIONS, 0.7842021826),
    (squat_pose(), 0.4440219775),
    ([-math.pi / 2, 0, 0, math.pi / 2, 0, 0] * 2 + [0] * 17, 0.4552703309),
])
def test_standing_squat_and_seated_heights(q, expected):
    assert pelvis_to_sole_heights(q, (1, 0, 0, 0)) == pytest.approx((expected, expected), abs=1e-6)


def test_raised_swing_foot_does_not_reduce_standing_height():
    q = list(squat_pose())
    q[:6] = [0] * 6
    left, right = pelvis_to_sole_heights(q, (1, 0, 0, 0))
    assert left > STANDING_HEIGHT_MIN_M > right
    assert standing_height(q, (1, 0, 0, 0)) == left


@pytest.mark.parametrize('q, quat', [
    ([0] * 28, (1, 0, 0, 0)),
    ([float('nan')] + [0] * 28, (1, 0, 0, 0)),
    ([0] * 29, (0, 0, 0, 0)),
    ([0] * 29, (1, 0, float('inf'), 0)),
])
def test_invalid_geometry_inputs_fail_closed(q, quat):
    with pytest.raises(ValueError):
        standing_height(q, quat)


def test_geometry_matches_mujoco_for_asymmetric_joints_and_orientation():
    mujoco = pytest.importorskip('mujoco')
    path = Path(__file__).resolve().parents[1] / (
        'packages/policy_safety/src/policy_safety/robots/g1/g1_robot.xml')
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    rng = np.random.default_rng(42)
    for _ in range(100):
        q = rng.uniform(-0.5, 0.5, 29)
        q[[3, 9]] = rng.uniform(0, 2.5, 2)
        quat = rng.normal(size=4)
        quat /= np.linalg.norm(quat)
        data.qpos[:] = model.qpos0
        data.qpos[:3] = rng.normal(size=3)
        data.qpos[3:7] = quat
        for name, value in zip(G1_29_JOINT_NAMES, q):
            data.qpos[model.joint(name + '_joint').qposadr[0]] = value
        mujoco.mj_forward(model, data)
        expected = []
        for side in ('left', 'right'):
            body_id = model.body(side + '_ankle_roll_link').id
            feet = [i for i in range(model.ngeom)
                    if model.geom_bodyid[i] == body_id
                    and model.geom_type[i] == mujoco.mjtGeom.mjGEOM_CAPSULE]
            assert len(feet) == 7
            sole_z = min(
                data.geom_xpos[i, 2] + sign * model.geom_size[i, 1]
                * data.geom_xmat[i].reshape(3, 3)[2, 2] - model.geom_size[i, 0]
                for i in feet for sign in (-1, 1))
            expected.append(data.body('pelvis').xpos[2] - sole_z)
        assert pelvis_to_sole_heights(q, quat) == pytest.approx(expected, abs=1e-9)
        assert pelvis_to_sole_heights(q, -quat * 1.01) == pytest.approx(expected, abs=1e-9)
