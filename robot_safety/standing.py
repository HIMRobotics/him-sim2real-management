"""G1 standing geometry from measured joints and pelvis IMU orientation."""

import math

import numpy as np

from .hardware_constants import NUM_JOINTS

# Fixed transforms from packages/policy_safety/src/policy_safety/robots/g1/g1_robot.xml.
# LowState IMU is in the pelvis frame. The torso IMU is a separate SDK topic.
_HIP_FRAME_PITCH = 2 * math.atan2(0.0873386, 0.996179)
_JOINT_AXES = (1, 0, 2, 1, 1, 0)
# Endpoints of the 7 sole capsules per foot. A capsule's lowest point is at an
# endpoint, so these points plus the radius bound the sole exactly.
_SOLE_SEGMENTS = [
    [(0.1, -0.026), (0.05, -0.026)], [(-0.044, -0.018), (0.123, -0.018)],
    [(-0.052, -0.01), (0.13, -0.01)], [(-0.054, 0.0), (0.132, 0.0)],
    [(-0.052, 0.01), (0.13, 0.01)], [(-0.044, 0.018), (0.123, 0.018)],
    [(0.1, 0.026), (0.05, 0.026)],
]
# The upstream left sole's outer back endpoint sits 1 mm further out.
_FOOT_POINTS = tuple(
    np.array([[x, y, -0.025] for segment in segments for x, y in segment])
    for segments in ([[(0.1, -0.026), (0.05, -0.027)]] + _SOLE_SEGMENTS[1:], _SOLE_SEGMENTS)
)
_FOOT_RADIUS_M = 0.01


def _unrotate(axis, angle, v):
    """Express v in a child frame rotated by angle about axis (R^T v)."""
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = v
    if axis == 0:
        return x, c * y + s * z, c * z - s * y
    if axis == 1:
        return c * x - s * z, y, s * x + c * z
    return c * x + s * y, c * y - s * x, z


# Fixed pitch of each joint frame about y. Plain tuples: this runs on every
# 200 Hz state, where small numpy arrays cost more than the arithmetic.
_FIXED_PITCH = (0.0, -_HIP_FRAME_PITCH, 0.0, _HIP_FRAME_PITCH, 0.0, 0.0)
_LEG_OFFSETS = tuple((
    (0, side * 0.064452, -0.1027), (0, side * 0.052, -0.030465),
    (0.025001, 0, -0.12412), (-0.078273, side * 0.0021489, -0.17734),
    (0, -side * 9.4445e-05, -0.30001), (0, 0, -0.017558),
) for side in (1, -1))


def pelvis_to_sole_heights(q, quat):
    """Return vertical pelvis-to-sole distances for left and right feet."""
    if (len(q) != NUM_JOINTS or len(quat) != 4
            or not all(math.isfinite(v) for values in (q, quat) for v in values)):
        raise ValueError("invalid_standing_telemetry")
    norm = sum(v * v for v in quat)
    if not 0.9 <= norm <= 1.1:
        raise ValueError("invalid_standing_quaternion")
    w, x, y, z = quat
    # World-up expressed in the pelvis frame, independent of global position.
    pelvis_up = (
        2 * (x * z - w * y) / norm,
        2 * (y * z + w * x) / norm,
        1 - 2 * (x * x + y * y) / norm,
    )
    heights = []
    for leg, offsets in enumerate(_LEG_OFFSETS):
        up = pelvis_up
        height = 0.0
        for index, (offset, pitch, axis) in enumerate(zip(offsets, _FIXED_PITCH, _JOINT_AXES)):
            height -= up[0] * offset[0] + up[1] * offset[1] + up[2] * offset[2]
            if pitch:
                up = _unrotate(1, pitch, up)
            up = _unrotate(axis, q[leg * 6 + index], up)
        heights.append(height - float(np.min(_FOOT_POINTS[leg] @ up)) + _FOOT_RADIUS_M)
    return tuple(heights)


def standing_height(q, quat):
    """Estimate pelvis height above the lowest sole on level ground."""
    # Planted feet are the gantry operator's responsibility. During walking,
    # the lowest sole avoids treating a raised swing foot as lost base height.
    return max(pelvis_to_sole_heights(q, quat))
