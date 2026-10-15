"""
Pure geometry / guidance helpers for the search mission.

No ROS imports, so these can be unit tested directly.
"""

import math
from typing import List, Tuple


def clamp_norm(value: float, limit: float = 1.0) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (rotation about +z) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def pitch_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """
    Extract pitch (rotation about +y) from a quaternion.

    Positive pitch corresponds to the nose pointing DOWN (ROS convention).
    """
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


def heading_to(dx: float, dy: float) -> float:
    """Desired yaw to face a point offset (dx, dy) away."""
    return math.atan2(dy, dx)


def depth_hold(
    depth_err: float,
    vz: float,
    depth_gain: float = 0.6,
    depth_damp: float = 0.5,
) -> float:
    """
    Normalised vertical (heave, +up) command for the depth hold.

    Proportional on the depth error (positive = too deep) plus damping on the
    body vertical velocity (positive = ascending).  Clamped to ``[-1, +1]``;
    non-finite inputs are treated as zero error/velocity.
    """
    err = 0.0 if not math.isfinite(depth_err) else depth_err
    vel = 0.0 if not math.isfinite(vz) else vz
    return clamp_norm(depth_gain * err - depth_damp * vel)


def lawnmower_waypoints(
    xmin: float,
    xmax: float,
    ymin: float,
    ymax: float,
    spacing: float,
) -> List[Tuple[float, float]]:
    """
    Boustrophedon (lawnmower) survey path over an axis-aligned rectangle.

    Rows run along +x, stepping in y by ``spacing``.  Alternate rows are
    reversed so the path is continuous.
    """
    if spacing <= 0.0:
        raise ValueError('spacing must be positive')
    if xmax <= xmin or ymax <= ymin:
        raise ValueError('invalid rectangle')

    waypoints: List[Tuple[float, float]] = []
    n_rows = int(math.floor((ymax - ymin) / spacing)) + 1
    for i in range(n_rows):
        y = ymin + i * spacing
        if y > ymax:
            break
        if i % 2 == 0:
            waypoints.append((xmin, y))
            waypoints.append((xmax, y))
        else:
            waypoints.append((xmax, y))
            waypoints.append((xmin, y))
    return waypoints
