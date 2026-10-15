"""
Thruster allocation maths for the bio-inspired turtle robot.

This module contains *pure* functions only (no ROS imports) so that the
allocation and clamping behaviour can be unit tested without a simulator.

Thruster geometry (from urdf/final_robot.urdf, base_link frame,
+x forward, +y left, +z up):

    front_vertical    at x = +0.062                              -> pushes along  z
    rear_vertical     at x = -0.205                              -> pushes along  z
    left_horizontal   at x = -0.046, y = +0.133                  -> pushes along  x
    right_horizontal  at x = -0.046, y = -0.133                  -> pushes along  x

Sign conventions verified empirically in Gazebo Harmonic:
    * A POSITIVE vertical-thruster command produces a force in -z (downwards).
    * A POSITIVE horizontal-thruster command produces a force in +x (forwards).

Input command convention (geometry_msgs/Twist, ROS axes):
    linear.x    surge,   positive = forwards        (+x)
    linear.z    heave,   positive = ascend          (+z)
    angular.y   pitch,   positive = nose DOWN       (+y)
    angular.z   yaw,     positive = turn LEFT / CCW (+z)

Each input is interpreted as a NORMALISED body command in ``[-1, +1]`` and is
scaled by ``max_thrust`` to obtain a per-thruster thrust in Newtons.  This is
a control allocation, not a physical velocity command: the vehicle does not
have enough thrusters to realise an arbitrary Twist, so the allocator covers
only 4 controllable degrees of freedom (surge, heave, pitch, yaw).  Sway
(linear.y) and roll (angular.x) are NOT actuated and are deliberately ignored.
"""

from typing import List

# Canonical publish order used across the package and the bridge config.
THRUSTER_ORDER = (
    'front_vertical',
    'rear_vertical',
    'left_horizontal',
    'right_horizontal',
)


def clamp(value: float, limit: float) -> float:
    """Clamp ``value`` to the symmetric range ``[-limit, +limit]``."""
    if limit <= 0.0:
        return 0.0
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def allocate(
    surge: float,
    heave: float,
    pitch: float,
    yaw: float,
    max_thrust: float = 2.0,
) -> List[float]:
    """
    Map a normalised body command to four thruster thrust commands.

    ``surge``/``heave``/``pitch``/``yaw`` are expected in ``[-1, +1]``.
    Returns a list in :data:`THRUSTER_ORDER`, each element in Newtons and
    clamped to ``[-max_thrust, +max_thrust]``.
    """
    # Vertical thrusters.  Positive command == downward force, so an upward
    # heave request is negated.  A nose-down pitch (positive angular.y) is
    # produced by pushing the front down and the rear up.
    front_vertical = (-heave + pitch) * max_thrust
    rear_vertical = (-heave - pitch) * max_thrust

    # Horizontal thrusters.  Both point forwards for positive command.  A
    # left (CCW) yaw request spins the left propeller backwards.
    left_horizontal = (surge - yaw) * max_thrust
    right_horizontal = (surge + yaw) * max_thrust

    raw = [front_vertical, rear_vertical, left_horizontal, right_horizontal]
    return [clamp(v, max_thrust) for v in raw]


def zero_commands() -> List[float]:
    """Return an all-zero command list (safe stop)."""
    return [0.0] * len(THRUSTER_ORDER)
