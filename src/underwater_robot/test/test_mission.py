# Copyright 2026 Tanuja Ghashi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the mission guidance helpers."""

import math

import pytest

from underwater_robot.mission import (
    clamp_norm,
    depth_hold,
    heading_to,
    lawnmower_waypoints,
    pitch_from_quaternion,
    wrap_angle,
    yaw_from_quaternion,
)


def test_wrap_angle():
    assert abs(wrap_angle(0.3) - 0.3) < 1e-9
    assert abs(wrap_angle(2 * math.pi + 0.3) - 0.3) < 1e-6
    assert abs(abs(wrap_angle(4.0)) - (2 * math.pi - 4.0)) < 1e-6


def test_yaw_from_quaternion():
    # 90 deg about z
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert abs(yaw - math.pi / 2) < 1e-6


def test_pitch_from_quaternion():
    # 30 deg about y
    pitch = pitch_from_quaternion(0.0, math.sin(math.pi / 12), 0.0, math.cos(math.pi / 12))
    assert abs(pitch - math.pi / 6) < 1e-6


def test_heading_to():
    assert abs(heading_to(1.0, 0.0)) < 1e-9
    assert abs(heading_to(0.0, 1.0) - math.pi / 2) < 1e-9
    assert abs(heading_to(-1.0, 0.0) - math.pi) < 1e-6


def test_clamp_norm():
    assert clamp_norm(0.5) == 0.5
    assert clamp_norm(3.0) == 1.0
    assert clamp_norm(-3.0) == -1.0


def test_lawnmower_covers_area_contiguously():
    wp = lawnmower_waypoints(-6, 6, -6, 6, 3.0)
    assert len(wp) == 10
    # rows alternate direction
    assert wp[0] == (-6, -6.0)
    assert wp[1] == (6, -6.0)
    assert wp[2] == (6, -3.0)
    assert wp[3] == (-6, -3.0)
    # every x is at one of the ends, every y within bounds
    for x, y in wp:
        assert x in (-6.0, 6.0)
        assert -6.0 <= y <= 6.0


def test_lawnmower_rejects_bad_arguments():
    with pytest.raises(ValueError):
        lawnmower_waypoints(0, 0, 0, 1, 1.0)
    with pytest.raises(ValueError):
        lawnmower_waypoints(0, 1, 0, 1, 0.0)


def test_depth_hold_clamps_and_scales():
    # Exact target depth and no motion -> hover.
    assert abs(depth_hold(0.0, 0.0)) < 1e-12
    # Too shallow (err < 0) with no velocity -> descend command (negative heave).
    assert depth_hold(-1.0, 0.0, depth_gain=0.6, depth_damp=0.5) < 0.0
    # Too deep (err > 0) -> ascend command (positive heave).
    assert depth_hold(1.0, 0.0, depth_gain=0.6, depth_damp=0.5) > 0.0
    # Output clamped to [-1, 1].
    assert depth_hold(6.0, 0.0) == 1.0
    assert depth_hold(-6.0, 0.0) == -1.0


def test_depth_hold_damps_vertical_velocity():
    # Sinking quickly (vz < 0) at target depth still asks to ascend (counteracts).
    assert depth_hold(0.0, -1.0, depth_gain=0.6, depth_damp=0.5) > 0.0
    # Rising quickly (vz > 0) at target depth asks to descend.
    assert depth_hold(0.0, 1.0, depth_gain=0.6, depth_damp=0.5) < 0.0
    # Damping is subtracted: with everything else fixed, faster descent raises
    # the heave command by depth_damp per m/s.
    base = depth_hold(0.0, 0.0, 0.6, 0.5)
    fast = depth_hold(0.0, -2.0, 0.6, 0.5)
    assert abs((fast - base) - 1.0) < 1e-12


def test_depth_hold_ignores_non_finite_inputs():
    assert abs(depth_hold(float('nan'), 0.0)) < 1e-12
    assert abs(depth_hold(0.0, float('inf'))) < 1e-12
    assert abs(depth_hold(float('-inf'), float('nan'))) < 1e-12
