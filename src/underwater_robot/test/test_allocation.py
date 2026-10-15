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

"""Unit tests for the thruster allocation maths."""

from underwater_robot.allocation import (
    allocate,
    clamp,
    THRUSTER_ORDER,
    zero_commands,
)


def test_order_is_canonical():
    assert THRUSTER_ORDER == (
        'front_vertical',
        'rear_vertical',
        'left_horizontal',
        'right_horizontal',
    )


def test_zero_command_gives_zero_thrust():
    assert allocate(0.0, 0.0, 0.0, 0.0, max_thrust=2.0) == [0.0, 0.0, 0.0, 0.0]
    assert zero_commands() == [0.0, 0.0, 0.0, 0.0]


def test_upward_heave_is_negative_vertical_thrust():
    # positive command == downward force, so ascend => negative numbers
    f, r, lh, rh = allocate(0.0, 1.0, 0.0, 0.0, max_thrust=2.0)
    assert f < 0.0 and r < 0.0
    assert lh == 0.0 and rh == 0.0


def test_forward_surge_uses_both_horizontal_thrusters():
    f, r, lh, rh = allocate(1.0, 0.0, 0.0, 0.0, max_thrust=2.0)
    assert lh > 0.0 and rh > 0.0
    assert f == 0.0 and r == 0.0


def test_positive_yaw_turns_left():
    # left propeller reversed, right propeller forward -> counter-clockwise
    f, r, lh, rh = allocate(0.0, 0.0, 0.0, 1.0, max_thrust=2.0)
    assert lh < 0.0 and rh > 0.0


def test_positive_pitch_is_nose_down():
    f, r, lh, rh = allocate(0.0, 0.0, 1.0, 0.0, max_thrust=2.0)
    assert f > 0.0 and r < 0.0


def test_output_is_clamped_to_max_thrust():
    out = allocate(1.0, 1.0, 1.0, 1.0, max_thrust=2.0)
    assert out == [0.0, -2.0, 0.0, 2.0]
    assert all(abs(v) <= 2.0 for v in out)


def test_clamp_helper():
    assert clamp(5.0, 2.0) == 2.0
    assert clamp(-5.0, 2.0) == -2.0
    assert clamp(0.5, 2.0) == 0.5
    assert clamp(1.0, 0.0) == 0.0
