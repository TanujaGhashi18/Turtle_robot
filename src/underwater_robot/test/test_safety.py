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

"""Unit tests for the actuator safety helpers."""

from underwater_robot.safety import clamp_commands, StaleCommandGuard


def test_clamp_commands_limits_every_element():
    assert clamp_commands([5.0, -5.0, 0.5, -0.5], 2.0) == [2.0, -2.0, 0.5, -0.5]


def test_clamp_commands_pads_missing_thrusters_with_zero():
    assert clamp_commands([1.0, 2.0], 2.0) == [1.0, 2.0, 0.0, 0.0]


def test_clamp_commands_truncates_extra_values():
    assert clamp_commands([1.0, 1.0, 1.0, 1.0, 99.0], 2.0) == [1.0, 1.0, 1.0, 1.0]


def test_guard_is_fresh_then_stale():
    guard = StaleCommandGuard(timeout=0.5, now=100.0)
    assert not guard.is_stale(100.4)
    guard.mark_valid(100.4)
    assert not guard.is_stale(100.8)
    assert guard.is_stale(100.91)


def test_guard_timeout_boundary():
    guard = StaleCommandGuard(timeout=1.0, now=0.0)
    assert not guard.is_stale(1.0)
    assert guard.is_stale(1.0001)
