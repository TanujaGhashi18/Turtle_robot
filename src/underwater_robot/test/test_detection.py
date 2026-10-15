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

"""Unit tests for the oil-detection helpers."""

import numpy as np

from underwater_robot.detection import (
    DEFAULT_HSV_HIGH,
    DEFAULT_HSV_LOW,
    oil_fraction,
    should_log,
)


def _hsv_image(pixel, shape=(20, 20, 3)):
    img = np.zeros(shape, dtype=np.uint8)
    img[:, :] = np.array(pixel, dtype=np.uint8)
    return img


def test_oil_fraction_full_match():
    img = _hsv_image((10, 200, 100))
    assert oil_fraction(img) == 1.0


def test_oil_fraction_no_match():
    img = _hsv_image((120, 200, 200))  # blue
    assert oil_fraction(img) == 0.0


def test_oil_fraction_partial_match():
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    img[:2, :, :] = np.array((10, 200, 100), dtype=np.uint8)
    assert abs(oil_fraction(img) - 0.2) < 1e-9


def test_oil_fraction_empty_is_zero():
    assert oil_fraction(np.zeros((0, 0, 3), dtype=np.uint8)) == 0.0


def test_default_bounds_are_tuples_of_three():
    assert len(DEFAULT_HSV_LOW) == 3
    assert len(DEFAULT_HSV_HIGH) == 3


def test_should_log_rising_edge_and_periodic():
    # rising edge is always logged
    assert should_log(0.0, 0.0, True, False, 2.0) is True
    # continuing detection is throttled
    assert should_log(1.0, 0.0, True, True, 2.0) is False
    # ... and logged again after the interval
    assert should_log(2.5, 0.0, True, True, 2.0) is True


def test_should_log_never_logs_without_detection():
    assert should_log(10.0, 0.0, False, True, 2.0) is False
    assert should_log(10.0, 0.0, False, False, 2.0) is False
