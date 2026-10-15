"""
Pure oil-detection helpers (no ROS, no file I/O).

The detector looks for pixels whose HSV colour matches weathered crude oil:
a dark orange/brown hue with a moderate-to-high saturation.  These thresholds
are *simulation* values chosen for the oil-like target materials used in
``worlds/underwater_world.sdf`` and are not a calibrated sensor model.

OpenCV hue is in the range 0..179 (degrees / 2).  The values below correspond
to roughly 0..50 degrees, which covers orange/brown/red.
"""

from typing import Tuple

import numpy as np

# default OpenCV HSV bounds (H 0..179, S 0..255, V 0..255)
DEFAULT_HSV_LOW = (0, 100, 0)
DEFAULT_HSV_HIGH = (28, 255, 150)


def oil_fraction(
    hsv_image: 'np.ndarray',
    hsv_low: Tuple[int, int, int] = DEFAULT_HSV_LOW,
    hsv_high: Tuple[int, int, int] = DEFAULT_HSV_HIGH,
) -> float:
    """
    Fraction (0..1) of pixels inside the oil HSV window.

    ``hsv_image`` must be an ``HxWx3`` uint8 array in OpenCV HSV ordering.
    """
    if hsv_image is None or hsv_image.size == 0:
        return 0.0
    import cv2

    low = np.array(hsv_low, dtype=np.uint8)
    high = np.array(hsv_high, dtype=np.uint8)
    mask = cv2.inRange(hsv_image, low, high)
    return float(np.count_nonzero(mask)) / float(mask.size)


def should_log(
    now: float,
    last_log_time: float,
    detected: bool,
    was_detected: bool,
    min_interval: float,
) -> bool:
    """
    Decide whether a detection event should be written to the log.

    A row is logged:
      * on a rising edge (detection starts), or
      * periodically while a detection continues, every ``min_interval`` s.

    This suppresses the flood of duplicate rows that would otherwise be
    produced at camera frame rate.
    """
    if not detected:
        return False
    if not was_detected:
        return True
    return (now - last_log_time) >= min_interval
