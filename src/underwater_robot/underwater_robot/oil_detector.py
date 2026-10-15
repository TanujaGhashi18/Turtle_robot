"""
Oil-spill detector node.

Subscribes to the camera image, segments oil-like colours in HSV space and
publishes:

    /oil/detected    std_msgs/Bool     True while an oil-like patch is visible
    /oil/confidence  std_msgs/Float64  fraction of matching pixels (0..1)

Every confirmed detection is appended to a CSV log with both a wall-clock
timestamp (``time.time()``) and the message header stamp, plus the vehicle
depth at the time of detection.  Duplicate rows are suppressed: a row is
written on the rising edge of a detection and then at most once every
``min_log_interval`` seconds.

Thresholds are uncalibrated simulation values (see detection.py).
"""

import csv
import os
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64

from .detection import DEFAULT_HSV_HIGH, DEFAULT_HSV_LOW, oil_fraction, should_log


class OilDetector(Node):
    def __init__(self):
        super().__init__('oil_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detected_topic', '/oil/detected')
        self.declare_parameter('confidence_topic', '/oil/confidence')
        self.declare_parameter('depth_topic', '/depth')
        # Minimum detected-oil pixel fraction before a positive is reported.
        # 0.05 was chosen after observing that the true oil patch in the
        # sim camera reads ~0.9 confidence while stray dark pixels (propeller
        # blades, housing edges in the centre crop) stay below ~0.05.
        # PROVISIONAL, uncalibrated threshold -- see docs/architecture.md.
        self.declare_parameter('min_oil_fraction', 0.05)
        self.declare_parameter('min_log_interval', 2.0)
        self.declare_parameter('hsv_low', list(DEFAULT_HSV_LOW))
        self.declare_parameter('hsv_high', list(DEFAULT_HSV_HIGH))
        self.declare_parameter(
            'log_file', os.path.expanduser('~/ros2_ws/oil_detections.csv')
        )

        self.min_oil_fraction = float(self.get_parameter('min_oil_fraction').value)
        self.min_log_interval = float(self.get_parameter('min_log_interval').value)
        self.hsv_low = tuple(int(v) for v in self.get_parameter('hsv_low').value)
        self.hsv_high = tuple(int(v) for v in self.get_parameter('hsv_high').value)
        self.log_file = os.path.expanduser(str(self.get_parameter('log_file').value))

        self.bridge = CvBridge()
        self._depth = 0.0
        self._was_detected = False
        self._last_log_time = 0.0
        self._log_ready = self._open_log()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, qos
        )
        self.create_subscription(
            Float64, self.get_parameter('depth_topic').value, self._on_depth, 10
        )
        self.pub_detected = self.create_publisher(
            Bool, self.get_parameter('detected_topic').value, 10
        )
        self.pub_conf = self.create_publisher(
            Float64, self.get_parameter('confidence_topic').value, 10
        )

        self.get_logger().info(
            'oil_detector up: hsv %s..%s, min_fraction=%.3f, log=%s'
            % (self.hsv_low, self.hsv_high, self.min_oil_fraction, self.log_file)
        )

    # ------------------------------------------------------------------ logging
    def _open_log(self) -> bool:
        try:
            os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
            new = not os.path.exists(self.log_file)
            if new:
                with open(self.log_file, 'w', newline='') as fh:
                    csv.writer(fh).writerow(
                        [
                            'wall_time_iso',
                            'wall_time_unix',
                            'stamp_sec',
                            'stamp_nanosec',
                            'confidence',
                            'depth_m',
                        ]
                    )
            return True
        except Exception as exc:
            self.get_logger().error("cannot open log file '%s': %s" % (self.log_file, exc))
            return False

    def _write_row(self, confidence: float, stamp) -> None:
        if not self._log_ready:
            return
        try:
            with open(self.log_file, 'a', newline='') as fh:
                csv.writer(fh).writerow(
                    [
                        time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime()),
                        '%.3f' % time.time(),
                        stamp.sec,
                        stamp.nanosec,
                        '%.5f' % confidence,
                        '%.4f' % self._depth,
                    ]
                )
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn('failed writing detection log: %s' % exc)

    # ------------------------------------------------------------------ callbacks
    def _on_depth(self, msg: Float64) -> None:
        self._depth = float(msg.data)

    def _on_image(self, msg: Image) -> None:
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().warn('cv_bridge failed: %s' % exc, throttle_duration_sec=10.0)
            return

        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        fraction = oil_fraction(hsv, self.hsv_low, self.hsv_high)
        detected = fraction >= self.min_oil_fraction

        b = Bool()
        b.data = bool(detected)
        self.pub_detected.publish(b)
        c = Float64()
        c.data = float(fraction)
        self.pub_conf.publish(c)

        now = time.time()
        if should_log(now, self._last_log_time, detected, self._was_detected,
                      self.min_log_interval):
            self._write_row(fraction, msg.header.stamp)
            self._last_log_time = now
            self.get_logger().info(
                'OIL DETECTED confidence=%.4f depth=%.2f m' % (fraction, self._depth)
            )
        self._was_detected = detected


def main(args=None):
    rclpy.init(args=args)
    node = OilDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
