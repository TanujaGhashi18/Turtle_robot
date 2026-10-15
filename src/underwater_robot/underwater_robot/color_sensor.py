"""
Colour sensor adapter.

Emulates a TCS3200-style colour sensor by sampling the mean RGB colour of a
central region of the vehicle camera image and publishing it as a
``std_msgs/ColorRGBA`` on ``/color_sensor/rgb`` (the ``a`` field carries the
normalised brightness, 0..1).

This is an approximation: a real TCS3200 has its own optics and is normally
pointed at a specific spot, whereas here we reuse the simulated camera.  That
limitation is documented in docs/architecture.md.
"""

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA


class ColorSensor(Node):
    def __init__(self):
        super().__init__('color_sensor')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/color_sensor/rgb')
        self.declare_parameter('roi_fraction', 0.25)

        self.roi_fraction = float(self.get_parameter('roi_fraction').value)
        self.bridge = CvBridge()
        self._pub = self.create_publisher(
            ColorRGBA, self.get_parameter('output_topic').value, 10
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, qos
        )
        self.get_logger().info(
            "color_sensor up: sampling %.0f%% centre of '%s'"
            % (self.roi_fraction * 100.0, self.get_parameter('image_topic').value)
        )

    def _on_image(self, msg: Image) -> None:
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:  # pragma: no cover - depends on live driver
            self.get_logger().warn('cv_bridge conversion failed: %s' % exc,
                                   throttle_duration_sec=10.0)
            return

        h, w = img.shape[:2]
        frac = min(max(self.roi_fraction, 0.05), 1.0)
        rh, rw = max(int(h * frac), 1), max(int(w * frac), 1)
        y0, x0 = (h - rh) // 2, (w - rw) // 2
        roi = img[y0:y0 + rh, x0:x0 + rw].astype(np.float32)

        r, g, b = roi[..., 0].mean(), roi[..., 1].mean(), roi[..., 2].mean()
        out = ColorRGBA()
        out.r = float(r / 255.0)
        out.g = float(g / 255.0)
        out.b = float(b / 255.0)
        out.a = float(max(r, g, b) / 255.0)  # normalised brightness
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ColorSensor()
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
