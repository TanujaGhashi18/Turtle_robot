"""
Surface-demo frame saver.

Subscribes to the world overview camera (``/overview_camera``) and
writes one PNG every ``save_period`` seconds to ``~/ros2_ws/surface_demo_frames/``.
These frames are the recorded "what the simulation actually renders" evidence
for the capstone submission (robot + water surface + oil patches), independent
of the desktop/GUI compositor.

Only used by the surface demonstration launch.
"""

import os

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class SurfaceFrameSaver(Node):
    def __init__(self):
        super().__init__('surface_frame_saver')

        self.declare_parameter('image_topic', '/overview_camera')
        self.declare_parameter('save_dir',
                               os.path.expanduser('~/ros2_ws/surface_demo_frames'))
        self.declare_parameter('save_period', 3.0)
        self.declare_parameter('max_frames', 400)

        self.save_dir = os.path.expanduser(str(self.get_parameter('save_dir').value))
        self.save_period = float(self.get_parameter('save_period').value)
        self.max_frames = int(self.get_parameter('max_frames').value)

        os.makedirs(self.save_dir, exist_ok=True)
        self.bridge = CvBridge()
        self._last = None
        self._last_saved_at = 0.0
        self._count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, qos)
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            'surface_frame_saver up: saving %s every %.1f s to %s'
            % (self.get_parameter('image_topic').value, self.save_period, self.save_dir))

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._last = bgr
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn('cv_bridge failed: %s' % exc,
                                   throttle_duration_sec=10.0)

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last is None:
            return
        if now - self._last_saved_at < self.save_period:
            return
        if self._count >= self.max_frames:
            return
        path = os.path.join(self.save_dir, 'frame_%04d.png' % self._count)
        cv2.imwrite(path, self._last)
        self._count += 1
        self._last_saved_at = now
        self.get_logger().info('saved frame %s' % path, throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceFrameSaver()
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
