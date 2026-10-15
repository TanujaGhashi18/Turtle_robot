"""
GPS adapter for the turtle robot.

Subscribes to the raw ``sensor_msgs/NavSatFix`` produced by the world's
``gz-sim-navsat-system`` and re-publishes it on ``/gps/fix`` with a *correct*
fix status: a GNSS receiver cannot obtain a fix under water, so whenever the
vehicle is deeper than ``submerged_depth`` the status is set to
``STATUS_NO_FIX`` and the position covariance is inflated.

The upstream lat/lon/alt values are still forwarded (they are useful for
logging / for the surface case) but must be treated as invalid while the
status says NO_FIX.  This mirrors how a real GNSS+MS5837 combination behaves.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64


class GpsAdapter(Node):
    def __init__(self):
        super().__init__('gps_adapter')

        self.declare_parameter('submerged_depth', 0.30)
        self.declare_parameter('input_topic', '/navsat')
        self.declare_parameter('output_topic', '/gps/fix')
        self.declare_parameter('depth_topic', '/depth')

        self.submerged_depth = float(self.get_parameter('submerged_depth').value)
        self._depth = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            NavSatFix, self.get_parameter('input_topic').value, self._on_fix, qos
        )
        self.create_subscription(
            Float64, self.get_parameter('depth_topic').value, self._on_depth, 10
        )
        self.pub = self.create_publisher(
            NavSatFix, self.get_parameter('output_topic').value, 10
        )

        self.get_logger().info(
            'gps_adapter up: fixes marked INVALID deeper than %.2f m' % self.submerged_depth
        )

    def _on_depth(self, msg: Float64) -> None:
        self._depth = float(msg.data)

    def _on_fix(self, msg: NavSatFix) -> None:
        out = NavSatFix()
        out.header = msg.header
        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude
        out.position_covariance_type = msg.position_covariance_type

        if self._depth > self.submerged_depth:
            out.status.status = NavSatStatus.STATUS_NO_FIX
            out.status.service = msg.status.service
            # Effectively "unknown": large diagonal covariance.
            cov = list(msg.position_covariance)
            for i in (0, 4, 8):
                cov[i] = 1.0e6
            out.position_covariance = cov
        else:
            out.status.status = NavSatStatus.STATUS_FIX
            out.status.service = msg.status.service
            out.position_covariance = msg.position_covariance

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GpsAdapter()
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
