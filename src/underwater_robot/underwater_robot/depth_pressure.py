"""
Depth / pressure sensor adapter.

Models an MS5837-class depth sensor by converting the simulated vehicle
height above the sea floor into a water depth and a hydrostatic pressure:

    depth    = surface_z - z_body          [m, positive below the surface]
    pressure = atm + rho * g * depth       [Pa]

Both the depth (``std_msgs/Float64`` on ``/depth``) and the pressure
(``sensor_msgs/FluidPressure`` on ``/depth/pressure``) are published.

This is an honestly-labelled *simulation adapter*: the value is derived from
ground-truth odometry rather than from an emulated I2C device, and no real
sensor noise model is applied (see docs/architecture.md).
"""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64


class DepthPressure(Node):
    def __init__(self):
        super().__init__('depth_pressure')

        self.declare_parameter('surface_z', 0.0)
        self.declare_parameter('fluid_density', 1000.0)
        self.declare_parameter('atmospheric_pressure', 101325.0)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('odom_topic', '/odometry')

        self.surface_z = float(self.get_parameter('surface_z').value)
        self.rho = float(self.get_parameter('fluid_density').value)
        self.atm = float(self.get_parameter('atmospheric_pressure').value)
        self.g = float(self.get_parameter('gravity').value)
        odom_topic = self.get_parameter('odom_topic').value

        self._z = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        self.pub_depth = self.create_publisher(Float64, '/depth', 10)
        self.pub_press = self.create_publisher(FluidPressure, '/depth/pressure', 10)

        rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            'depth_pressure up: rho=%.1f kg/m^3, atm=%.0f Pa, surface_z=%.2f m'
            % (self.rho, self.atm, self.surface_z)
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._z = msg.pose.pose.position.z

    def _publish(self) -> None:
        if self._z is None:
            return
        depth = self.surface_z - self._z
        pressure = self.atm + self.rho * self.g * depth

        d = Float64()
        d.data = float(depth)
        self.pub_depth.publish(d)

        p = FluidPressure()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'base_link'
        p.fluid_pressure = float(pressure)
        p.variance = 0.0  # ideal simulation value, no noise model
        self.pub_press.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = DepthPressure()
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
