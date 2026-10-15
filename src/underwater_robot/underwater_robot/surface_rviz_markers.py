"""
Surface-demo RViz visualizer markers.

Publishes a ``visualization_msgs/MarkerArray`` on ``/surface_markers`` for the
RViz-only view of the surface mission (used when the Gazebo GUI cannot render
on the VM's virtual GPU):

  * the lawnmower search route and its 10 waypoints (white / pale blue),
  * the home station (green cylinder at the origin),
  * the three oil patches (dark sliver/black discs) that turn *bright red*
    the moment the robot's ground-truth detector raises ``/oil/detected``
    while the robot is within the patch detection radius.

The patch state still comes from the real simulated detector
(``surface_oil_trigger`` -> ``/oil/detected`` + ``/odometry``), so the visual
flash is driven by the same mission logic that writes the CSV, not by a
separate fake trigger.
"""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from .mission import lawnmower_waypoints


class SurfaceRvizMarkers(Node):
    def __init__(self):
        super().__init__('surface_rviz_markers')

        self.declare_parameter('patch_ids', ['oil_patch_1', 'oil_patch_2', 'oil_patch_3'])
        self.declare_parameter('patch_x', [2.0, -2.0, 2.0])
        self.declare_parameter('patch_y', [0.6, 2.6, -3.4])
        self.declare_parameter('detect_radius', 1.4)
        self.declare_parameter('xmin', -4.0)
        self.declare_parameter('xmax', 4.0)
        self.declare_parameter('ymin', -4.0)
        self.declare_parameter('ymax', 4.0)
        self.declare_parameter('spacing', 2.0)
        self.declare_parameter('marker_topic', '/surface_markers')
        self.declare_parameter('frame', 'odom')
        self.declare_parameter('rate', 2.0)

        self.ids = list(self.get_parameter('patch_ids').value)
        self.xs = [float(v) for v in self.get_parameter('patch_x').value]
        self.ys = [float(v) for v in self.get_parameter('patch_y').value]
        self.detect_radius = float(self.get_parameter('detect_radius').value)
        self.frame = str(self.get_parameter('frame').value)

        self.waypoints = lawnmower_waypoints(
            float(self.get_parameter('xmin').value),
            float(self.get_parameter('xmax').value),
            float(self.get_parameter('ymin').value),
            float(self.get_parameter('ymax').value),
            float(self.get_parameter('spacing').value),
        )

        # Which patches have already flashed each run (match by robot proximity).
        self._detected = [False] * len(self.ids)
        self._last_detect_at = -1e9
        self._detected_index = -1
        self._odom = None
        self._odom_t = -1e9

        self.pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)
        self.create_subscription(
            Bool, '/oil/detected', self._on_detected, 10)
        self.create_subscription(
            Odometry, '/odometry', self._on_odom,
            10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.get_logger().info(
            'surface_rviz_markers up: %d patches, %d waypoints, frame=%s'
            % (len(self.ids), len(self.waypoints), self.frame))

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._odom_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_detected(self, msg: Bool) -> None:
        if not msg.data or self._odom is None:
            return
        x, y = self._odom
        best, best_d = -1, self.detect_radius
        for i, (px, py) in enumerate(zip(self.xs, self.ys)):
            d = math.hypot(x - px, y - py)
            if d < best_d and not self._detected[i]:
                best_d = d
                best = i
        if best >= 0:
            self._detected[best] = True
            self._detected_index = best
            self._last_detect_at = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
                'visual marker %s FLASHED at (%.2f, %.2f)'
                % (self.ids[best], x, y))

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        # Flash the most recently detected patch for ~2 s
        flash = (self._detected_index >= 0
                 and now - self._last_detect_at < 2.0)
        msgs = []

        # Search route polyline (waypoints then back home).
        pts = [(wx, wy) for wx, wy in self.waypoints] + [(0.0, 0.0)]
        line = Marker()
        line.header.frame_id = self.frame
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'route'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.03
        line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 1.0, 1.0, 0.9
        from geometry_msgs.msg import Point
        for wx, wy in pts:
            p = Point(x=float(wx), y=float(wy), z=0.0)
            line.points.append(p)
        msgs.append(line)

        # Home
        home = Marker()
        home.header.frame_id = self.frame
        home.header.stamp = self.get_clock().now().to_msg()
        home.ns = 'home'
        home.id = 0
        home.type = Marker.CYLINDER
        home.action = Marker.ADD
        home.pose.position.x, home.pose.position.y, home.pose.position.z = 0.0, 0.0, 0.05
        home.scale.x, home.scale.y, home.scale.z = 0.9, 0.9, 0.1
        home.color.r, home.color.g, home.color.b, home.color.a = 0.1, 0.9, 0.2, 0.95
        msgs.append(home)

        # Waypoint spheres
        for i, (wx, wy) in enumerate(self.waypoints):
            wp = Marker()
            wp.header.frame_id = self.frame
            wp.header.stamp = self.get_clock().now().to_msg()
            wp.ns = 'waypoints'
            wp.id = i
            wp.type = Marker.SPHERE
            wp.action = Marker.ADD
            wp.pose.position.x, wp.pose.position.y, wp.pose.position.z = float(wx), float(wy), 0.06
            wp.scale.x = wp.scale.y = wp.scale.z = 0.22
            wp.color.r, wp.color.g, wp.color.b, wp.color.a = 0.5, 0.75, 1.0, 0.85
            msgs.append(wp)

        # Oil patches: dark by default; bright red when detected (+flash)
        for i, (pid, px, py) in enumerate(zip(self.ids, self.xs, self.ys)):
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'oil'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(px), float(py), 0.03
            m.scale.x = m.scale.y = 1.6
            m.scale.z = 0.06
            if self._detected[i]:
                if flash and i == self._detected_index:
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.1, 0.95
                else:
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.15, 0.1, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.12, 0.10, 0.08, 0.9
            msgs.append(m)

        a = MarkerArray()
        a.markers = msgs
        self.pub.publish(a)


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceRvizMarkers()
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
