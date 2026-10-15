"""
Bounded, scripted command test for the control chain.

Publishes a short, hard-coded sequence of *normalised* body commands on
``/cmd_vel`` (each component limited to +/- ``amplitude``) and finishes with
an explicit zero (safe stop).  Useful for verifying the
``/cmd_vel -> allocator -> backend -> Gazebo`` chain and the timeout
behaviour without needing a joystick.

Run, then watch e.g.:
    ros2 topic echo /actuator/thruster_cmd
    ros2 topic echo /depth
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


def _twist(surge=0.0, heave=0.0, pitch=0.0, yaw=0.0) -> Twist:
    t = Twist()
    t.linear.x = surge
    t.linear.z = heave
    t.angular.y = pitch
    t.angular.z = yaw
    return t


# (duration_seconds, twist)
DEFAULT_SEQUENCE = [
    (1.0, _twist()),                      # settle
    (2.0, _twist(heave=0.5)),             # ascend
    (1.0, _twist()),                      # settle
    (3.0, _twist(surge=0.4)),             # surge forward
    (1.0, _twist()),                      # settle
    (2.0, _twist(yaw=0.4)),               # yaw left
    (1.0, _twist()),                      # settle
    (2.0, _twist(heave=-0.4)),            # descend
    (2.0, _twist()),                      # safe stop
]


class ScriptedTest(Node):
    def __init__(self):
        super().__init__('scripted_test')
        self.declare_parameter('amplitude', 0.5)
        self.amplitude = float(self.get_parameter('amplitude').value)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._seq = [(d, t) for d, t in DEFAULT_SEQUENCE]
        self._idx = 0
        self._deadline = self._now() + self._seq[0][0]
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'scripted_test started: %d phases, amplitude=%.2f' % (len(self._seq), self.amplitude)
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _clamp(self, t: Twist) -> Twist:
        a = self.amplitude
        t.linear.x = max(-a, min(a, t.linear.x))
        t.linear.z = max(-a, min(a, t.linear.z))
        t.angular.y = max(-a, min(a, t.angular.y))
        t.angular.z = max(-a, min(a, t.angular.z))
        return t

    def _tick(self) -> None:
        now = self._now()
        if self._idx >= len(self._seq):
            self.pub.publish(Twist())
            return
        if now >= self._deadline:
            self._idx += 1
            if self._idx >= len(self._seq):
                self.get_logger().info('scripted_test complete - zeroing')
                self.pub.publish(Twist())
                return
            self._deadline = now + self._seq[self._idx][0]
            self.get_logger().info('phase %d/%d' % (self._idx + 1, len(self._seq)))
        self.pub.publish(self._clamp(self._seq[self._idx][1]))


def main(args=None):
    rclpy.init(args=args)
    node = ScriptedTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
