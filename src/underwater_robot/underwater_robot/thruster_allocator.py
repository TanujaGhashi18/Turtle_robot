"""
Thruster allocator node.

Subscribes to a body-velocity command (``geometry_msgs/Twist``) and converts
it to four per-thruster thrust commands (Newtons), which it publishes as a
``std_msgs/Float64MultiArray`` on ``/actuator/thruster_cmd``.

This node performs *no* actuator I/O.  It is deliberately separated from
:mod:`actuator_backend`, which is the sole component allowed to talk to the
Gazebo thrusters (or, in a future hardware build, to the ESP32 serial link).

Safety behaviour:
    * commands are clamped to +/- ``max_thrust``;
    * if no ``/cmd_vel`` message arrives within ``cmd_timeout`` seconds the
      published command is zero (safe stop on lost command);
    * the command is zeroed on shutdown.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from .allocation import allocate, zero_commands
from .safety import StaleCommandGuard


class ThrusterAllocator(Node):
    def __init__(self):
        super().__init__('thruster_allocator')

        self.declare_parameter('max_thrust', 2.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/actuator/thruster_cmd')
        self.declare_parameter('max_step', 0.12)

        self.max_thrust = float(self.get_parameter('max_thrust').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.max_step = float(self.get_parameter('max_step').value)
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        output_topic = self.get_parameter('output_topic').value

        now = self._now()
        self.guard = StaleCommandGuard(
            timeout=float(self.get_parameter('cmd_timeout').value), now=now
        )
        self._commands = zero_commands()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, qos)
        self.pub = self.create_publisher(Float64MultiArray, output_topic, qos)

        period = 1.0 / max(self.publish_rate, 1.0)
        self.create_timer(period, self._publish)

        self.get_logger().info(
            'thruster_allocator up: %s -> %s (max_thrust=%.2f N, timeout=%.2f s, '
            'max_step=%.2f N/cycle)'
            % (
                cmd_vel_topic, output_topic, self.max_thrust, self.guard.timeout,
                self.max_step,
            )
        )

    # ------------------------------------------------------------------ utils
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --------------------------------------------------------------- callbacks
    def _on_cmd_vel(self, msg: Twist) -> None:
        target = allocate(
            surge=msg.linear.x,
            heave=msg.linear.z,
            pitch=msg.angular.y,
            yaw=msg.angular.z,
            max_thrust=self.max_thrust,
        )
        # Command slew-rate limiter (thrust rate limit): each per-thruster
        # command may change by at most max_step Newtons per allocator cycle.
        # This keeps step inputs and controller changes smooth, reducing
        # thruster/propeller transients both in simulation and on the future
        # hardware.  Safe stop (timeout/shutdown) intentionally bypasses the
        # limiter and zeroes immediately.
        step = max(float(self.max_step), 0.0)
        for i in range(len(target)):
            delta = target[i] - self._commands[i]
            if delta > step:
                delta = step
            elif delta < -step:
                delta = -step
            self._commands[i] = self._commands[i] + delta
        # A command with sway/roll content is accepted but those DOFs cannot
        # be produced; note it once so the operator is not misled.
        if abs(msg.linear.y) > 1e-6 or abs(msg.angular.x) > 1e-6:
            self.get_logger().warn(
                'sway/roll are not actuated; ignoring linear.y=%.3f angular.x=%.3f'
                % (msg.linear.y, msg.angular.x),
                throttle_duration_sec=5.0,
            )
        self.guard.mark_valid(self._now())

    def _publish(self) -> None:
        data = zero_commands() if self.guard.is_stale(self._now()) else self._commands
        msg = Float64MultiArray()
        msg.layout.dim = []
        msg.data = [float(v) for v in data]
        self.pub.publish(msg)

    # ---------------------------------------------------------------- shutdown
    def publish_zero(self) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in zero_commands()]
        for _ in range(3):
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_zero()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
