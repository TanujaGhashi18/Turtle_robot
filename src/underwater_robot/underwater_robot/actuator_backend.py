"""
Actuator backend: the single authority that commands the thrusters.

This node is the ONLY component in the package that publishes on the
Gazebo-bridged thruster command topics.  Everything else (mission manager,
joystick teleop, tests) talks to it indirectly through
``/actuator/thruster_cmd``.  Centralising the final output here is what
guarantees "only one backend can command the actuators".

Backends
--------
``sim`` (default)
    Publish ``std_msgs/Float64`` thrust commands on the four bridged topics.
    This is the only fully implemented path.

``hardware``
    Reserved for a future ESP32 / ESC bridge.  It is **disabled by default**
    and additionally requires ``allow_hardware:=true``.  Even then this node
    does not contain any serial/motor code: it logs an error and keeps the
    actuators at zero.  This is intentional - the simulation must never be
    able to energise real motors by accident.

Safety behaviour
----------------
* every command is clamped to +/- ``max_thrust``;
* if no fresh command arrives within ``command_timeout`` seconds the
  published thrust is zero;
* the backend starts in the zero state and returns to zero on shutdown.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, Float64MultiArray

from .allocation import THRUSTER_ORDER, zero_commands
from .safety import clamp_commands, StaleCommandGuard


class ActuatorBackend(Node):
    def __init__(self):
        super().__init__('actuator_backend')

        self.declare_parameter('backend', 'sim')
        self.declare_parameter('allow_hardware', False)
        self.declare_parameter('max_thrust', 2.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('input_topic', '/actuator/thruster_cmd')
        self.declare_parameter('topic_prefix', '/underwater_robot/thruster')

        self.backend = str(self.get_parameter('backend').value).lower()
        self.allow_hardware = bool(self.get_parameter('allow_hardware').value)
        self.max_thrust = float(self.get_parameter('max_thrust').value)
        prefix = self.get_parameter('topic_prefix').value
        input_topic = self.get_parameter('input_topic').value

        self._armed = self._resolve_backend()

        now = self._now()
        self.guard = StaleCommandGuard(
            timeout=float(self.get_parameter('command_timeout').value), now=now
        )
        self._commands = zero_commands()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Float64MultiArray, input_topic, self._on_command, qos
        )

        self.pubs = []
        for name in THRUSTER_ORDER:
            topic = '%s/%s/cmd_thrust' % (prefix, name)
            self.pubs.append(self.create_publisher(Float64, topic, qos))

        rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            'actuator_backend up: backend=%s armed=%s max_thrust=%.2f N timeout=%.2f s'
            % (self.backend, self._armed, self.max_thrust, self.guard.timeout)
        )

    # ------------------------------------------------------------------ setup
    def _resolve_backend(self) -> bool:
        if self.backend == 'sim':
            return True
        if self.backend == 'hardware':
            if not self.allow_hardware:
                self.get_logger().error(
                    "backend='hardware' requested but allow_hardware is false: "
                    'REFUSING to command actuators (staying at zero).'
                )
                return False
            self.get_logger().error(
                "backend='hardware' with allow_hardware=true was requested, but "
                'no serial/ESC transport is implemented in this build: staying at '
                'zero. See docs/hardware_validation.md.'
            )
            return False
        self.get_logger().error("unknown backend '%s': staying at zero" % self.backend)
        return False

    # ------------------------------------------------------------------ utils
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --------------------------------------------------------------- callbacks
    def _on_command(self, msg: Float64MultiArray) -> None:
        # Input is always accepted and tracked; the backend decides whether to
        # forward it based on the resolved backend/arming state.
        self._commands = clamp_commands(msg.data, self.max_thrust)
        self.guard.mark_valid(self._now())

    def _publish(self) -> None:
        if not self._armed:
            data = zero_commands()
        elif self.guard.is_stale(self._now()):
            data = zero_commands()
        else:
            data = self._commands
        for pub, value in zip(self.pubs, data):
            pub.publish(Float64(data=float(value)))

    def publish_zero(self) -> None:
        for _ in range(5):
            for pub in self.pubs:
                pub.publish(Float64(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorBackend()
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
