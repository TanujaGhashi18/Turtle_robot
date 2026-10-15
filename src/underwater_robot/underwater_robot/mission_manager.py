"""
Autonomous search-mission manager.

Drives the vehicle through a lawnmower survey of the pool while holding a
target depth and a level attitude, pausing when an oil detection arrives, and
finally returning home.  All output goes to ``/cmd_vel`` (normalised body
command, see allocation.py); the mission never touches the thrusters directly.

Safety
------
* ``autostart`` defaults to **false**; the vehicle only moves when the mission
  is explicitly started (``autostart:=true`` or a ``True`` on ``/mission/start``).
* If odometry stops arriving the published command is zero.
* A hard ``mission_timeout`` ends the run and zeroes the command.
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64

from .mission import (
    clamp_norm,
    depth_hold,
    heading_to,
    lawnmower_waypoints,
    pitch_from_quaternion,
    wrap_angle,
    yaw_from_quaternion,
)

# states
IDLE = 'IDLE'
SEARCH = 'SEARCH'
HOLD = 'HOLD'
RETURN = 'RETURN'
DONE = 'DONE'


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('autostart', False)
        self.declare_parameter('target_depth', 4.0)
        # Cruise command (normalised surge 0..1). Calibrated against the
        # Hydrodynamics drag in urdf/final_robot.urdf (linear 2.0 +
        # quadratic 0.8 N): 0.20 normalised == ~0.33 m/s cruise, kept well
        # under the ~1 m/s regime where the contact solver went unstable.
        # This is a normalised thrust command, NOT a velocity in m/s.
        self.declare_parameter('forward_speed', 0.20)
        self.declare_parameter('yaw_gain', 0.9)
        self.declare_parameter('depth_gain', 0.8)
        # Vertical authority cap (normalised heave).  Uncapped, the full 2 N
        # per vertical thruster rockets the robot up/down at ~2 m/s and the
        # depth hold overshoots from seabed to surface.  Capping the command
        # keeps vertical transients gentle; the cap is conservative on purpose.
        self.declare_parameter('heave_limit', 0.35)
        # Vertical-velocity damping on the depth hold (N per m/s of body z
        # velocity).  Counteracts depth overshoot caused by the buoyancy and
        # propeller response lag.
        self.declare_parameter('depth_damp', 2.0)
        self.declare_parameter('pitch_gain', 0.5)
        # Arrival radius used for two things: declaring a waypoint reached and
        # switching to the next leg.  Large enough that the vehicle starts its
        # turn while still a comfortable distance from the pool walls.
        self.declare_parameter('arrival_radius', 1.2)
        self.declare_parameter('xmin', -6.0)
        self.declare_parameter('xmax', 6.0)
        self.declare_parameter('ymin', -6.0)
        self.declare_parameter('ymax', 6.0)
        self.declare_parameter('spacing', 3.0)
        self.declare_parameter('hold_duration', 5.0)
        self.declare_parameter('odom_timeout', 1.0)
        self.declare_parameter('mission_timeout', 600.0)
        self.declare_parameter('rate', 10.0)

        g = self.get_parameter
        self.target_depth = float(g('target_depth').value)
        self.forward_speed = float(g('forward_speed').value)
        self.yaw_gain = float(g('yaw_gain').value)
        self.depth_gain = float(g('depth_gain').value)
        self.heave_limit = float(g('heave_limit').value)
        self.depth_damp = float(g('depth_damp').value)
        self.pitch_gain = float(g('pitch_gain').value)
        self.arrival_radius = float(g('arrival_radius').value)
        self.hold_duration = float(g('hold_duration').value)
        self.odom_timeout = float(g('odom_timeout').value)
        self.mission_timeout = float(g('mission_timeout').value)

        self.waypoints = lawnmower_waypoints(
            float(g('xmin').value), float(g('xmax').value),
            float(g('ymin').value), float(g('ymax').value),
            float(g('spacing').value),
        )
        self._wp_index = 0
        # Partial turning within the arrival envelope: the vehicle pauses and
        # rotates in place onto the next leg before advancing, so it never
        # skids through a waypoint into the pool walls at speed.
        self._aligning = False

        self.state = IDLE
        self._odom = None
        # Depth is filled by /depth once it arrives; until then assume we are
        # at the target so the hold controller does not need a message yet.
        self._depth = self.target_depth
        self._last_odom_time = 0.0
        self._detected = False
        self._state_since = self._now()
        self._mission_start = None
        self._detections = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, '/odometry', self._on_odom, qos)
        self.create_subscription(Float64, '/depth', self._on_depth, 10)
        self.create_subscription(Bool, '/oil/detected', self._on_detected, 10)
        self.create_subscription(Bool, '/mission/start', self._on_start, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        rate = max(float(g('rate').value), 1.0)
        self.create_timer(1.0 / rate, self._tick)

        if bool(g('autostart').value):
            self._start()

        self.get_logger().info(
            'mission_manager up: state=%s waypoints=%d target_depth=%.1f m '
            '(depth_gain=%.2f depth_damp=%.2f heave_limit=%.2f)'
            % (
                self.state, len(self.waypoints), self.target_depth,
                self.depth_gain, self.depth_damp, self.heave_limit,
            )
        )

    # ------------------------------------------------------------------ utils
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_state(self, new_state: str) -> None:
        if new_state != self.state:
            self.get_logger().info('mission state: %s -> %s' % (self.state, new_state))
            self.state = new_state
            self._state_since = self._now()

    def _start(self) -> None:
        if self.state in (IDLE, DONE):
            self._wp_index = 0
            self._aligning = False
            self._detections = 0
            self._mission_start = self._now()
            self._set_state(SEARCH)

    # --------------------------------------------------------------- callbacks
    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._last_odom_time = self._now()

    def _on_depth(self, msg: Float64) -> None:
        value = float(msg.data)
        # A non-finite reading (e.g. sensor diverging after an out-of-water
        # escape) must not poison the depth hold; treat it as "unknown" and
        # hold the last sane value.
        if math.isfinite(value):
            self._depth = value

    def _on_detected(self, msg: Bool) -> None:
        self._detected = bool(msg.data)
        if self._detected and self.state == SEARCH:
            self._detections += 1
            self.get_logger().warn('detection #%d - holding position' % self._detections)
            self._set_state(HOLD)

    def _on_start(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info('mission start requested')
            self._start()

    # ------------------------------------------------------------- guidance
    def _zero(self) -> Twist:
        return Twist()

    def _path_error(self, tx: float, ty: float):
        """Distance to (tx, ty) and wrapped yaw error onto it (dist, err)."""
        p = self._odom.pose.pose
        dx, dy = tx - p.position.x, ty - p.position.y
        dist = (dx * dx + dy * dy) ** 0.5
        q = p.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        err = wrap_angle(heading_to(dx, dy) - yaw)
        return dist, err

    def _navigate(self, tx: float, ty: float):
        """Return (surge, yaw) to steer towards (tx, ty)."""
        dist, err = self._path_error(tx, ty)
        yaw_cmd = clamp_norm(self.yaw_gain * err)
        # Ease off forward speed while badly mis-aligned.
        surge = clamp_norm(self.forward_speed * max(0.0, math.cos(err)))
        # Bleed speed well before the waypoint so the vehicle arrives slow
        # instead of plowing past it into the pool walls/corners (fast
        # wall contact with the spinning propellers destabilises the solver).
        approach_span = 3.0 * self.arrival_radius
        approach = min(1.0, dist / approach_span) if approach_span > 0 else 1.0
        return surge * approach, yaw_cmd, dist

    def _body_vz(self) -> float:
        """Body-frame vertical velocity (m/s, +z up); 0 if unavailable."""
        if self._odom is None:
            return 0.0
        vz = self._odom.twist.twist.linear.z
        return 0.0 if not math.isfinite(vz) else float(vz)

    def _hold_outputs(self) -> Twist:
        out = Twist()
        if self._odom is not None:
            q = self._odom.pose.pose.orientation
            pitch = pitch_from_quaternion(q.x, q.y, q.z, q.w)
            out.angular.y = clamp_norm(-self.pitch_gain * pitch)
        # Depth hold: proportional on depth error plus damping proportional on
        # body vertical velocity, capped by heave_limit so the vehicle cannot
        # rocket vertically (see the parameter docstring).
        out.linear.z = max(-self.heave_limit,
                           min(self.heave_limit, depth_hold(
                               self._depth - self.target_depth,
                               self._body_vz(),
                               self.depth_gain,
                               self.depth_damp,
                           )))
        return out

    # ------------------------------------------------------------- main loop
    def _tick(self) -> None:
        now = self._now()

        # While IDLE / DONE the manager publishes NOTHING.  The downstream
        # allocator and actuator_backend both independently time out to zero,
        # and staying silent leaves /cmd_vel free for teleop or tests.
        if self.state in (IDLE, DONE):
            return

        if now - self._last_odom_time > self.odom_timeout:
            self.get_logger().warn('odometry stale - safe stop', throttle_duration_sec=5.0)
            self.pub.publish(self._zero())
            return

        if self._mission_start is not None and now - self._mission_start > self.mission_timeout:
            self.get_logger().warn('mission timeout reached - stopping')
            self._set_state(DONE)
            self.pub.publish(self._zero())
            return

        out = self._hold_outputs()

        if self.state == HOLD:
            out.linear.x = 0.0
            out.angular.z = 0.0
            self.get_logger().info(
                'hold: depth=%.2f m (target %.2f) vz=%.2f m/s'
                % (self._depth, self.target_depth, self._body_vz()),
                throttle_duration_sec=2.0,
            )
            if now - self._state_since > self.hold_duration:
                self._set_state(SEARCH)
            self.pub.publish(out)
            return

        if self.state == SEARCH:
            if self._wp_index >= len(self.waypoints):
                self._set_state(RETURN)
            else:
                tx, ty = self.waypoints[self._wp_index]
                surge, yaw, dist = self._navigate(tx, ty)
                out.angular.z = yaw
                _, err_now = self._path_error(tx, ty)
                if self._aligning:
                    # Waypoint just claimed: rotate in place onto this (next)
                    # leg's heading before advancing.  Zero surge keeps the
                    # turn from skidding the vehicle into a pool wall.
                    if abs(err_now) < 0.15:
                        self._aligning = False
                        self.get_logger().info('heading aligned - advancing')
                    else:
                        out.linear.x = 0.0
                        self.pub.publish(out)
                        return
                out.linear.x = surge
                if dist < self.arrival_radius:
                    if abs(err_now) < 0.35:
                        self.get_logger().info(
                            'waypoint %d/%d reached'
                            % (self._wp_index + 1, len(self.waypoints))
                        )
                        self._wp_index += 1
                        if self._wp_index >= len(self.waypoints):
                            self._set_state(RETURN)
                        else:
                            self._aligning = True
                    else:
                        # Close but misaligned: rotate onto the waypoint first.
                        out.linear.x = 0.0
                self.pub.publish(out)
                return

        if self.state == RETURN:
            surge, yaw, dist = self._navigate(0.0, 0.0)
            out.linear.x = surge
            out.angular.z = yaw
            if dist < self.arrival_radius:
                self.get_logger().info(
                    'home reached; mission complete (%d detections)' % self._detections
                )
                self.pub.publish(self._zero())
                self._set_state(DONE)
                return
            self.pub.publish(out)
            return

        self.pub.publish(self._zero())


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
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
