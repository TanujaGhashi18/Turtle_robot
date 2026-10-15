"""
Simulated (ground-truth) oil-spill detection trigger for the SURFACE demo.

WHY THIS EXISTS
---------------
The robot's on-board camera in ``urdf/final_robot.urdf`` is pitched ~34 deg
DOWN to look at the sea floor, which is the correct configuration for the
underwater acceptance mission.  In the surface demonstration the oil slicks sit
ON (or just above) the waterline, i.e. in a plane that the downward camera
grazes and cannot image reliably.  Rather than re-pointing the authoritative
camera or faking a computer-vision result, this node provides a clearly labelled
*simulated ground-truth* trigger: when the vehicle's own odometry enters the
known detection radius of a known patch, it raises ``/oil/detected``.

The existing camera + OpenCV pipeline (``oil_detector``) is deliberately NOT run
in the surface launch, so ``/oil/detected`` has exactly one publisher here.

HONESTY / LABELLING
-------------------
This is NOT image-based detection.  Every detection is tagged
``method=SIMULATED_GROUND_TRUTH`` in the log line and in the CSV column
``method``.  It is a mission-logic trigger keyed to the published patch
coordinates, and must be described as such in the report/viva.

Behaviour
---------
* Subscribes to ``/odometry`` (nav_msgs/Odometry, BEST_EFFORT).
* Publishes ``/oil/detected`` (std_msgs/Bool) as a short pulse per patch and
  ``/oil/confidence`` (std_msgs/Float64) = 1.0 during the pulse, else 0.0.
* Each patch fires AT MOST ONCE per run (cooldown), so the mission does not get
  stuck in HOLD.
* Every event is logged to a CSV with: wall time, patch id, method, and the
  robot position at detection.
"""

import csv
import os
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64


class SurfaceOilTrigger(Node):
    def __init__(self):
        super().__init__('surface_oil_trigger')

        # Patch ground-truth table.  Defaults match worlds/surface_final_world.sdf.
        self.declare_parameter(
            'patch_ids', ['oil_patch_1', 'oil_patch_2', 'oil_patch_3'])
        self.declare_parameter('patch_x', [2.0, -2.0, 2.0])
        self.declare_parameter('patch_y', [0.6, 2.6, -3.4])
        # Radius (m) within which passing the patch counts as a detection.
        self.declare_parameter('detect_radius', 1.4)
        # Length (s) of the True pulse published on /oil/detected.
        self.declare_parameter('pulse_duration', 0.6)
        self.declare_parameter('odom_topic', '/odometry')
        self.declare_parameter('detected_topic', '/oil/detected')
        self.declare_parameter('confidence_topic', '/oil/confidence')
        self.declare_parameter(
            'log_file', os.path.expanduser('~/ros2_ws/surface_oil_detections.csv'))
        self.declare_parameter('rate', 10.0)

        self.ids = list(self.get_parameter('patch_ids').value)
        self.xs = [float(v) for v in self.get_parameter('patch_x').value]
        self.ys = [float(v) for v in self.get_parameter('patch_y').value]
        if not (len(self.ids) == len(self.xs) == len(self.ys)):
            raise ValueError('patch_ids/patch_x/patch_y must have equal length')
        self.radius = float(self.get_parameter('detect_radius').value)
        self.pulse_duration = float(self.get_parameter('pulse_duration').value)
        self.log_file = os.path.expanduser(str(self.get_parameter('log_file').value))

        self._odom = None
        self._fired = [False] * len(self.ids)
        self._pulse_until = 0.0
        self._log_ready = self._open_log()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, qos)
        self.pub_detected = self.create_publisher(
            Bool, self.get_parameter('detected_topic').value, 10)
        self.pub_conf = self.create_publisher(
            Float64, self.get_parameter('confidence_topic').value, 10)

        rate = max(float(self.get_parameter('rate').value), 1.0)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            'surface_oil_trigger up (SIMULATED GROUND-TRUTH, not camera): %d '
            'patches, radius=%.2f m, log=%s'
            % (len(self.ids), self.radius, self.log_file))

    # ------------------------------------------------------------------ logging
    def _open_log(self) -> bool:
        try:
            os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
            new = not os.path.exists(self.log_file)
            if new:
                with open(self.log_file, 'w', newline='') as fh:
                    csv.writer(fh).writerow(
                        ['wall_time_iso', 'wall_time_unix', 'patch_id', 'method',
                         'pos_x', 'pos_y', 'pos_z'])
            return True
        except Exception as exc:
            self.get_logger().error("cannot open log '%s': %s" % (self.log_file, exc))
            return False

    def _write_row(self, patch_id: str, x: float, y: float, z: float) -> None:
        if not self._log_ready:
            return
        try:
            with open(self.log_file, 'a', newline='') as fh:
                csv.writer(fh).writerow([
                    time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime()),
                    '%.3f' % time.time(),
                    patch_id,
                    'SIMULATED_GROUND_TRUTH',
                    '%.3f' % x, '%.3f' % y, '%.3f' % z,
                ])
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn('failed writing detection log: %s' % exc)

    # --------------------------------------------------------------- callbacks
    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        # Always keep /oil/detected and /oil/confidence in a defined state.
        detected = self._now() < self._pulse_until

        if self._odom is not None:
            p = self._odom.pose.pose.position
            # Rising-edge check: fire the first patch (if any) that is in range
            # and has not fired yet this run.
            for i, (pid, px, py) in enumerate(zip(self.ids, self.xs, self.ys)):
                if self._fired[i]:
                    continue
                d = ((p.x - px) ** 2 + (p.y - py) ** 2) ** 0.5
                if d <= self.radius:
                    self._fired[i] = True
                    self._pulse_until = self._now() + self.pulse_duration
                    detected = True
                    self._write_row(pid, p.x, p.y, p.z)
                    self.get_logger().warn(
                        'OIL DETECTED patch=%s method=SIMULATED_GROUND_TRUTH '
                        'pos=(%.2f, %.2f, %.2f) range=%.2f m'
                        % (pid, p.x, p.y, p.z, d))
                    break

        b = Bool()
        b.data = bool(detected)
        self.pub_detected.publish(b)
        c = Float64()
        c.data = 1.0 if detected else 0.0
        self.pub_conf.publish(c)


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceOilTrigger()
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
