"""
vm_demo.py — Fake robot + lidar data for UI development on a VM.

Publishes /fused_pose and /lidar_world_points so the World Canvas,
GPS card, and ROS Nodes card all show live data without real hardware.

Run inside the Docker container after sourcing ROS2:

    source /ros2_ws/install/setup.bash
    python3 /ros2_ws/vm_demo.py

The robot drives a slow circle. Fake lidar points form a rough square
"room" around the robot so the point cloud looks realistic.
Press Ctrl-C to stop.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from bridge_interfaces.msg import FusedPose, LidarWorldPoints

# ── Tune these ────────────────────────────────────────────────────────────────
CIRCLE_RADIUS_MM   = 600.0   # robot orbit radius
CIRCLE_PERIOD_S    = 20.0    # seconds per full lap
ROOM_HALF_SIZE_MM  = 1500.0  # half-width of the fake square room
LIDAR_RAYS         = 60      # number of fake lidar rays
PUBLISH_HZ         = 10      # update rate
# ─────────────────────────────────────────────────────────────────────────────


class VMDemo(Node):
    def __init__(self):
        super().__init__('vm_demo')
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pose_pub  = self.create_publisher(FusedPose, '/fused_pose', best_effort)
        self._lidar_pub = self.create_publisher(LidarWorldPoints, '/lidar_world_points', best_effort)

        self._t = 0.0
        self._dt = 1.0 / PUBLISH_HZ
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'vm_demo started — robot circles r={CIRCLE_RADIUS_MM:.0f} mm, '
            f'period={CIRCLE_PERIOD_S:.0f} s'
        )

    def _tick(self):
        self._t += self._dt
        phase = (self._t / CIRCLE_PERIOD_S) * 2.0 * math.pi

        # Robot pose — circle in world frame
        rx = CIRCLE_RADIUS_MM * math.cos(phase)
        ry = CIRCLE_RADIUS_MM * math.sin(phase)
        rtheta = phase + math.pi / 2.0   # tangent direction

        # Publish fused pose (GPS not active in VM)
        fp = FusedPose()
        fp.header.stamp = self.get_clock().now().to_msg()
        fp.x          = float(rx)
        fp.y          = float(ry)
        fp.theta      = float(rtheta)
        fp.gps_active = False
        self._pose_pub.publish(fp)

        # Fake lidar — rays cast from robot toward a square room boundary.
        # Each ray hits the nearest wall; add slight jitter so it looks scanned.
        import random
        xs, ys = [], []
        for i in range(LIDAR_RAYS):
            angle = rtheta + (i / LIDAR_RAYS) * 2.0 * math.pi
            dx = math.cos(angle)
            dy = math.sin(angle)

            # Ray-box intersection: find t for each wall, take the nearest.
            R = ROOM_HALF_SIZE_MM
            ts = []
            if abs(dx) > 1e-6:
                ts += [(R - rx) / dx, (-R - rx) / dx]
            if abs(dy) > 1e-6:
                ts += [(R - ry) / dy, (-R - ry) / dy]
            t_hit = min(t for t in ts if t > 10.0)   # nearest wall in front

            jitter = random.uniform(-20.0, 20.0)
            xs.append(rx + dx * (t_hit + jitter))
            ys.append(ry + dy * (t_hit + jitter))

        lw = LidarWorldPoints()
        lw.header.stamp = fp.header.stamp
        lw.xs          = xs
        lw.ys          = ys
        lw.robot_x     = float(rx)
        lw.robot_y     = float(ry)
        lw.robot_theta = float(rtheta)
        self._lidar_pub.publish(lw)


def main():
    rclpy.init()
    node = VMDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
