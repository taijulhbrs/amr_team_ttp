#!/usr/bin/env python3
"""
Potential field local planner.

Subscribes:
  /global_path (nav_msgs/Path)      - waypoints from the A* global planner
  /scan        (sensor_msgs/LaserScan) - obstacle ranges for repulsive force
  /odom        (nav_msgs/Odometry)  - current robot pose

Publishes:
  /cmd_vel (geometry_msgs/Twist)

Behaviour:
  - Tracks an index into the waypoint list. The current target is the
    waypoint at that index.
  - Attractive force pulls the robot toward the current target waypoint.
  - Repulsive force pushes the robot away from nearby laser scan points
    inside an influence radius.
  - Net force direction/magnitude is converted into linear + angular
    velocity commands.
  - When the robot gets within `waypoint_tolerance` of the current
    target, the index advances to the next waypoint. Reaching the final
    waypoint stops the robot.
"""
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf_transformations


class PotentialFieldPlanner(Node):

    def __init__(self):
        super().__init__('potential_field_planner')

        # --- tunable parameters ---
        self.declare_parameter('attractive_gain', 1.0)
        self.declare_parameter('repulsive_gain', 1.5)
        self.declare_parameter('obstacle_influence_radius', 0.8)  # meters
        self.declare_parameter('waypoint_tolerance', 0.25)        # meters
        self.declare_parameter('max_linear_speed', 0.3)           # m/s
        self.declare_parameter('max_angular_speed', 1.0)          # rad/s
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_frequency', 10.0)         # Hz

        self.k_att = self.get_parameter('attractive_gain').value
        self.k_rep = self.get_parameter('repulsive_gain').value
        self.rho_0 = self.get_parameter('obstacle_influence_radius').value
        self.wp_tolerance = self.get_parameter('waypoint_tolerance').value
        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value

        self.waypoints = []       # list of (x, y)
        self.current_wp_idx = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.latest_scan = None
        self.have_pose = False

        self.create_subscription(Path, self.get_parameter('path_topic').value, self.path_cb, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.scan_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)

        period = 1.0 / self.get_parameter('control_frequency').value
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info('Potential field planner ready. Waiting for path and odometry...')

    def path_cb(self, msg: Path):
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_wp_idx = 0
        self.get_logger().info(f'Received new path with {len(self.waypoints)} waypoints.')

    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.have_pose = True

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def control_loop(self):
        if not self.have_pose or not self.waypoints:
            return

        if self.current_wp_idx >= len(self.waypoints):
            self._publish_cmd(0.0, 0.0)
            return

        target = self.waypoints[self.current_wp_idx]
        dist_to_target = math.hypot(target[0] - self.robot_x, target[1] - self.robot_y)

        if dist_to_target < self.wp_tolerance:
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info('Final waypoint reached. Stopping.')
                self._publish_cmd(0.0, 0.0)
                return
            target = self.waypoints[self.current_wp_idx]

        fx_att, fy_att = self._attractive_force(target)
        fx_rep, fy_rep = self._repulsive_force()

        fx = fx_att + fx_rep
        fy = fy_att + fy_rep

        self._apply_force_as_velocity(fx, fy)

    def _attractive_force(self, target):
        dx = target[0] - self.robot_x
        dy = target[1] - self.robot_y
        return self.k_att * dx, self.k_att * dy

    def _repulsive_force(self):
        """Compute repulsive force in the robot's world frame from laser scan points."""
        if self.latest_scan is None:
            return 0.0, 0.0

        fx_total, fy_total = 0.0, 0.0
        scan = self.latest_scan
        angle = scan.angle_min

        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min < r < self.rho_0:
                # obstacle position in world frame
                obs_angle_world = self.robot_yaw + angle
                ox = self.robot_x + r * math.cos(obs_angle_world)
                oy = self.robot_y + r * math.sin(obs_angle_world)

                dx = self.robot_x - ox
                dy = self.robot_y - oy
                dist = max(r, 0.05)  # avoid division blow-up at zero range

                magnitude = self.k_rep * (1.0 / dist - 1.0 / self.rho_0) * (1.0 / (dist ** 2))
                fx_total += magnitude * (dx / dist)
                fy_total += magnitude * (dy / dist)

            angle += scan.angle_increment

        return fx_total, fy_total

    def _apply_force_as_velocity(self, fx, fy):
        """Convert a world-frame force vector into linear/angular velocity commands."""
        force_angle = math.atan2(fy, fx)
        force_mag = math.hypot(fx, fy)

        angle_error = self._normalize_angle(force_angle - self.robot_yaw)

        # Slow down / turn in place when badly misaligned with desired direction
        angular_z = max(-self.max_w, min(self.max_w, 2.0 * angle_error))

        if abs(angle_error) > 1.2:  # > ~70 deg misaligned: rotate in place first
            linear_x = 0.0
        else:
            linear_x = max(0.0, min(self.max_v, 0.5 * force_mag))

        self._publish_cmd(linear_x, angular_z)

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _publish_cmd(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
