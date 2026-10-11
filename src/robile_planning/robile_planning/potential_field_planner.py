#!/usr/bin/env python3
"""
Potential field local planner.

- Pose (x, y, yaw) all come from the SAME frame: TF map -> base_link
  (falls back to /odom only if TF is unavailable).
- Repulsive force is capped so it cannot overwhelm the attractive force.
- Near the final goal ("docking phase") repulsion and the safety stop are
  disabled so goals close to walls can still be reached.
- Stuck / oscillation detection with an escape manoeuvre.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf_transformations
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class PotentialFieldPlanner(Node):

    def __init__(self):
        super().__init__('potential_field_planner')

        self.declare_parameter('attractive_gain', 1.0)
        self.declare_parameter('repulsive_gain', 0.8)
        self.declare_parameter('obstacle_influence_radius', 0.35)
        self.declare_parameter('max_repulsive_magnitude', 2.0)
        self.declare_parameter('waypoint_tolerance', 0.25)
        self.declare_parameter('docking_radius', 0.5)
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_tf_pose', True)
        self.declare_parameter('safety_distance', 0.3)

        self.declare_parameter('stuck_time_window', 3.0)
        self.declare_parameter('stuck_distance_threshold', 0.15)
        self.declare_parameter('escape_duration', 2.5)
        self.declare_parameter('escape_angular_speed', 0.8)
        self.declare_parameter('escape_linear_speed', 0.1)

        self.k_att = self.get_parameter('attractive_gain').value
        self.k_rep = self.get_parameter('repulsive_gain').value
        self.rho_0 = self.get_parameter('obstacle_influence_radius').value
        self.max_rep = self.get_parameter('max_repulsive_magnitude').value
        self.wp_tolerance = self.get_parameter('waypoint_tolerance').value
        self.docking_radius = self.get_parameter('docking_radius').value
        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value

        self.stuck_time_window = self.get_parameter('stuck_time_window').value
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').value
        self.escape_duration = self.get_parameter('escape_duration').value
        self.escape_angular_speed = self.get_parameter('escape_angular_speed').value
        self.escape_linear_speed = self.get_parameter('escape_linear_speed').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.use_tf_pose = self.get_parameter('use_tf_pose').value

        self.waypoints = []
        self.current_wp_idx = 0

        # Active pose (map frame when TF works)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        # Odom pose (fallback only)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.have_odom = False
        self.tf_ok = False
        self._tf_warned = False

        self.latest_scan = None
        self.goal_reached_announced = False

        self.pose_history = deque()   # (Time, x, y)
        self.escape_until = None
        self.escape_direction = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, self.get_parameter('path_topic').value, self.path_cb, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)

        period = 1.0 / self.get_parameter('control_frequency').value
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info('Potential field planner ready (pose from TF map->base_link).')

    # ---------------- callbacks ----------------

    def path_cb(self, msg: Path):
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_wp_idx = 0
        self.goal_reached_announced = False
        self.pose_history.clear()
        self.escape_until = None
        self.get_logger().info(f'Received new path with {len(self.waypoints)} waypoints.')

    def odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.odom_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.have_odom = True

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # ---------------- pose ----------------

    def _update_pose(self):
        """x, y AND yaw all from the same frame. Returns True if a pose exists."""
        if self.use_tf_pose:
            try:
                t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
                self.robot_x = t.transform.translation.x
                self.robot_y = t.transform.translation.y
                q = t.transform.rotation
                _, _, self.robot_yaw = tf_transformations.euler_from_quaternion(
                    [q.x, q.y, q.z, q.w])
                self.tf_ok = True
                return True
            except (LookupException, ConnectivityException, ExtrapolationException):
                self.tf_ok = False
                if not self._tf_warned:
                    self.get_logger().warn(
                        f'TF {self.map_frame}->{self.base_frame} not available, '
                        f'falling back to odom (goal frame may be wrong!).')
                    self._tf_warned = True

        if self.have_odom:
            self.robot_x, self.robot_y, self.robot_yaw = self.odom_x, self.odom_y, self.odom_yaw
            return True
        return False

    # ---------------- safety / stuck ----------------

    def _obstacle_too_close(self):
        if self.latest_scan is None:
            return False
        rng = [r for r in self.latest_scan.ranges
               if math.isfinite(r) and r > self.latest_scan.range_min]
        if not rng:
            return False
        return min(rng) < self.safety_distance

    def _update_stuck_detection_and_check(self, now):
        self.pose_history.append((now, self.robot_x, self.robot_y))
        max_age = Duration(seconds=self.stuck_time_window * 2)
        while self.pose_history and (now - self.pose_history[0][0]) > max_age:
            self.pose_history.popleft()

        window = Duration(seconds=self.stuck_time_window)
        reference = None
        for t, x, y in self.pose_history:
            if (now - t) >= window:
                reference = (x, y)
            else:
                break
        if reference is None:
            return False
        return math.hypot(self.robot_x - reference[0],
                          self.robot_y - reference[1]) < self.stuck_distance_threshold

    def _choose_escape_direction(self):
        if self.latest_scan is None:
            return self.escape_angular_speed
        scan = self.latest_scan
        angle = scan.angle_min
        ls, ln, rs, rn = 0.0, 0, 0.0, 0
        for r in scan.ranges:
            if math.isfinite(r) and r > scan.range_min:
                if angle >= 0:
                    ls += r
                    ln += 1
                else:
                    rs += r
                    rn += 1
            angle += scan.angle_increment
        left_avg = ls / ln if ln else 0.0
        right_avg = rs / rn if rn else 0.0
        return self.escape_angular_speed if left_avg >= right_avg else -self.escape_angular_speed

    # ---------------- main loop ----------------

    def control_loop(self):
        if not self.waypoints:
            return
        if not self._update_pose():
            return

        now = self.get_clock().now()

        # escape manoeuvre in progress
        if self.escape_until is not None:
            if now < self.escape_until:
                self._publish_cmd(self.escape_linear_speed, self.escape_direction)
                return
            self.escape_until = None
            self.pose_history.clear()
            self.get_logger().info('Escape finished, resuming navigation.')

        # already finished
        if self.current_wp_idx >= len(self.waypoints):
            if not self.goal_reached_announced:
                self.get_logger().info('Reached goal!')
                self.goal_reached_announced = True
            self._publish_cmd(0.0, 0.0)
            return

        # arrival check FIRST (so goals near walls still count)
        target = self.waypoints[self.current_wp_idx]
        dist = math.hypot(target[0] - self.robot_x, target[1] - self.robot_y)
        if dist < self.wp_tolerance:
            self.current_wp_idx += 1
            self.pose_history.clear()
            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info('Reached goal!')
                self.goal_reached_announced = True
                self._publish_cmd(0.0, 0.0)
                return
            target = self.waypoints[self.current_wp_idx]
            dist = math.hypot(target[0] - self.robot_x, target[1] - self.robot_y)

        is_last = self.current_wp_idx == len(self.waypoints) - 1
        in_docking_phase = is_last and dist < self.docking_radius

        if not in_docking_phase:
            if self._obstacle_too_close():
                self.get_logger().warn('Obstacle too close! Turning in place.',
                                       throttle_duration_sec=1.0)
                self._publish_cmd(0.0, 0.3)
                self.pose_history.clear()
                return

            if self._update_stuck_detection_and_check(now):
                self.escape_direction = self._choose_escape_direction()
                self.escape_until = now + Duration(seconds=self.escape_duration)
                self.get_logger().warn(
                    f'Local minimum detected - escaping '
                    f'({"left" if self.escape_direction > 0 else "right"}).')
                self._publish_cmd(self.escape_linear_speed, self.escape_direction)
                return
        else:
            self.pose_history.clear()

        fx_att, fy_att = self._attractive_force(target)
        if in_docking_phase:
            fx_rep, fy_rep = 0.0, 0.0
        else:
            fx_rep, fy_rep = self._repulsive_force()

        self._apply_force_as_velocity(fx_att + fx_rep, fy_att + fy_rep)

    # ---------------- forces ----------------

    def _attractive_force(self, target):
        return (self.k_att * (target[0] - self.robot_x),
                self.k_att * (target[1] - self.robot_y))

    def _repulsive_force(self):
        if self.latest_scan is None:
            return 0.0, 0.0
        fx_total, fy_total = 0.0, 0.0
        scan = self.latest_scan
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min < r < self.rho_0:
                a = self.robot_yaw + angle          # same frame as position
                dist = max(r, 0.05)
                mag = self.k_rep * (1.0 / dist - 1.0 / self.rho_0) / (dist ** 2)
                fx_total += -mag * math.cos(a)      # push away from obstacle
                fy_total += -mag * math.sin(a)
            angle += scan.angle_increment

        m = math.hypot(fx_total, fy_total)
        if m > self.max_rep:
            fx_total *= self.max_rep / m
            fy_total *= self.max_rep / m
        return fx_total, fy_total

    def _apply_force_as_velocity(self, fx, fy):
        force_angle = math.atan2(fy, fx)
        force_mag = math.hypot(fx, fy)
        angle_error = self._normalize_angle(force_angle - self.robot_yaw)

        angular_z = max(-self.max_w, min(self.max_w, 2.0 * angle_error))
        if abs(angle_error) > 0.6:
            linear_x = 0.0
        else:
            linear_x = max(0.0, min(self.max_v, 0.5 * force_mag))
        self._publish_cmd(linear_x, angular_z)

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _publish_cmd(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
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
