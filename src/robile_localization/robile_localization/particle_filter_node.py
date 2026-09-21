#!/usr/bin/env python3
"""
Monte Carlo Localisation (particle filter), simple/vanilla version.
"""
import math
import random

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped, TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class ParticleFilter(Node):

    def __init__(self):
        super().__init__('particle_filter')

        self.declare_parameter('num_particles', 300)
        self.declare_parameter('initial_spread_xy', 0.3)
        self.declare_parameter('initial_spread_theta', 0.3)
        self.declare_parameter('alpha1', 0.05)
        self.declare_parameter('alpha2', 0.05)
        self.declare_parameter('alpha3', 0.05)
        self.declare_parameter('alpha4', 0.02)
        self.declare_parameter('laser_subsample', 10)
        self.declare_parameter('z_hit', 0.9)
        self.declare_parameter('z_rand', 0.1)
        self.declare_parameter('sigma_hit', 0.2)
        self.declare_parameter('max_laser_range', 8.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 10.0)

        self.num_particles = self.get_parameter('num_particles').value
        self.init_spread_xy = self.get_parameter('initial_spread_xy').value
        self.init_spread_theta = self.get_parameter('initial_spread_theta').value
        self.a1 = self.get_parameter('alpha1').value
        self.a2 = self.get_parameter('alpha2').value
        self.a3 = self.get_parameter('alpha3').value
        self.a4 = self.get_parameter('alpha4').value
        self.laser_subsample = self.get_parameter('laser_subsample').value
        self.z_hit = self.get_parameter('z_hit').value
        self.z_rand = self.get_parameter('z_rand').value
        self.sigma_hit = self.get_parameter('sigma_hit').value
        self.max_laser_range = self.get_parameter('max_laser_range').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        self.particles = None
        self.weights = None
        self.map_msg = None
        self.likelihood_field = None
        self.last_odom_pose = None
        self.initialized = False

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                  self.map_cb, map_qos)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                  self.odom_cb, 10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                  self.scan_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                  self.initialpose_cb, 10)

        self.cloud_pub = self.create_publisher(PoseArray, '/particle_cloud', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/pf_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        period = 1.0 / self.get_parameter('publish_rate').value
        self.create_timer(period, self.publish_estimate)

        self.get_logger().info(
            'Particle filter ready. Waiting for map and an initial pose '
            '(use RViz "2D Pose Estimate").'
        )

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.likelihood_field = self._build_likelihood_field(msg)
        self.get_logger().info('Map received; likelihood field built.')

    def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._initialize_particles(x, y, theta)
        self.get_logger().info(f'Initialized {self.num_particles} particles around '
                                f'({x:.2f}, {y:.2f}, {theta:.2f}).')

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self.initialized:
            self._initialize_particles(x, y, theta)
            self.last_odom_pose = (x, y, theta)
            return

        self._predict((x, y, theta))
        self.last_odom_pose = (x, y, theta)

    def scan_cb(self, msg: LaserScan):
        if not self.initialized or self.likelihood_field is None:
            return
        self._update_weights(msg)
        self._resample()

    def _initialize_particles(self, x, y, theta):
        n = self.num_particles
        self.particles = np.zeros((n, 3))
        self.particles[:, 0] = np.random.normal(x, self.init_spread_xy, n)
        self.particles[:, 1] = np.random.normal(y, self.init_spread_xy, n)
        self.particles[:, 2] = np.random.normal(theta, self.init_spread_theta, n)
        self.weights = np.ones(n) / n
        self.initialized = True

    def _predict(self, new_odom_pose):
        x0, y0, th0 = self.last_odom_pose
        x1, y1, th1 = new_odom_pose

        d_trans = math.hypot(x1 - x0, y1 - y0)
        if d_trans < 1e-6:
            d_rot1 = 0.0
        else:
            d_rot1 = normalize_angle(math.atan2(y1 - y0, x1 - x0) - th0)
        d_rot2 = normalize_angle(th1 - th0 - d_rot1)

        n = self.particles.shape[0]

        rot1_noise = np.random.normal(
            0.0, self.a1 * abs(d_rot1) + self.a2 * d_trans, n)
        trans_noise = np.random.normal(
            0.0, self.a3 * d_trans + self.a4 * (abs(d_rot1) + abs(d_rot2)), n)
        rot2_noise = np.random.normal(
            0.0, self.a1 * abs(d_rot2) + self.a2 * d_trans, n)

        rot1_hat = d_rot1 - rot1_noise
        trans_hat = d_trans - trans_noise
        rot2_hat = d_rot2 - rot2_noise

        self.particles[:, 0] += trans_hat * np.cos(self.particles[:, 2] + rot1_hat)
        self.particles[:, 1] += trans_hat * np.sin(self.particles[:, 2] + rot1_hat)
        self.particles[:, 2] += rot1_hat + rot2_hat
        self.particles[:, 2] = np.array([normalize_angle(a) for a in self.particles[:, 2]])

    def _build_likelihood_field(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        data = msg.data
        res = msg.info.resolution

        dist = np.full((h, w), np.inf, dtype=np.float32)
        from collections import deque
        q = deque()

        for y in range(h):
            row = y * w
            for x in range(w):
                if data[row + x] >= 50:
                    dist[y, x] = 0.0
                    q.append((x, y))

        while q:
            x, y = q.popleft()
            d = dist[y, x]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    nd = d + 1.0
                    if nd < dist[ny, nx]:
                        dist[ny, nx] = nd
                        q.append((nx, ny))

        dist[np.isinf(dist)] = dist[np.isfinite(dist)].max() if np.isfinite(dist).any() else 0.0
        return dist * res

    def _world_to_grid(self, x, y):
        mx = int((x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        my = int((y - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)
        return mx, my

    def _update_weights(self, scan: LaserScan):
        h, w = self.likelihood_field.shape
        n = self.particles.shape[0]
        new_weights = np.zeros(n)

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges = scan.ranges
        step = max(1, self.laser_subsample)

        two_sigma_sq = 2.0 * self.sigma_hit * self.sigma_hit

        for i in range(n):
            px, py, ptheta = self.particles[i]
            log_prob = 0.0
            for j in range(0, len(ranges), step):
                r = ranges[j]
                if not math.isfinite(r) or r >= self.max_laser_range or r <= scan.range_min:
                    continue
                beam_angle = ptheta + angle_min + j * angle_inc
                ex = px + r * math.cos(beam_angle)
                ey = py + r * math.sin(beam_angle)
                gx, gy = self._world_to_grid(ex, ey)

                if 0 <= gx < w and 0 <= gy < h:
                    dist_to_obstacle = self.likelihood_field[gy, gx]
                else:
                    dist_to_obstacle = self.max_laser_range

                prob = (self.z_hit * math.exp(-(dist_to_obstacle ** 2) / two_sigma_sq)
                        + self.z_rand / self.max_laser_range)
                log_prob += math.log(max(prob, 1e-9))

            new_weights[i] = math.exp(log_prob)

        total = new_weights.sum()
        if total > 1e-12:
            self.weights = new_weights / total
        else:
            self.weights = np.ones(n) / n

    def _resample(self):
        n = self.particles.shape[0]
        positions = (np.arange(n) + random.random()) / n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0

        indices = np.searchsorted(cumulative, positions)
        self.particles = self.particles[indices]
        self.weights = np.ones(n) / n

    def publish_estimate(self):
        if not self.initialized:
            return

        mean_x = float(np.average(self.particles[:, 0], weights=self.weights))
        mean_y = float(np.average(self.particles[:, 1], weights=self.weights))
        sin_sum = float(np.average(np.sin(self.particles[:, 2]), weights=self.weights))
        cos_sum = float(np.average(np.cos(self.particles[:, 2]), weights=self.weights))
        mean_theta = math.atan2(sin_sum, cos_sum)

        now = self.get_clock().now().to_msg()

        cloud = PoseArray()
        cloud.header.stamp = now
        cloud.header.frame_id = self.map_frame
        for x, y, theta in self.particles:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            q = tf_transformations.quaternion_from_euler(0, 0, float(theta))
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
            cloud.poses.append(p)
        self.cloud_pub.publish(cloud)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.pose.position.x = mean_x
        pose_msg.pose.pose.position.y = mean_y
        q = tf_transformations.quaternion_from_euler(0, 0, mean_theta)
        (pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y,
         pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w) = q
        self.pose_pub.publish(pose_msg)

        if self.last_odom_pose is not None:
            self._broadcast_map_to_odom(mean_x, mean_y, mean_theta, now)

    def _broadcast_map_to_odom(self, mx, my, mtheta, stamp):
        ox, oy, otheta = self.last_odom_pose

        map_to_base = tf_transformations.compose_matrix(
            translate=[mx, my, 0], angles=[0, 0, mtheta])
        odom_to_base = tf_transformations.compose_matrix(
            translate=[ox, oy, 0], angles=[0, 0, otheta])
        base_to_odom = np.linalg.inv(odom_to_base)
        map_to_odom = np.dot(map_to_base, base_to_odom)

        trans = tf_transformations.translation_from_matrix(map_to_odom)
        quat = tf_transformations.quaternion_from_matrix(map_to_odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(trans[0])
        t.transform.translation.y = float(trans[1])
        t.transform.translation.z = 0.0
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
