#!/usr/bin/env python3
"""
Monte Carlo Localisation (particle filter) for the Robile.

- Motion model: odometry delta expressed in the robot's own frame
  (works for holonomic bases, no atan2 singularity at standstill).
- Measurement model: likelihood field, vectorised with numpy.
- Updates only after the robot has moved; resamples only when the
  effective sample size drops (avoids particle depletion).
- Laser pose relative to base_link is taken from TF.
- Publishes /particle_cloud, /pf_pose and TF map -> odom.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (Pose, PoseArray, PoseWithCovarianceStamped,
                               TransformStamped)
import tf_transformations
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def wrap_array(a):
    return np.arctan2(np.sin(a), np.cos(a))


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
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 10.0)
        # new tuning parameters (defaults are fine)
        self.declare_parameter('likelihood_temper', 0.5)
        self.declare_parameter('update_min_d', 0.05)
        self.declare_parameter('update_min_a', 0.1)
        self.declare_parameter('transform_tolerance', 0.2)

        gp = lambda n: self.get_parameter(n).value
        self.num_particles = int(gp('num_particles'))
        self.init_spread_xy = gp('initial_spread_xy')
        self.init_spread_theta = gp('initial_spread_theta')
        self.a1, self.a2, self.a3, self.a4 = (gp('alpha1'), gp('alpha2'),
                                              gp('alpha3'), gp('alpha4'))
        self.laser_subsample = int(gp('laser_subsample'))
        self.z_hit = gp('z_hit')
        self.z_rand = gp('z_rand')
        self.sigma_hit = gp('sigma_hit')
        self.max_laser_range = gp('max_laser_range')
        self.base_frame = gp('base_frame')
        self.odom_frame = gp('odom_frame')
        self.map_frame = gp('map_frame')
        self.temper = gp('likelihood_temper')
        self.update_min_d = gp('update_min_d')
        self.update_min_a = gp('update_min_a')
        self.tf_tolerance = gp('transform_tolerance')

        # state
        self.particles = np.zeros((self.num_particles, 3))
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.initialized = False
        self.odom_pose = None          # latest (x, y, th) from /odom
        self.last_odom_pose = None     # odom pose at previous predict step
        self.last_update_odom = None   # odom pose at previous weight update
        self.force_update = False
        self.map_msg = None
        self.likelihood_field = None
        self.laser_offset = None       # (lx, ly, lyaw) of laser in base_frame
        self._laser_warned = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, gp('map_topic'), self.map_cb, map_qos)
        self.create_subscription(Odometry, gp('odom_topic'), self.odom_cb, 20)
        self.create_subscription(LaserScan, gp('scan_topic'), self.scan_cb, 5)
        self.create_subscription(PoseWithCovarianceStamped, gp('initialpose_topic'),
                                 self.initialpose_cb, 10)

        self.cloud_pub = self.create_publisher(PoseArray, '/particle_cloud', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/pf_pose', 10)

        self.create_timer(1.0 / gp('publish_rate'), self.publish_estimate)
        self.get_logger().info('Particle filter ready. Waiting for map, odom and scan...')

    # ------------------------------------------------------------------ callbacks

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.likelihood_field = self._build_likelihood_field(msg)
        self.get_logger().info(
            f'Map received ({msg.info.width}x{msg.info.height}, '
            f'res {msg.info.resolution:.3f}); likelihood field built.')

    def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, th = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._initialize_particles(x, y, th)
        if self.odom_pose is not None:
            self.last_odom_pose = self.odom_pose
            self.last_update_odom = self.odom_pose
        self.force_update = True
        self.get_logger().info(
            f'Initialized {self.num_particles} particles around '
            f'({x:.2f}, {y:.2f}, {th:.2f}).')

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, th = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        new_pose = (x, y, th)
        self.odom_pose = new_pose

        if not self.initialized:
            self._initialize_particles(x, y, th)
            self.last_odom_pose = new_pose
            self.last_update_odom = new_pose
            self.force_update = True
            return

        if self.last_odom_pose is None:
            self.last_odom_pose = new_pose
            return

        self._predict(new_pose)
        self.last_odom_pose = new_pose

    def scan_cb(self, msg: LaserScan):
        if (not self.initialized or self.likelihood_field is None
                or self.odom_pose is None):
            return

        if self.laser_offset is None:
            self._lookup_laser_offset(msg.header.frame_id)

        # only update when the robot has moved (or right after initialisation)
        if not self.force_update and self.last_update_odom is not None:
            ox, oy, oth = self.odom_pose
            lx, ly, lth = self.last_update_odom
            moved = math.hypot(ox - lx, oy - ly) > self.update_min_d
            turned = abs(normalize_angle(oth - lth)) > self.update_min_a
            if not (moved or turned):
                return

        if self._update_weights(msg):
            self.last_update_odom = self.odom_pose
            self.force_update = False
            self._resample_if_needed()

    # ------------------------------------------------------------------ helpers

    def _lookup_laser_offset(self, laser_frame):
        offset = (0.0, 0.0, 0.0)
        if laser_frame and laser_frame != self.base_frame:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, laser_frame, Time())
                q = t.transform.rotation
                _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                offset = (t.transform.translation.x, t.transform.translation.y, yaw)
            except (LookupException, ConnectivityException, ExtrapolationException):
                if not self._laser_warned:
                    self.get_logger().warn(
                        f'TF {self.base_frame}->{laser_frame} not available yet; '
                        f'assuming laser at base origin.')
                    self._laser_warned = True
                return  # retry on next scan (keep self.laser_offset None)
        self.laser_offset = offset
        self.get_logger().info(
            f'Laser offset in {self.base_frame}: x={offset[0]:.3f} y={offset[1]:.3f} '
            f'yaw={offset[2]:.3f}')

    def _initialize_particles(self, x, y, theta):
        n = self.num_particles
        self.particles = np.zeros((n, 3))
        self.particles[:, 0] = np.random.normal(x, self.init_spread_xy, n)
        self.particles[:, 1] = np.random.normal(y, self.init_spread_xy, n)
        self.particles[:, 2] = wrap_array(np.random.normal(theta, self.init_spread_theta, n))
        self.weights = np.ones(n) / n
        self.initialized = True

    def _predict(self, new_pose):
        x0, y0, th0 = self.last_odom_pose
        x1, y1, th1 = new_pose

        dx, dy = x1 - x0, y1 - y0
        dth = normalize_angle(th1 - th0)

        # odom delta in the robot's previous body frame
        dxb = math.cos(th0) * dx + math.sin(th0) * dy
        dyb = -math.sin(th0) * dx + math.cos(th0) * dy

        trans = math.hypot(dxb, dyb)
        rot = abs(dth)
        if trans < 1e-5 and rot < 1e-5:
            return

        n = self.particles.shape[0]
        std_xy = self.a3 * trans + self.a4 * rot
        std_th = self.a1 * rot + self.a2 * trans

        ndx = dxb + np.random.normal(0.0, std_xy, n)
        ndy = dyb + np.random.normal(0.0, std_xy, n)
        ndth = dth + np.random.normal(0.0, std_th, n)

        th = self.particles[:, 2].copy()
        self.particles[:, 0] += np.cos(th) * ndx - np.sin(th) * ndy
        self.particles[:, 1] += np.sin(th) * ndx + np.cos(th) * ndy
        self.particles[:, 2] = wrap_array(th + ndth)

    def _build_likelihood_field(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        grid = np.array(msg.data, dtype=np.int16).reshape(h, w)
        occ = grid >= 50
        if not occ.any():
            return np.full((h, w), self.max_laser_range, dtype=np.float32)

        try:
            from scipy.ndimage import distance_transform_edt
            return (distance_transform_edt(~occ) * res).astype(np.float32)
        except ImportError:
            from collections import deque
            dist = np.full((h, w), np.inf, dtype=np.float32)
            q = deque()
            ys, xs = np.nonzero(occ)
            for x, y in zip(xs, ys):
                dist[y, x] = 0.0
                q.append((x, y))
            while q:
                x, y = q.popleft()
                d = dist[y, x]
                for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = x + ddx, y + ddy
                    if 0 <= nx < w and 0 <= ny < h and d + 1.0 < dist[ny, nx]:
                        dist[ny, nx] = d + 1.0
                        q.append((nx, ny))
            dist[np.isinf(dist)] = dist[np.isfinite(dist)].max()
            return dist * res

    def _update_weights(self, scan: LaserScan):
        h, w = self.likelihood_field.shape
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        idx = np.arange(0, len(ranges), max(1, self.laser_subsample))
        r = ranges[idx]
        ang = scan.angle_min + idx * scan.angle_increment
        valid = np.isfinite(r) & (r > scan.range_min) & (r < self.max_laser_range)
        if valid.sum() < 5:
            return False
        r = r[valid]
        ang = ang[valid]

        lx, ly, lyaw = self.laser_offset if self.laser_offset is not None else (0.0, 0.0, 0.0)
        bx = lx + r * np.cos(ang + lyaw)      # beam end points in base frame
        by = ly + r * np.sin(ang + lyaw)

        px = self.particles[:, 0][:, None]
        py = self.particles[:, 1][:, None]
        th = self.particles[:, 2][:, None]
        ex = px + bx[None, :] * np.cos(th) - by[None, :] * np.sin(th)
        ey = py + bx[None, :] * np.sin(th) + by[None, :] * np.cos(th)

        info = self.map_msg.info
        gx = np.floor((ex - info.origin.position.x) / info.resolution).astype(np.int64)
        gy = np.floor((ey - info.origin.position.y) / info.resolution).astype(np.int64)
        inside = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)

        d = np.full(ex.shape, self.max_laser_range)
        d[inside] = self.likelihood_field[gy[inside], gx[inside]]

        prob = (self.z_hit * np.exp(-(d ** 2) / (2.0 * self.sigma_hit ** 2))
                + self.z_rand / self.max_laser_range)
        logp = np.log(np.maximum(prob, 1e-9)).sum(axis=1)

        logw = self.temper * (logp - logp.max())
        new_w = self.weights * np.exp(logw)
        s = new_w.sum()
        if s > 1e-300 and np.isfinite(s):
            self.weights = new_w / s
        else:
            self.weights = np.ones(len(self.weights)) / len(self.weights)
        return True

    def _resample_if_needed(self):
        n = self.particles.shape[0]
        neff = 1.0 / np.sum(self.weights ** 2)
        if neff >= 0.5 * n:
            return
        positions = (np.arange(n) + np.random.uniform()) / n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0
        indices = np.searchsorted(cumulative, positions)
        self.particles = self.particles[indices]
        self.weights = np.full(n, 1.0 / n)

    # ------------------------------------------------------------------ output

    def publish_estimate(self):
        if not self.initialized or self.odom_pose is None:
            return

        mean_x = float(np.average(self.particles[:, 0], weights=self.weights))
        mean_y = float(np.average(self.particles[:, 1], weights=self.weights))
        s = float(np.average(np.sin(self.particles[:, 2]), weights=self.weights))
        c = float(np.average(np.cos(self.particles[:, 2]), weights=self.weights))
        mean_theta = math.atan2(s, c)

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

        self._broadcast_map_to_odom(mean_x, mean_y, mean_theta)

    def _broadcast_map_to_odom(self, mx, my, mth):
        ox, oy, oth = self.odom_pose
        # T_map_odom = T_map_base * inverse(T_odom_base)
        th = normalize_angle(mth - oth)
        tx = mx - (math.cos(th) * ox - math.sin(th) * oy)
        ty = my - (math.sin(th) * ox + math.cos(th) * oy)

        t = TransformStamped()
        t.header.stamp = (self.get_clock().now()
                          + Duration(seconds=self.tf_tolerance)).to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, th)
        t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
