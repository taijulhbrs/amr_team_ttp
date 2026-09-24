#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_frontier_size', 6)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('goal_tolerance', 0.4)
        self.declare_parameter('goal_timeout', 25.0)
        self.declare_parameter('explore_check_period', 2.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.occ_thresh = self.get_parameter('occupied_threshold').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                  self.map_cb, map_qos)

        self.goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter('goal_topic').value, 10)

        self.map_msg = None
        self.current_goal = None
        self.goal_set_time = None
        self.exploration_complete = False

        period = self.get_parameter('explore_check_period').value
        self.create_timer(period, self.explore_step)

        self.get_logger().info('Frontier explorer ready. Waiting for map...')

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def explore_step(self):
        if self.exploration_complete:
            return
        if self.map_msg is None:
            return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn('No robot pose available yet, cannot explore.')
            return

        if self.current_goal is not None:
            dist = math.hypot(self.current_goal[0] - robot_pose[0],
                               self.current_goal[1] - robot_pose[1])
            elapsed = (self.get_clock().now() - self.goal_set_time).nanoseconds / 1e9

            if dist > self.goal_tolerance and elapsed < self.goal_timeout:
                return

        frontiers = self._find_frontier_clusters()
        if not frontiers:
            if not self.exploration_complete:
                self.get_logger().info(
                    '🗺️  Exploration complete! No more frontiers found.')
            self.exploration_complete = True
            return

        target = self._choose_target(frontiers, robot_pose)
        self._publish_goal(target)
        self.current_goal = target
        self.goal_set_time = self.get_clock().now()
        self.get_logger().info(
            f'New exploration goal: ({target[0]:.2f}, {target[1]:.2f}) '
            f'— {len(frontiers)} frontier region(s) available.')

    def _find_frontier_clusters(self):
        msg = self.map_msg
        w, h = msg.info.width, msg.info.height
        data = msg.data

        def idx(x, y):
            return y * w + x

        def is_free(x, y):
            v = data[idx(x, y)]
            return 0 <= v < self.occ_thresh

        def is_unknown(x, y):
            return data[idx(x, y)] == -1

        frontier_cells = set()
        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if not is_free(x, y):
                    continue
                if (is_unknown(x - 1, y) or is_unknown(x + 1, y) or
                        is_unknown(x, y - 1) or is_unknown(x, y + 1)):
                    frontier_cells.add((x, y))

        clusters = []
        visited = set()
        for cell in frontier_cells:
            if cell in visited:
                continue
            cluster = []
            q = deque([cell])
            visited.add(cell)
            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    n = (cx + dx, cy + dy)
                    if n in frontier_cells and n not in visited:
                        visited.add(n)
                        q.append(n)
            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)

        centroids = []
        for cluster in clusters:
            avg_x = sum(c[0] for c in cluster) / len(cluster)
            avg_y = sum(c[1] for c in cluster) / len(cluster)
            wx = avg_x * msg.info.resolution + msg.info.origin.position.x
            wy = avg_y * msg.info.resolution + msg.info.origin.position.y
            centroids.append((wx, wy))

        return centroids

    def _choose_target(self, frontiers, robot_pose):
        return min(frontiers, key=lambda f: math.hypot(
            f[0] - robot_pose[0], f[1] - robot_pose[1]))

    def _publish_goal(self, target):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
