#!/usr/bin/env python3
"""
A* global planner.

Subscribes:
  /map        (nav_msgs/OccupancyGrid) - the environment map
  /goal_pose  (geometry_msgs/PoseStamped) - goal, e.g. from RViz "2D Nav Goal"
  /odom       (nav_msgs/Odometry) - current robot pose (start of the search)

Publishes:
  /global_path (nav_msgs/Path) - a sparse list of waypoints from robot to goal

The path is simplified (only kept at direction changes) so that the
potential-field planner has a small number of waypoints to chase rather
than every single grid cell.
"""
import math
import heapq

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped


class GlobalPlanner(Node):

    def __init__(self):
        super().__init__('global_planner')

        self.declare_parameter('inflation_radius_cells', 3)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/global_path')

        self.inflation_radius = self.get_parameter('inflation_radius_cells').value
        self.occ_thresh = self.get_parameter('occupied_threshold').value

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_msg = None
        self.inflated_grid = None
        self.robot_pose = None  # (x, y) in world frame

        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                  self.map_cb, map_qos)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                  self.odom_cb, 10)
        self.create_subscription(PoseStamped, self.get_parameter('goal_topic').value,
                                  self.goal_cb, 10)

        self.path_pub = self.create_publisher(Path, self.get_parameter('path_topic').value, 10)

        self.get_logger().info('Global planner (A*) ready. Waiting for map, odom, and goal...')

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.inflated_grid = self._inflate_obstacles(msg)

    def odom_cb(self, msg: Odometry):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def goal_cb(self, msg: PoseStamped):
        if self.map_msg is None:
            self.get_logger().warn('No map received yet, cannot plan.')
            return
        if self.robot_pose is None:
            self.get_logger().warn('No odometry received yet, cannot plan.')
            return

        goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Planning from {self.robot_pose} to {goal_xy}')

        path_cells = self._astar(self.robot_pose, goal_xy)
        if path_cells is None:
            self.get_logger().error('A* failed to find a path to the goal.')
            return

        path_world = [self._grid_to_world(c) for c in path_cells]
        simplified = self._simplify_path(path_world)
        self._publish_path(simplified, msg.header.frame_id or self.map_msg.header.frame_id)
        self.get_logger().info(
            f'Published path with {len(simplified)} waypoints '
            f'(from {len(path_world)} raw cells).'
        )

    # ---------- grid helpers ----------

    def _world_to_grid(self, xy):
        mx = int((xy[0] - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        my = int((xy[1] - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)
        return (mx, my)

    def _grid_to_world(self, cell):
        wx = cell[0] * self.map_msg.info.resolution + self.map_msg.info.origin.position.x
        wy = cell[1] * self.map_msg.info.resolution + self.map_msg.info.origin.position.y
        return (wx, wy)

    def _inflate_obstacles(self, msg: OccupancyGrid):
        """Return a set of occupied (inflated) grid cells for fast lookup."""
        w, h = msg.info.width, msg.info.height
        data = msg.data
        occupied = set()
        for y in range(h):
            row_off = y * w
            for x in range(w):
                if data[row_off + x] >= self.occ_thresh:
                    occupied.add((x, y))

        if self.inflation_radius <= 0:
            return occupied

        inflated = set(occupied)
        r = self.inflation_radius
        for (ox, oy) in occupied:
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if dx * dx + dy * dy <= r * r:
                        nx, ny = ox + dx, oy + dy
                        if 0 <= nx < w and 0 <= ny < h:
                            inflated.add((nx, ny))
        return inflated

    def _is_free(self, cell):
        x, y = cell
        w, h = self.map_msg.info.width, self.map_msg.info.height
        if not (0 <= x < w and 0 <= y < h):
            return False
        return cell not in self.inflated_grid

    # ---------- A* ----------

    def _astar(self, start_xy, goal_xy):
        start = self._world_to_grid(start_xy)
        goal = self._world_to_grid(goal_xy)

        if not self._is_free(start):
            self.get_logger().warn('Start cell is occupied/inflated; searching nearest free cell.')
            start = self._nearest_free(start)
        if not self._is_free(goal):
            self.get_logger().warn('Goal cell is occupied/inflated; searching nearest free cell.')
            goal = self._nearest_free(goal)
        if start is None or goal is None:
            return None

        neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                     (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                     (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        open_set = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for dx, dy, cost in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not self._is_free(nxt):
                    continue
                tentative = g_score[current] + cost
                if tentative < g_score.get(nxt, math.inf):
                    g_score[nxt] = tentative
                    f = tentative + heuristic(nxt, goal)
                    came_from[nxt] = current
                    heapq.heappush(open_set, (f, nxt))

        return None

    def _nearest_free(self, cell, max_radius=15):
        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    c = (cell[0] + dx, cell[1] + dy)
                    if self._is_free(c):
                        return c
        return None

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _simplify_path(self, points, min_spacing=0.3):
        """Keep only points at direction changes, plus enforce a minimum spacing."""
        if len(points) <= 2:
            return points

        simplified = [points[0]]
        prev_dir = None
        for i in range(1, len(points) - 1):
            dx1 = points[i][0] - points[i - 1][0]
            dy1 = points[i][1] - points[i - 1][1]
            dx2 = points[i + 1][0] - points[i][0]
            dy2 = points[i + 1][1] - points[i][1]
            cur_dir = math.atan2(dy1, dx1)
            nxt_dir = math.atan2(dy2, dx2)
            if abs(cur_dir - nxt_dir) > 0.15:  # ~8.5 degrees
                if math.hypot(points[i][0] - simplified[-1][0],
                              points[i][1] - simplified[-1][1]) >= min_spacing:
                    simplified.append(points[i])
        simplified.append(points[-1])
        return simplified

    def _publish_path(self, points, frame_id):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id
        for (x, y) in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
