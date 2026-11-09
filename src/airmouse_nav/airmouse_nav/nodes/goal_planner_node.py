"""Goal planner node (test tool): RViz "2D Goal Pose" -> A* on the live /map -> /nav/path.

From Phase 4 the goal stays active: the node replans from the drone's current position every
`replan_period` s (so the path stays fresh for the controller's path_timeout and follows the
growing map) until the controller reports "reached" with the drone within goal_tolerance_xy.
Path z = flight_altitude; the last pose carries the clicked goal yaw. Unknown cells within
guarded_radius + safety_margin + 0.1 m of the drone count as free (the drone is there).

Subscribes:  /map (nav_msgs/OccupancyGrid), /goal_pose (geometry_msgs/PoseStamped, any frame
             with a TF to the map frame; RViz publishes in its fixed frame).
Publishes:   /nav/path (nav_msgs/Path, map frame) on every new goal;
             /nav/planning_grid (nav_msgs/OccupancyGrid, debug): the inflated grid used for
             the plan (100 = blocked, 0..99 = wall cost x 99).
Start pose:  TF map -> base_link at the time of the click.
Parameters:  guarded_radius, safety_margin, wall_cost_weight, wall_cost_range (planner.yaml);
             map_frame ('map'), robot_frame ('base_link').
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from airmouse_nav.core.astar import plan
from airmouse_nav.core.grid_utils import GridInfo, build_planning_grid, grid_from_occupancy

NODE_NAME = 'goal_planner_node'


def clear_unknown_around(grid: np.ndarray, info: GridInfo, xy: tuple[float, float],
                         radius: float) -> np.ndarray:
    """Copy of `grid` with unknown cells within `radius` of the drone marked free.

    The drone is physically there, so that space is free; the camera cannot see within 0.3 m
    of itself, so without this a camera-only map leaves the drone's own spot unknown (=
    blocked) after take-off. Known obstacles are never cleared.
    """
    rows, cols = np.ogrid[0:info.rows, 0:info.cols]
    cx = info.origin_x + (cols + 0.5) * info.resolution
    cy = info.origin_y + (rows + 0.5) * info.resolution
    near = (cx - xy[0]) ** 2 + (cy - xy[1]) ** 2 <= radius ** 2
    out = grid.copy()
    out[near & (grid < 0)] = 0
    return out


def transform_xy(tf, x: float, y: float, z: float = 0.0) -> tuple[float, float]:
    """Apply a geometry_msgs/Transform to a point and return the new (x, y)."""
    q, t = tf.rotation, tf.translation
    # Rotate v by unit quaternion q: v' = v + 2w(u x v) + 2u x (u x v), u = (qx, qy, qz).
    ux, uy, uz, w = q.x, q.y, q.z, q.w
    cx, cy, cz = uy * z - uz * y, uz * x - ux * z, ux * y - uy * x
    ccx, ccy = uy * cz - uz * cy, uz * cx - ux * cz
    return (x + 2 * (w * cx + ccx) + t.x, y + 2 * (w * cy + ccy) + t.y)


def yaw_quaternion(yaw: float) -> Quaternion:
    """Quaternion for a rotation of `yaw` radians about z."""
    return Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class GoalPlannerNode(Node):
    """Plans an A* path to each RViz goal on the latest map."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.guarded_radius = self.declare_parameter('guarded_radius', 0.30).value
        self.safety_margin = self.declare_parameter('safety_margin', 0.10).value
        self.wall_cost_weight = self.declare_parameter('wall_cost_weight', 2.0).value
        self.wall_cost_range = self.declare_parameter('wall_cost_range', 0.5).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        self.flight_altitude = self.declare_parameter('flight_altitude', 1.2).value
        self.goal_tolerance_xy = self.declare_parameter('goal_tolerance_xy', 0.15).value
        self.replan_period = self.declare_parameter('replan_period', 1.0).value
        self.goal: tuple[float, float, float] | None = None   # map-frame x, y, yaw
        self.controller_state = ''

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_msg: OccupancyGrid | None = None
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(String, '/nav/controller_state', self.on_state, 10)
        self.create_timer(self.replan_period, self.on_timer)
        self.path_pub = self.create_publisher(Path, '/nav/path', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/nav/planning_grid', latched)
        self.get_logger().info(
            f'ready: inflation {self.guarded_radius + self.safety_margin:.2f} m, '
            f'wall_cost_weight {self.wall_cost_weight}; click "2D Goal Pose" in RViz')

    def on_map(self, msg: OccupancyGrid) -> None:
        """Keep the latest map."""
        self.map_msg = msg

    def lookup_tf(self, frame: str):
        """Latest transform from `frame` into the map frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, frame, Time(),
                                                 timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self.get_logger().warn(f'no TF {self.map_frame} -> {frame}: {exc}')
            return None
        return tf

    def on_state(self, msg: String) -> None:
        """Remember the controller's state."""
        self.controller_state = msg.data

    def on_goal(self, msg: PoseStamped) -> None:
        """Accept a new goal (any frame with a TF to the map) and plan to it."""
        goal_tf = self.lookup_tf(msg.header.frame_id or self.map_frame)
        if goal_tf is None:
            return
        p = msg.pose.position
        gx, gy = transform_xy(goal_tf.transform, p.x, p.y, p.z)
        q, r = msg.pose.orientation, goal_tf.transform.rotation
        yaw = (math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
               + math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z)))
        self.goal = (gx, gy, yaw)
        self.controller_state = ''
        self.replan(first=True)

    def on_timer(self) -> None:
        """Replan the active goal; drop it once the controller has reached it."""
        if self.goal is None:
            return
        robot_tf = self.lookup_tf(self.robot_frame)
        if robot_tf is None:
            return
        t = robot_tf.transform.translation
        if (self.controller_state == 'reached'
                and math.hypot(t.x - self.goal[0], t.y - self.goal[1]) <= self.goal_tolerance_xy):
            self.get_logger().info('goal reached')
            self.goal = None
            return
        self.replan(first=False)

    def replan(self, first: bool) -> None:
        """Plan from the drone to the active goal; publish the path (empty on a first-plan failure)."""
        if self.map_msg is None:
            self.get_logger().warn('no /map received yet; start localization.launch.py')
            return
        robot_tf = self.lookup_tf(self.robot_frame)
        if robot_tf is None:
            return
        start = (robot_tf.transform.translation.x, robot_tf.transform.translation.y)
        goal = self.goal[:2]

        m = self.map_msg
        t0 = time.perf_counter()
        grid = grid_from_occupancy(m.data, m.info.width, m.info.height)
        info = GridInfo(m.info.resolution, m.info.origin.position.x, m.info.origin.position.y,
                        m.info.height, m.info.width)
        grid = clear_unknown_around(grid, info, start,
                                    self.guarded_radius + self.safety_margin + 0.1)
        pg = build_planning_grid(grid, info, self.guarded_radius, self.safety_margin,
                                 self.wall_cost_range)
        t1 = time.perf_counter()
        result = plan(pg, start, goal, wall_cost_weight=self.wall_cost_weight)
        t2 = time.perf_counter()
        if first:
            self.publish_grid(m, pg)

        where = f'({start[0]:.2f}, {start[1]:.2f}) -> ({goal[0]:.2f}, {goal[1]:.2f})'
        timing = f'grid {1e3 * (t1 - t0):.0f} ms, A* {1e3 * (t2 - t1):.0f} ms'
        if not result.success:
            self.get_logger().warn(f'no path {where}: {result.reason} ({timing})')
            if first:
                self.goal = None
                self.path_pub.publish(self.make_path([]))   # clears the old path
            return   # replan failure: keep the last path; the controller times it out
        points = list(result.path)
        if result.start_snapped:
            points.insert(0, start)   # include the short move out of the inflation
        self.path_pub.publish(self.make_path(points))
        if first:
            snapped = ', start moved out of inflation' if result.start_snapped else ''
            self.get_logger().info(f'path {where}: {result.length:.2f} m, '
                                   f'{len(result.path)} points ({timing}{snapped})')

    def make_path(self, points: list[tuple[float, float]]) -> Path:
        """nav_msgs/Path at flight altitude; headings along the path, goal yaw at the end."""
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(points):
            if i == len(points) - 1:
                yaw = self.goal[2]
            else:
                nx, ny = points[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.position.z = self.flight_altitude
            pose.pose.orientation = yaw_quaternion(yaw)
            path.poses.append(pose)
        return path

    def publish_grid(self, m: OccupancyGrid, pg) -> None:
        """Publish the inflated grid (100 = blocked, else wall cost x 99) for RViz."""
        out = OccupancyGrid()
        out.header = m.header
        out.info = m.info
        values = np.where(pg.blocked, 100, np.round(pg.wall_cost * 99)).astype(np.int8)
        out.data = values.ravel().tolist()
        self.grid_pub.publish(out)


def main(args: list[str] | None = None) -> None:
    """Run the goal planner node until shutdown.

    Args:
        args: Command-line arguments forwarded to rclpy.
    """
    rclpy.init(args=args)
    node = GoalPlannerNode()
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
