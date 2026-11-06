"""Goal planner node (test tool): RViz "2D Goal Pose" -> A* on the live /map -> /nav/path.

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
from tf2_ros import Buffer, TransformException, TransformListener

from airmouse_nav.core.astar import plan
from airmouse_nav.core.grid_utils import GridInfo, build_planning_grid, grid_from_occupancy

NODE_NAME = 'goal_planner_node'


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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_msg: OccupancyGrid | None = None
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
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

    def on_goal(self, msg: PoseStamped) -> None:
        """Plan from the drone to the clicked goal and publish the path."""
        if self.map_msg is None:
            self.get_logger().warn('no /map received yet; start localization.launch.py')
            return
        robot_tf = self.lookup_tf(self.robot_frame)
        goal_tf = self.lookup_tf(msg.header.frame_id or self.map_frame)
        if robot_tf is None or goal_tf is None:
            return
        start = (robot_tf.transform.translation.x, robot_tf.transform.translation.y)
        p = msg.pose.position
        goal = transform_xy(goal_tf.transform, p.x, p.y, p.z)

        m = self.map_msg
        t0 = time.perf_counter()
        grid = grid_from_occupancy(m.data, m.info.width, m.info.height)
        info = GridInfo(m.info.resolution, m.info.origin.position.x, m.info.origin.position.y,
                        m.info.height, m.info.width)
        pg = build_planning_grid(grid, info, self.guarded_radius, self.safety_margin,
                                 self.wall_cost_range)
        t1 = time.perf_counter()
        result = plan(pg, start, goal, wall_cost_weight=self.wall_cost_weight)
        t2 = time.perf_counter()
        self.publish_grid(m, pg)

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(result.path):
            nx, ny = result.path[min(i + 1, len(result.path) - 1)]
            px, py = result.path[max(i - 1, 0)] if i == len(result.path) - 1 else (x, y)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation = yaw_quaternion(math.atan2(ny - py, nx - px))
            path.poses.append(pose)
        self.path_pub.publish(path)   # empty path on failure clears the old one in RViz

        where = f'({start[0]:.2f}, {start[1]:.2f}) -> ({goal[0]:.2f}, {goal[1]:.2f})'
        timing = f'grid {1e3 * (t1 - t0):.0f} ms, A* {1e3 * (t2 - t1):.0f} ms'
        if result.success:
            snapped = ', start moved out of inflation' if result.start_snapped else ''
            self.get_logger().info(f'path {where}: {result.length:.2f} m, '
                                   f'{len(result.path)} points ({timing}{snapped})')
        else:
            self.get_logger().warn(f'no path {where}: {result.reason} ({timing})')

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
