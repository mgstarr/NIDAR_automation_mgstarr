"""Explorer node: runs the mission (core/mission.py) and exploration (core/viewpoint.py).

Subscribes: /map, /nav/controller_state, TF map -> base_link, optional /battery (logged only).
Publishes:  /nav/path (to the controller; republished at least every replan_period),
            /nav/goal (chosen viewpoint), /nav/frontiers (MarkerArray: map frontiers orange,
            view frontiers cyan), /nav/coverage (OccupancyGrid: 0 = seen, 100 = free but unseen,
            -1 = other), /mission/state (String, latched; also republished every second).
Services:   /mission/start, /mission/abort, /mission/return_home (std_srvs/Trigger).

Per state (tick at 10 Hz):
  TAKEOFF      vertical path to flight_altitude at the start point (= home).
  EXPLORE      360-degree look-around first (four 90-degree turns), then viewpoint replans at
               replan_period, on goal reached, or when > large_map_change of known cells
               changed. Heading modes: use_lidar true -> the path carries the viewpoint yaw
               (the controller turns before arrival); false -> the path keeps the travel
               heading and the drone turns to the viewpoint yaw after arriving.
               -> RETURN_HOME when no viewpoint is left, or by the return-budget rule
               (Dijkstra from home, refreshed every 10 s).
  RETURN_HOME  A* to home, replanned at replan_period.
  LAND         vertical path to z = 0 at home.
  ABORT        hold position for 2 s, then a vertical landing path where the drone is.
Aborts: /mission/abort, no pose for localization_lost_timeout, or the pose outside
arena_bounds (geofence breach).
"""

from __future__ import annotations

import math
import time  # noqa: F401  (perf_counter)

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger as TriggerSrv
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from airmouse_nav.core.astar import plan
from airmouse_nav.core.coverage import Coverage
from airmouse_nav.core.grid_utils import (GridInfo, build_planning_grid, cell_to_world,
                                          clear_unknown_around, grid_from_occupancy,
                                          world_to_cell)
from airmouse_nav.core.mission import Mission, State, Trigger, should_return
from airmouse_nav.core.viewpoint import ExplorationPlanner, ExplorerParams, dijkstra_costs

NODE_NAME = 'explorer_node'
TICK = 0.1                 # s
LOOK_STEPS = 4             # look-around: four 90-degree turns
ABORT_HOLD = 2.0           # s of hover before landing in place


def yaw_of(q) -> float:
    """Yaw of a quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ExplorerNode(Node):
    """Mission + exploration."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        d = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.replan_period = d('replan_period', 1.0)
        self.time_limit = d('mission_time_limit', 1800.0)
        self.reserve = d('return_reserve', 120.0)
        self.safety_factor = d('return_safety_factor', 1.5)
        self.loc_timeout = d('localization_lost_timeout', 3.0)
        self.bounds = list(d('arena_bounds', [-0.6, 13.8, -7.2, 7.2]))
        self.landing_time = d('landing_time', 15.0)
        self.large_change = d('large_map_change', 0.02)
        self.use_lidar = d('use_lidar', True)
        self.flight_altitude = d('flight_altitude', 1.2)
        self.max_speed = d('max_speed', 0.6)
        self.tol = d('goal_tolerance_xy', 0.15)
        self.p = ExplorerParams(
            camera_hfov=math.radians(d('camera_hfov', 87.0)),
            coverage_range=d('coverage_range', 2.5),
            min_cluster_size=int(d('min_cluster_size', 6)),
            gain_weight=d('gain_weight', 1.0),
            goal_switch_margin=d('goal_switch_margin', 0.2),
            stuck_timeout=d('stuck_timeout', 10.0),
            guarded_radius=d('guarded_radius', 0.30),
            safety_margin=d('safety_margin', 0.10),
            wall_cost_weight=d('wall_cost_weight', 2.0),
            wall_cost_range=d('wall_cost_range', 0.5))

        self.mission = Mission()
        self.explorer = ExplorationPlanner(self.p)
        self.coverage: Coverage | None = None
        self.map_msg: OccupancyGrid | None = None
        self.map_known = 0
        self.map_known_at_plan = 0
        self.controller_state = ''
        self.pose = None              # (x, y, z, yaw)
        self.pose_time = -math.inf    # wall time of the last good pose
        self.home = None
        self.t_start = None
        self.state_since = self.now()
        self.path_points: list[tuple[float, float, float]] = []
        self.path_yaw: float | None = None
        self.last_publish = 0.0
        self.next_replan = 0.0
        self.look_step = 0
        self.turning_to: float | None = None
        self.home_costs = None
        self.home_costs_t = -math.inf
        self.seen_following = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos_map)
        self.create_subscription(String, '/nav/controller_state', self.on_controller, 10)
        try:
            from sensor_msgs.msg import BatteryState
            self.create_subscription(BatteryState, '/battery', self.on_battery, 1)
        except ImportError:   # pragma: no cover
            pass
        self.path_pub = self.create_publisher(Path, '/nav/path', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/nav/goal', 10)
        self.front_pub = self.create_publisher(MarkerArray, '/nav/frontiers', 1)
        self.cov_pub = self.create_publisher(OccupancyGrid, '/nav/coverage', latched)
        self.state_pub = self.create_publisher(String, '/mission/state', latched)
        self.create_service(TriggerSrv, '/mission/start', self.srv_start)
        self.create_service(TriggerSrv, '/mission/abort', self.srv_abort)
        self.create_service(TriggerSrv, '/mission/return_home', self.srv_return)
        self.create_timer(TICK, self.tick)
        self.create_timer(1.0, self.publish_slow)
        self.publish_state()
        self.get_logger().info(f'ready (use_lidar={self.use_lidar}); call /mission/start')

    def now(self) -> float:
        """ROS time in seconds (sim time in simulation: mission timing follows the sim)."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def on_map(self, msg: OccupancyGrid) -> None:
        """Keep the latest map; re-grid the coverage to it."""
        self.map_msg = msg
        info = self.grid_info(msg)
        if self.coverage is None:
            self.coverage = Coverage(info)
        else:
            self.coverage.match(info)
        self.map_known = int(np.count_nonzero(np.asarray(msg.data, dtype=np.int8) >= 0))

    def on_controller(self, msg: String) -> None:
        """Track the controller's state (following / reached / ...)."""
        self.controller_state = msg.data
        if msg.data == 'following':
            self.seen_following = True

    def on_battery(self, msg) -> None:
        """Battery input reserved for later (PLAN.md); only logged."""
        self.get_logger().info(f'battery {msg.percentage:.0%}', throttle_duration_sec=60.0)

    @staticmethod
    def grid_info(msg: OccupancyGrid) -> GridInfo:
        """GridInfo of an OccupancyGrid message."""
        return GridInfo(msg.info.resolution, msg.info.origin.position.x,
                        msg.info.origin.position.y, msg.info.height, msg.info.width)

    def read_pose(self) -> None:
        """Latest map -> base_link pose."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return
        t = tf.transform.translation
        stamp = Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
        if self.now() - stamp <= self.loc_timeout:
            self.pose = (t.x, t.y, t.z, yaw_of(tf.transform.rotation))
            self.pose_time = self.now()

    # ------------------------------------------------------------------ services
    def srv_start(self, _req, resp):
        """IDLE -> TAKEOFF."""
        if self.mission.state != State.IDLE or self.pose is None:
            resp.success, resp.message = False, (
                f'cannot start in {self.mission.state.value}' if self.pose else 'no pose yet')
            return resp
        self.home = (self.pose[0], self.pose[1])
        self.t_start = self.now()
        self.change(Trigger.START)
        resp.success, resp.message = True, 'mission started'
        return resp

    def srv_abort(self, _req, resp):
        """Any state -> ABORT."""
        self.change(Trigger.ABORT)
        resp.success, resp.message = True, f'state {self.mission.state.value}'
        return resp

    def srv_return(self, _req, resp):
        """Emergency recall -> RETURN_HOME."""
        old = self.mission.state
        new = self.change(Trigger.RECALL)
        resp.success = new == State.RETURN_HOME
        resp.message = f'{old.value} -> {new.value}'
        return resp

    def change(self, trigger: Trigger) -> State:
        """Apply a trigger; reset per-state bookkeeping on a change."""
        old = self.mission.state
        new = self.mission.handle(trigger)
        if new != old:
            self.get_logger().info(f'mission: {old.value} --{trigger.value}--> {new.value}')
            self.state_since = self.now()
            self.path_points, self.seen_following = [], False
            self.next_replan = 0.0
            self.turning_to = None
            if new == State.ABORT:
                self.publish_path([])          # empty path: the controller holds position
            self.publish_state()
        return new

    # ------------------------------------------------------------------ outputs
    def publish_state(self) -> None:
        """Publish /mission/state."""
        self.state_pub.publish(String(data=self.mission.state.value))

    def publish_path(self, points: list[tuple[float, float, float]],
                     yaw: float | None = None) -> None:
        """Publish /nav/path; the last pose carries `yaw` (zero quaternion = no goal yaw)."""
        self.path_points, self.path_yaw = points, yaw
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y, z) in enumerate(points):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
            if i == len(points) - 1:
                if yaw is not None:
                    ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                else:
                    ps.pose.orientation.w = 0.0
            else:
                nx, ny, _ = points[i + 1]
                h = math.atan2(ny - y, nx - x)
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(h / 2), math.cos(h / 2)
            msg.poses.append(ps)
        self.path_pub.publish(msg)
        self.last_publish = self.now()
        self.seen_following = False

    def republish(self) -> None:
        """Keep the current path fresh for the controller's path_timeout."""
        if self.path_points and self.now() - self.last_publish > 0.5:
            seen = self.seen_following
            self.publish_path(self.path_points, self.path_yaw)
            self.seen_following = seen

    def publish_slow(self) -> None:
        """1 Hz: state, coverage grid."""
        self.publish_state()
        if self.coverage is None or self.map_msg is None:
            return
        m = self.map_msg
        grid = grid_from_occupancy(m.data, m.info.width, m.info.height)
        if grid.shape != self.coverage.seen.shape:
            return
        free = (grid >= 0) & (grid < 50)
        out = OccupancyGrid()
        out.header = m.header
        out.info = m.info
        vals = np.full(grid.shape, -1, np.int8)
        vals[free] = 100
        vals[free & self.coverage.seen] = 0
        out.data = vals.ravel().tolist()
        self.cov_pub.publish(out)

    def publish_frontiers(self, decision) -> None:
        """MarkerArray: map frontiers (orange) and view frontiers (cyan) as cube lists."""
        info = self.grid_info(self.map_msg)
        arr = MarkerArray()
        for k, (clusters, rgb) in enumerate(((decision.map_clusters, (1.0, 0.5, 0.0)),
                                              (decision.view_clusters, (0.0, 0.8, 1.0)))):
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns, mk.id, mk.type, mk.action = 'frontiers', k, Marker.CUBE_LIST, Marker.ADD
            mk.scale.x = mk.scale.y = info.resolution
            mk.scale.z = 0.02
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = *rgb, 0.8
            mk.pose.orientation.w = 1.0
            from geometry_msgs.msg import Point
            for cl in clusters:
                for r, c in cl.cells[::2]:
                    x, y = cell_to_world(info, int(r), int(c))
                    mk.points.append(Point(x=x, y=y, z=0.05 + 0.02 * k))
            arr.markers.append(mk)
        self.front_pub.publish(arr)

    # ------------------------------------------------------------------ main loop
    def tick(self) -> None:
        """10 Hz: safety checks, coverage, state actions."""
        self.read_pose()
        st = self.mission.state
        airborne = st in (State.TAKEOFF, State.EXPLORE, State.RETURN_HOME, State.LAND)
        if airborne and self.now() - self.pose_time > self.loc_timeout:
            self.get_logger().error('localization lost')
            self.change(Trigger.LOCALIZATION_LOST)
            return
        if self.pose is None:
            return
        x, y, z, yaw = self.pose
        if airborne and not (self.bounds[0] <= x <= self.bounds[1]
                             and self.bounds[2] <= y <= self.bounds[3]):
            self.get_logger().error(f'geofence breach at ({x:.2f}, {y:.2f})')
            self.change(Trigger.GEOFENCE_BREACH)
            return
        if self.coverage is not None and self.map_msg is not None and z > 0.5:
            m = self.map_msg
            grid = grid_from_occupancy(m.data, m.info.width, m.info.height)
            if grid.shape == self.coverage.seen.shape:
                self.coverage.update(grid, x, y, yaw, self.p.camera_hfov, self.p.coverage_range)

        reached = self.seen_following and self.controller_state == 'reached'
        if st == State.TAKEOFF:
            if not self.path_points:
                self.publish_path([(x, y, z), (x, y, self.flight_altitude)], yaw)
            elif reached and z >= self.flight_altitude - self.tol:
                self.look_step = 0
                self.change(Trigger.ALTITUDE_REACHED)
        elif st == State.EXPLORE:
            self.explore_step(x, y, z, yaw, reached)
        elif st == State.RETURN_HOME:
            self.return_step(x, y, z, yaw, reached)
        elif st == State.LAND:
            if not self.path_points:
                self.publish_path([(x, y, z), (x, y, 0.0)], yaw)
            elif (reached or z < 0.15) and self.now() - self.state_since > 2.0:
                self.publish_path([])
                self.change(Trigger.LANDED)
        elif st == State.ABORT:
            if not self.path_points and self.now() - self.state_since > ABORT_HOLD and z > 0.15:
                self.publish_path([(x, y, z), (x, y, 0.0)], yaw)
            elif self.path_points and (reached or z < 0.15):
                self.publish_path([])
        self.republish()

    def explore_step(self, x, y, z, yaw, reached) -> None:
        """Look-around, then viewpoint replans."""
        if self.look_step < LOOK_STEPS:
            if not self.path_points or reached:
                if reached:
                    self.look_step += 1
                if self.look_step < LOOK_STEPS:
                    target = yaw + math.pi / 2
                    self.publish_path([(x, y, self.flight_altitude)] * 2, target)
            return
        if self.map_msg is None or self.coverage is None:
            return
        if self.turning_to is not None:          # no-LiDAR mode: turn on arrival
            if reached:
                self.turning_to = None
                self.explorer.goal_reached()
                self.next_replan = 0.0
            return
        if reached and self.path_points:
            if not self.use_lidar and self.explorer.goal is not None:
                self.turning_to = self.explorer.goal.yaw
                self.publish_path([(x, y, self.flight_altitude)] * 2, self.turning_to)
                return
            self.explorer.goal_reached()
            self.next_replan = 0.0
        changed = abs(self.map_known - self.map_known_at_plan) > self.large_change * max(1, self.map_known_at_plan)
        if self.now() < self.next_replan and not changed:
            return
        self.next_replan = self.now() + self.replan_period
        self.map_known_at_plan = self.map_known
        m = self.map_msg
        grid = grid_from_occupancy(m.data, m.info.width, m.info.height)
        info = self.grid_info(m)
        grid = self.outside_blocked(grid, info)
        t0 = time.perf_counter()
        dec = self.explorer.replan(grid, info, (x, y, yaw), self.now(), self.coverage.seen)
        dt = time.perf_counter() - t0
        self.publish_frontiers(dec)
        if dec.done:
            self.get_logger().info('exploration complete: no viewpoint left')
            self.change(Trigger.EXPLORATION_DONE)
            return
        if dec.reason:
            self.get_logger().info(f'replan: {dec.reason}')
        if self.budget_low(grid, info, x, y):
            self.get_logger().warn('return budget reached')
            self.change(Trigger.BUDGET_LOW)
            return
        if dec.goal is None or dec.plan is None or not dec.plan.success:
            return
        g = dec.goal
        pts = [(px, py, self.flight_altitude) for px, py in dec.plan.path]
        if math.hypot(pts[0][0] - x, pts[0][1] - y) > 0.05:
            pts.insert(0, (x, y, self.flight_altitude))
        self.publish_path(pts, g.yaw if self.use_lidar else None)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = g.x, g.y, self.flight_altitude
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(g.yaw / 2), math.cos(g.yaw / 2)
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'goal ({g.x:.2f}, {g.y:.2f}) gain {g.gain:.2f} m2 cost {g.cost:.2f} m, '
            f'path {dec.plan.length:.1f} m, replan {1e3 * dt:.0f} ms',
            throttle_duration_sec=5.0)

    def budget_low(self, grid, info, x, y) -> bool:
        """Return-budget rule with a Dijkstra search from home (refreshed every 10 s)."""
        r, c = world_to_cell(info, x, y)
        stale = self.home_costs is None or self.home_costs.shape != grid.shape
        if stale or self.now() - self.home_costs_t > 10.0 or not np.isfinite(self.home_costs[r, c]):
            g = clear_unknown_around(grid, info, self.home, self.p.clear_radius)
            pg = build_planning_grid(g, info, self.p.guarded_radius, self.p.safety_margin,
                                     self.p.wall_cost_range)
            self.home_costs = dijkstra_costs(pg, self.home, self.p.wall_cost_weight)
            self.home_costs_t = self.now()
        hc = self.home_costs[r, c]
        if not np.isfinite(hc):
            hc = 2.0 * math.hypot(x - self.home[0], y - self.home[1])
        return should_return(self.now() - self.t_start, self.time_limit, hc, self.max_speed,
                             self.safety_factor, self.landing_time, self.reserve)

    def return_step(self, x, y, z, yaw, reached) -> None:
        """Fly home; AT_HOME when there and the controller reports reached."""
        if math.hypot(x - self.home[0], y - self.home[1]) <= self.tol and reached:
            self.change(Trigger.AT_HOME)
            return
        if self.now() < self.next_replan or self.map_msg is None:
            return
        self.next_replan = self.now() + self.replan_period
        m = self.map_msg
        info = self.grid_info(m)
        grid = self.outside_blocked(grid_from_occupancy(m.data, m.info.width, m.info.height), info)
        grid = clear_unknown_around(grid, info, (x, y), self.p.clear_radius)
        grid = clear_unknown_around(grid, info, self.home, self.p.clear_radius)
        pg = build_planning_grid(grid, info, self.p.guarded_radius, self.p.safety_margin,
                                 self.p.wall_cost_range)
        res = plan(pg, (x, y), self.home, wall_cost_weight=self.p.wall_cost_weight)
        if not res.success:
            self.get_logger().warn(f'no path home: {res.reason}', throttle_duration_sec=5.0)
            return
        pts = [(px, py, self.flight_altitude) for px, py in res.path]
        pts.insert(0, (x, y, self.flight_altitude))
        pts.append((self.home[0], self.home[1], self.flight_altitude))
        self.publish_path(pts, None)

    def outside_blocked(self, grid: np.ndarray, info: GridInfo) -> np.ndarray:
        """Cells outside arena_bounds count as occupied (never explore outside the arena)."""
        rows, cols = np.ogrid[0:info.rows, 0:info.cols]
        cx = info.origin_x + (cols + 0.5) * info.resolution
        cy = info.origin_y + (rows + 0.5) * info.resolution
        xa, xb, ya, yb = self.bounds
        out = (cx < xa) | (cx > xb) | (cy < ya) | (cy > yb)
        g = grid.copy()
        g[out] = 100
        return g


def main(args: list[str] | None = None) -> None:
    """Run the explorer node until shutdown.

    Args:
        args: Command-line arguments forwarded to rclpy.
    """
    rclpy.init(args=args)
    node = ExplorerNode()
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
