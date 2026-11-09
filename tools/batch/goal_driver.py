#!/usr/bin/env python3
"""Phase 4 automated goal test: take off, fly N reachable goals picked on the live map, land.

Each goal: a free cell 1.5-5 m (path length) from the drone, picked at random (seeded) from the
current /map, preferring places not yet visited. It is published on /goal_pose (map frame) and
counts as reached when the controller reports "reached" there. Tracking error = distance from
the drone to the current /nav/path polyline while following. Prints one line per goal and a
summary. Run with sim + localization + navigation up (see goal_run.sh).
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src' / 'airmouse_nav'))
from airmouse_nav.core.astar import plan  # noqa: E402
from airmouse_nav.core.grid_utils import (GridInfo, build_planning_grid, cell_to_world,  # noqa: E402
                                          grid_from_occupancy)


def seg_dist(px, py, a, b) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / L2))
    return math.hypot(px - a[0] - t * dx, py - a[1] - t * dy)


class Driver(Node):
    def __init__(self) -> None:
        super().__init__('goal_driver', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.map: OccupancyGrid | None = None
        self.path: list[tuple[float, float]] = []
        self.state = ''
        self.path_count = 0
        self.gt = None
        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(
            self, 'gt', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.trace = open(f'/tmp/goal_trace.csv', 'w')
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), 1)
        self.create_subscription(PathMsg, '/nav/path', self.on_path, 10)
        self.create_subscription(String, '/nav/controller_state',
                                 lambda m: setattr(self, 'state', m.data), 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.path_pub = self.create_publisher(PathMsg, '/nav/path', 10)

    def on_path(self, msg: PathMsg) -> None:
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_count += 1
        if self.trace is not None and self.path:
            self.trace.write('PATH,' + ';'.join(f'{x:.3f} {y:.3f}' for x, y in self.path) + '\n')

    def pose(self):
        try:
            t = self.buf.lookup_transform('map', 'base_link', Time()).transform.translation
            return t.x, t.y, t.z
        except TransformException:
            return None

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def pick_goal(self, rng, visited):
        m, pose = self.map, self.pose()
        if m is None or pose is None:
            return None
        grid = grid_from_occupancy(m.data, m.info.width, m.info.height)
        info = GridInfo(m.info.resolution, m.info.origin.position.x, m.info.origin.position.y,
                        m.info.height, m.info.width)
        from airmouse_nav.nodes.goal_planner_node import clear_unknown_around
        grid = clear_unknown_around(grid, info, pose[:2], 0.5)
        pg = build_planning_grid(grid, info)
        free = np.argwhere(~pg.blocked)
        xy = np.column_stack([info.origin_x + (free[:, 1] + 0.5) * info.resolution,
                              info.origin_y + (free[:, 0] + 0.5) * info.resolution])
        d = np.hypot(xy[:, 0] - pose[0], xy[:, 1] - pose[1])
        cand = xy[(d > 0.5) & (d < 5.0)]
        rng.shuffle(cand)
        best, best_score, reasons = None, -1.0, {}
        for g in cand[:80]:
            g = (float(g[0]), float(g[1]))
            res = plan(pg, pose[:2], g)
            if not res.success or not 0.5 <= res.length <= 6.0:
                key = res.reason if not res.success else 'length'
                reasons[key] = reasons.get(key, 0) + 1
                continue
            novelty = min([math.hypot(g[0] - v[0], g[1] - v[1]) for v in visited] or [10.0])
            score = novelty + 0.2 * res.length
            if score > best_score:
                best, best_score = g, score
        if best is None:
            print(f'  pick: {len(free)} free cells, {len(cand)} at 0.8-5 m, rejected {reasons}',
                  flush=True)
        return best

    def turn_to(self, yaw: float, timeout: float = 20.0) -> bool:
        """Rotate in place to `yaw` with a one-point path (republished: path_timeout)."""
        pose = self.pose()
        if pose is None:
            return False
        path = PathMsg()
        path.header.frame_id = 'map'
        for _ in range(2):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pose[0], pose[1], 1.2
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(ps)
        self.state = ''
        t0, last, seen = time.time(), 0.0, False
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            seen |= self.state == 'following'
            if seen and self.state == 'reached':
                return True
            if time.time() - last > 0.5:
                path.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(path)
                last = time.time()
        return False

    def look_around(self) -> None:
        """Turn a full circle in 90-degree steps so the map fills in around the drone."""
        try:
            q = self.buf.lookup_transform('map', 'base_link', Time()).transform.rotation
            yaw0 = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        except TransformException:
            return
        for k in range(1, 5):
            self.turn_to(yaw0 + k * math.pi / 2)

    def fly_goal(self, goal, timeout: float):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = goal
        msg.pose.orientation.w = 1.0
        self.state = ''
        self.goal_pub.publish(msg)
        t0, errs, seen_following = time.time(), [], False
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            pose = self.pose()
            if self.state == 'following':
                seen_following = True
                if pose and len(self.path) >= 2:
                    errs.append(min(seg_dist(pose[0], pose[1], a, b)
                                    for a, b in zip(self.path, self.path[1:])))
                    if self.trace is not None:
                        gt = self.gt or (math.nan, math.nan)
                        self.trace.write(f'{time.time():.3f},{pose[0]:.3f},{pose[1]:.3f},'
                                         f'{errs[-1]:.3f},{self.path_count},{gt[0]:.3f},{gt[1]:.3f}\n')
            if (seen_following and self.state == 'reached' and pose
                    and math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= 0.2):
                return True, time.time() - t0, errs
        return False, time.time() - t0, errs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--goals', type=int, default=7)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()
    rclpy.init()
    node = Driver()
    node.spin_for(3.0)
    tools = REPO / 'tools'
    r = subprocess.run([sys.executable, str(tools / 'fly_path.py'), 'takeoff'], capture_output=True, text=True)
    node.spin_for(1.0)
    pose = node.pose()
    print(f'TAKEOFF {"ok" if r.returncode == 0 else "FAILED"} z={pose[2] if pose else float("nan"):.2f}', flush=True)
    node.spin_for(2.0)
    node.look_around()
    print('LOOK-AROUND done', flush=True)
    node.spin_for(2.0)
    rng = np.random.default_rng(args.seed)
    visited, ok_count, all_errs = [node.pose()[:2]], 0, []
    for i in range(args.goals):
        goal = None
        for _ in range(10):
            node.spin_for(1.0)
            goal = node.pick_goal(rng, visited)
            if goal:
                break
        if goal is None:
            print(f'GOAL {i + 1}: no reachable goal found', flush=True)
            continue
        ok, dt, errs = node.fly_goal(goal, args.timeout)
        ok_count += ok
        all_errs += errs
        visited.append(goal)
        e = (max(errs), float(np.mean(errs))) if errs else (math.nan, math.nan)
        print(f'GOAL {i + 1}: ({goal[0]:.2f}, {goal[1]:.2f}) {"reached" if ok else "FAILED"} '
              f'in {dt:.1f} s, track err max {e[0]:.3f} mean {e[1]:.3f} m', flush=True)
        node.spin_for(1.0)
    r = subprocess.run([sys.executable, str(tools / 'fly_path.py'), 'land'], capture_output=True, text=True)
    node.spin_for(2.0)
    pose = node.pose()
    print(f'LAND {"ok" if r.returncode == 0 else "FAILED"} z={pose[2] if pose else float("nan"):.2f}', flush=True)
    if all_errs:
        a = np.array(all_errs)
        print(f'SUMMARY {ok_count}/{args.goals} reached; track err max {a.max():.3f} '
              f'p95 {np.percentile(a, 95):.3f} mean {a.mean():.3f} m', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
