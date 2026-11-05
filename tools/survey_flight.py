#!/usr/bin/env python3
"""Test tool (sim): fly a tour of every cell of a generated maze, using ground truth.

Used to build the Phase 2 saved maps without keyboard flying. It reads the maze's
ground-truth layout (layout.json + layout.yaml), plans a depth-first tour through all cells
(with backtracking), takes off and flies it facing the direction of travel.
This is not the navigation stack: it cheats with the true layout and the true pose.

Usage (sim + localization running):
    python3 tools/survey_flight.py --layout src/airmouse_description/worlds/generated/seed_1_plain
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import map_io  # noqa: E402

Cell = tuple[int, int]


def segment_distance(p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    """Distance from point p to segment a-b."""
    ax, ay, bx, by = *a, *b
    t = ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / max((bx - ax) ** 2 + (by - ay) ** 2, 1e-12)
    t = min(1.0, max(0.0, t))
    return math.hypot(p[0] - (ax + t * (bx - ax)), p[1] - (ay + t * (by - ay)))


def cell_graph(layout_dir: Path, survivor_clearance: float = 0.6
               ) -> tuple[dict[Cell, tuple[float, float]], dict[Cell, list[Cell]], Cell]:
    """Cell centres (map frame) and open neighbours, from the generator's layout files.

    Moves that pass within `survivor_clearance` of a survivor are left out, so the tour
    never flies into one (drone radius 0.30 m + survivor radius 0.20 m + margin).
    """
    layout = json.loads((layout_dir / 'layout.json').read_text())
    cfg = layout['config']
    survivors = [tuple(s['position'][:2]) for s in layout['survivors']]
    n, pitch = cfg['cells'], cfg['cell_clear'] + cfg['wall_thickness']
    entry = (0, n // 2)
    occ = map_io.load(layout_dir / 'layout.yaml')
    centres = {(i, j): ((i - entry[0]) * pitch, (j - entry[1]) * pitch) for i in range(n) for j in range(n)}

    def free(x: float, y: float) -> bool:
        r = int((y - occ.origin[1]) / occ.resolution)
        c = int((x - occ.origin[0]) / occ.resolution)
        return occ.grid[r, c] == map_io.FREE

    graph: dict[Cell, list[Cell]] = {cell: [] for cell in centres}
    for (i, j), (x, y) in centres.items():
        for nb in ((i + 1, j), (i, j + 1)):
            if nb in centres:
                mx, my = (x + centres[nb][0]) / 2, (y + centres[nb][1]) / 2
                clear = all(segment_distance(sv, (x, y), centres[nb]) > survivor_clearance
                            for sv in survivors)
                if free(mx, my) and clear:
                    graph[(i, j)].append(nb)
                    graph[nb].append((i, j))
    return centres, graph, entry


def tour(graph: dict[Cell, list[Cell]], start: Cell) -> list[Cell]:
    """Depth-first walk visiting every reachable cell, including the walk back (backtracking)."""
    order, visited = [start], {start}

    def visit(cell: Cell) -> None:
        for nb in sorted(graph[cell]):
            if nb not in visited:
                visited.add(nb)
                order.append(nb)
                visit(nb)
                order.append(cell)

    sys.setrecursionlimit(10000)
    visit(start)
    return order


def compress(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Drop intermediate points on straight runs, so the drone does not stop at every cell."""
    out = [points[0]]
    for k in range(1, len(points) - 1):
        a, b, c = out[-1], points[k], points[k + 1]
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        dot = (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1])
        if abs(cross) > 1e-6 or dot < 0:
            out.append(b)
    out.append(points[-1])
    return out


def wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def main(argv: list[str] | None = None) -> int:
    """Take off, fly the tour, hover at the end."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--layout', type=Path, required=True, help='generated maze folder')
    parser.add_argument('--altitude', type=float, default=1.2)
    parser.add_argument('--speed', type=float, default=0.6)
    parser.add_argument('--yaw-rate', type=float, default=0.5)
    parser.add_argument('--reach', type=float, default=0.20)
    parser.add_argument('--turn-radius', type=float, default=0.5,
                        help='stop turning when this close to the target')
    args = parser.parse_args(argv)

    centres, graph, entry = cell_graph(args.layout)
    waypoints = compress([centres[c] for c in tour(graph, entry)])

    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = rclpy.create_node('survey_flight_tool')
    pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    state = {}
    node.create_subscription(Odometry, '/ground_truth/odom', lambda m: state.update(odom=m), 10)

    def send(vx: float = 0.0, vy: float = 0.0, vz: float = 0.0, wz: float = 0.0) -> None:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.z = vx, vy, vz, wz
        pub.publish(msg)

    k, airborne = 0, False
    node.get_logger().info(f'{len(waypoints)} waypoints')
    while rclpy.ok() and k < len(waypoints):
        rclpy.spin_once(node, timeout_sec=0.05)
        if 'odom' not in state:
            continue
        p, q = state['odom'].pose.pose.position, state['odom'].pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        vz = float(np.clip(1.0 * (args.altitude - p.z), -0.5, 0.5))
        if not airborne:
            send(vz=vz)
            airborne = abs(args.altitude - p.z) < 0.05
            continue
        tx, ty = waypoints[k]
        dx, dy = tx - p.x, ty - p.y
        dist = math.hypot(dx, dy)
        if dist < args.reach:
            k += 1
            if k % 5 == 0:
                node.get_logger().info(f'waypoint {k}/{len(waypoints)}')
            continue
        # Holonomic: always move toward the target (this also cancels drift while turning);
        # turn toward the direction of travel at the same time, slower on sharp turns.
        yaw_err = wrap(math.atan2(dy, dx) - yaw)
        if dist < args.turn_radius:      # near the target the bearing swings: hold heading
            wz, speed = 0.0, min(args.speed, 1.0 * dist)
        else:
            wz = float(np.clip(1.0 * yaw_err, -args.yaw_rate, args.yaw_rate))
            speed = min(args.speed, 1.0 * dist) * (0.3 + 0.7 * max(0.0, math.cos(yaw_err)))
        # Velocity toward the target, expressed in the body frame.
        bx = speed * (math.cos(yaw) * dx + math.sin(yaw) * dy) / dist
        by = speed * (-math.sin(yaw) * dx + math.cos(yaw) * dy) / dist
        send(bx, by, vz, wz)
    send()
    node.get_logger().info('survey done')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
