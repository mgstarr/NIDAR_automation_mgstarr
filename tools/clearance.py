#!/usr/bin/env python3
"""Minimum wall clearance during a run, from the ground-truth pose and the maze layout (Phase 4).

Clearance = horizontal distance from the drone's centre to the nearest wall surface (wall cells
of the ground-truth layout.pgm, as exact squares) or survivor (disc, radius 0.20 m, from
layout.json). Wall contact if clearance < guarded_radius (0.30 m).

Usage (sim running with use_ground_truth:=true):
    python3 tools/clearance.py --layout src/airmouse_description/worlds/generated/seed_3_plain \
        [--duration 120] [--min-altitude 0.5]
Prints the running minimum every 10 s and a summary at the end (Ctrl+C or --duration).
Library use: Clearance(layout_dir).at(x, y) -> metres.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
from map_io import load  # noqa: E402

SURVIVOR_RADIUS = 0.20
GUARDED_RADIUS = 0.30


class Clearance:
    """Exact distance to the nearest wall cell square or survivor disc."""

    def __init__(self, layout_dir: str | Path) -> None:
        layout_dir = Path(layout_dir)
        m = load(layout_dir / 'layout.yaml')
        self.res = m.resolution
        rows, cols = np.nonzero(m.grid == 100)
        self.centres = np.column_stack([m.origin[0] + (cols + 0.5) * m.resolution,
                                        m.origin[1] + (rows + 0.5) * m.resolution])
        self.tree = cKDTree(self.centres)
        info = json.loads((layout_dir / 'layout.json').read_text())
        self.survivors = [tuple(s['position'][:2]) for s in info.get('survivors', [])]

    def at(self, x: float, y: float) -> float:
        """Clearance (m) at map-frame point (x, y)."""
        _, idx = self.tree.query((x, y), k=min(16, len(self.centres)))
        h = self.res / 2
        c = self.centres[np.atleast_1d(idx)]
        dx = np.maximum(np.abs(x - c[:, 0]) - h, 0.0)
        dy = np.maximum(np.abs(y - c[:, 1]) - h, 0.0)
        best = float(np.hypot(dx, dy).min())
        for sx, sy in self.survivors:
            best = min(best, math.hypot(x - sx, y - sy) - SURVIVOR_RADIUS)
        return best


def main() -> int:
    """Watch /ground_truth/odom and report the minimum clearance."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--layout', required=True, help='generated maze folder (layout.yaml/.json)')
    ap.add_argument('--duration', type=float, default=0.0, help='stop after this (s, wall); 0 = Ctrl+C')
    ap.add_argument('--min-altitude', type=float, default=0.5,
                    help='ignore samples below this height (on the floor)')
    args = ap.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    cl = Clearance(args.layout)
    worst = {'c': math.inf, 'where': None, 'n': 0}

    def on_odom(msg: Odometry) -> None:
        p = msg.pose.pose.position
        if p.z < args.min_altitude:
            return
        c = cl.at(p.x, p.y)
        worst['n'] += 1
        if c < worst['c']:
            worst['c'], worst['where'] = c, (round(p.x, 2), round(p.y, 2))

    rclpy.init()
    node = Node('clearance_monitor')
    node.create_subscription(Odometry, '/ground_truth/odom', on_odom, 50)
    t0 = last = time.time()
    try:
        while rclpy.ok() and (not args.duration or time.time() - t0 < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - last > 10:
                print(f'min clearance so far {worst["c"]:.3f} m at {worst["where"]}', flush=True)
                last = time.time()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    verdict = 'OK' if worst['c'] > GUARDED_RADIUS else 'WALL CONTACT'
    print(f'MIN CLEARANCE {worst["c"]:.3f} m at {worst["where"]} over {worst["n"]} samples: {verdict}')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
