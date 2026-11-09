#!/usr/bin/env python3
"""Score a recorded Gazebo mission against ground truth (Phase 5).

Replays the ground-truth poses from a mission bag (/ground_truth/odom, /mission/state) through
the same camera model as grid_sim (camera_hfov, coverage_range, rays stop at walls and survivors)
on the true maze, and reports: mission time (take-off to DONE, sim time), path length,
survivors that fell inside camera coverage, coverage of the arena's free cells, per-room
coverage of the seeable cells (grid_sim.World.seeable), and the final distance from home.

Usage:
    python3 tools/mission_report.py --bag /tmp/mission_bag_1_true --seed 1 [--texture plain]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import grid_sim as G  # noqa: E402
from airmouse_nav.core.coverage import ray_cells  # noqa: E402
from airmouse_nav.core.viewpoint import ExplorerParams  # noqa: E402


def read_bag(path: str):
    """Ground-truth poses (t, x, y, z, yaw) and state changes (t, state) from a mission bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    poses, states = [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic not in ('/ground_truth/odom', '/mission/state'):
            continue
        m = deserialize_message(data, get_message(types[topic]))
        if topic == '/ground_truth/odom':
            p, q = m.pose.pose.position, m.pose.pose.orientation
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            poses.append((stamp, p.x, p.y, p.z, yaw))
        else:
            states.append((t * 1e-9, m.data))
    poses.sort()
    return poses, states


def main() -> int:
    """Print the mission report."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--bag', required=True)
    ap.add_argument('--seed', type=int, required=True)
    args = ap.parse_args()
    p = ExplorerParams()
    w = G.World(args.seed)
    poses, states = read_bag(args.bag)
    flying = [q for q in poses if q[3] > 0.5]
    seen = np.zeros(w.occ.shape, bool)
    survivors: set[int] = set()
    last_t = -1.0
    for t, x, y, z, yaw in flying:
        if t - last_t < 0.2:          # 5 Hz is plenty at <= 0.6 m/s and 0.8 rad/s
            continue
        last_t = t
        rows, cols = ray_cells(w.truth, w.info, x, y, yaw, p.camera_hfov, p.coverage_range)
        d = np.hypot(w.info.origin_x + (cols + 0.5) * w.info.resolution - x,
                     w.info.origin_y + (rows + 0.5) * w.info.resolution - y)
        ok = d >= G.CAMERA_MIN_RANGE
        rows, cols = rows[ok], cols[ok]
        ids = w.survivor_id[rows, cols]
        survivors |= {int(i) for i in ids[ids >= 0]}
        free = ~w.occ[rows, cols]
        seen[rows[free], cols[free]] = True
    length = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(poses, poses[1:])
                 if b[3] > 0.3)
    t_takeoff = next((q[0] for q in poses if q[3] > 0.3), poses[0][0])
    t_end = next((q[0] for q in reversed(poses) if q[3] > 0.3), poses[-1][0])
    seeable = w.seeable(p)
    rooms = []
    for room in w.rooms:
        target = room & seeable
        rooms.append(float((seen & target).sum()) / max(1, int(target.sum())))
    true_free = w.arena & ~w.occ
    final = poses[-1]
    print(f'seed {args.seed}: final state {states[-1][1] if states else "?"}; '
          f'flight time {t_end - t_takeoff:.0f} s; path {length:.1f} m; '
          f'survivors in coverage {len(survivors)}/{len(w.maze.survivors)}; '
          f'arena coverage {100 * (seen & true_free).sum() / true_free.sum():.1f}%; '
          f'rooms >= 95% of seeable {sum(f >= 0.95 for f in rooms)}/{len(rooms)} '
          f'(min {100 * min(rooms):.0f}%); final distance from home '
          f'{math.hypot(final[1], final[2]):.2f} m, z {final[3]:.2f} m')
    return 0


if __name__ == '__main__':
    sys.exit(main())
