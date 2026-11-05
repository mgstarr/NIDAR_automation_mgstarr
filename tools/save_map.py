#!/usr/bin/env python3
"""Save the live /map (nav_msgs/OccupancyGrid) as PGM + YAML for offline tests.

Usage (sim + localization running):
    python3 tools/save_map.py --name seed1_lidar_plain --seed 1 --use-lidar true --wall-texture plain
Writes src/airmouse_nav/test/data/maps/<name>.pgm and .yaml (map_server format; extra fields
under `airmouse:`).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import map_io  # noqa: E402

DEFAULT_DIR = Path(__file__).resolve().parents[1] / 'src' / 'airmouse_nav' / 'test' / 'data' / 'maps'


def grid_from_msg_data(data: list[int], width: int, height: int) -> np.ndarray:
    """OccupancyGrid data (row-major, row 0 = lowest y) -> trinary int8 grid."""
    return map_io.from_occupancy_values(np.asarray(data, dtype=np.int16).reshape(height, width))


def main(argv: list[str] | None = None) -> int:
    """Wait for one /map message and save it."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--name', required=True, help='file name without suffix')
    parser.add_argument('--out-dir', type=Path, default=DEFAULT_DIR)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--use-lidar')
    parser.add_argument('--wall-texture')
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args(argv)

    import rclpy
    from nav_msgs.msg import OccupancyGrid

    rclpy.init()
    node = rclpy.create_node('save_map_tool')
    received = []
    node.create_subscription(OccupancyGrid, '/map', received.append, 1)
    end = node.get_clock().now().nanoseconds + args.timeout * 1e9
    while rclpy.ok() and not received and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not received:
        print('no /map received', file=sys.stderr)
        return 1
    msg = received[-1]
    info = msg.info
    meta = {k: v for k, v in (('seed', args.seed), ('use_lidar', args.use_lidar),
                              ('wall_texture', args.wall_texture)) if v is not None}
    occ = map_io.OccupancyMap(grid_from_msg_data(msg.data, info.width, info.height),
                              info.resolution, (info.origin.position.x, info.origin.position.y), meta)
    print(map_io.save(args.out_dir / args.name, occ))
    return 0


if __name__ == '__main__':
    sys.exit(main())
