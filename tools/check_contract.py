#!/usr/bin/env python3
"""Check that every contract topic is alive and the TF chain map -> odom -> base_link is complete.

Run with the stack up (any mode). Exit code 0 = all good.
    python3 tools/check_contract.py [--use-lidar true] [--timeout 20]
Topics checked (docs/PLAN.md > Topics): camera colour/depth images and camera_info, /camera/imu,
/scan (only with LiDAR), /range/down, /range/up, /odom (own odometry; skipped with
--ground-truth), /map, /mission/state, /nav/cmd_vel_raw, /cmd_vel.
"""

from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    """Wait for one message per topic and for both transforms."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--use-lidar', default='true')
    ap.add_argument('--ground-truth', action='store_true', help='stack runs on ground truth: no /odom')
    ap.add_argument('--timeout', type=float, default=20.0)
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer, TransformListener

    topics = {
        '/camera/color/image_raw': 'sensor_msgs/msg/Image',
        '/camera/depth/image_rect_raw': 'sensor_msgs/msg/Image',
        '/camera/color/camera_info': 'sensor_msgs/msg/CameraInfo',
        '/camera/depth/camera_info': 'sensor_msgs/msg/CameraInfo',
        '/camera/imu': 'sensor_msgs/msg/Imu',
        '/range/down': 'sensor_msgs/msg/Range',
        '/range/up': 'sensor_msgs/msg/Range',
        '/map': 'nav_msgs/msg/OccupancyGrid',
        '/mission/state': 'std_msgs/msg/String',
        '/nav/cmd_vel_raw': 'geometry_msgs/msg/TwistStamped',
        '/cmd_vel': 'geometry_msgs/msg/TwistStamped',
    }
    if args.use_lidar.lower() == 'true':
        topics['/scan'] = 'sensor_msgs/msg/LaserScan'
    if not args.ground_truth:
        topics['/odom'] = 'nav_msgs/msg/Odometry'

    rclpy.init()
    node = Node('check_contract')
    buf = Buffer()
    TransformListener(buf, node)
    seen: set[str] = set()
    for name, typ in topics.items():
        node.create_subscription(get_message(typ), name,
                                 lambda _m, n=name: seen.add(n), qos_profile_sensor_data)
    chain = {('map', 'odom'): False, ('odom', 'base_link'): False}
    t0 = time.time()
    while time.time() - t0 < args.timeout and (len(seen) < len(topics) or not all(chain.values())):
        rclpy.spin_once(node, timeout_sec=0.1)
        for a, b in chain:
            chain[(a, b)] = chain[(a, b)] or buf.can_transform(a, b, Time())
    ok = True
    for name in topics:
        alive = name in seen
        ok &= alive
        print(f'{"ok     " if alive else "MISSING"} {name}')
    for (a, b), good in chain.items():
        ok &= good
        print(f'{"ok     " if good else "MISSING"} TF {a} -> {b}')
    print('CONTRACT OK' if ok else 'CONTRACT FAILED')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
