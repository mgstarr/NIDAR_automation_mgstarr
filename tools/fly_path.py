#!/usr/bin/env python3
"""Sim/field test tool: take off or land through a path (Phase 4).

Publishes a vertical /nav/path at the drone's current x, y: from the current height to
flight_altitude (takeoff) or to 0 (land). The controller flies it like any other path. The path
is republished every 0.5 s (fresh for path_timeout) until the controller reports "reached", or,
when landing, until the drone has stopped descending near the floor.

Usage (sim + localization + navigation running):
    python3 tools/fly_path.py takeoff [--altitude 1.2]
    python3 tools/fly_path.py land
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def main() -> int:
    """Run take-off or landing and return 0 on success."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('action', choices=['takeoff', 'land'])
    ap.add_argument('--altitude', type=float, default=1.2, help='take-off height (m)')
    ap.add_argument('--timeout', type=float, default=40.0, help='give up after this (s, wall)')
    args = ap.parse_args()

    rclpy.init()
    node = Node('fly_path', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buf = Buffer()
    TransformListener(buf, node)
    pub = node.create_publisher(Path, '/nav/path', 10)
    state = {'s': '', 'seen_following': False}

    def on_state(msg: String) -> None:
        state['s'] = msg.data
        if msg.data == 'following':
            state['seen_following'] = True

    node.create_subscription(String, '/nav/controller_state', on_state, 10)
    target = args.altitude if args.action == 'takeoff' else 0.0
    start, ok, low_since = None, False, None
    t_end = time.time() + args.timeout
    last_pub, last_z = 0.0, None
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            tf = buf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            continue
        p, q = tf.transform.translation, tf.transform.rotation
        if start is None:
            start = (p.x, p.y, p.z, q)
            node.get_logger().info(f'{args.action}: z {p.z:.2f} -> {target:.2f} m '
                                   f'at ({p.x:.2f}, {p.y:.2f})')
        if state['seen_following'] and state['s'] == 'reached':
            ok = True
            break
        if args.action == 'land':
            moving = last_z is None or abs(p.z - last_z) > 0.002
            low_since = None if (moving or p.z > 0.3) else (low_since or time.time())
            if low_since and time.time() - low_since > 1.5:
                ok = True
                break
            last_z = p.z
        if time.time() - last_pub > 0.5:
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = node.get_clock().now().to_msg()
            for z in (start[2], target):
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = start[0], start[1], z
                ps.pose.orientation = start[3]
                path.poses.append(ps)
            pub.publish(path)
            last_pub = time.time()
    if ok and args.action == 'land':
        empty = Path()
        empty.header.frame_id = 'map'
        empty.header.stamp = node.get_clock().now().to_msg()
        pub.publish(empty)   # stop following: hover command = 0 on the floor
    try:
        z = buf.lookup_transform('map', 'base_link', Time()).transform.translation.z
    except TransformException:
        z = math.nan
    node.get_logger().info(f'{args.action} {"done" if ok else "TIMED OUT"}: z = {z:.2f} m')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
