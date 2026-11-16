#!/usr/bin/env python3
"""Estimated pose vs ground truth over a run: absolute trajectory error (Phase 6).

The estimate is the stack's own pose, TF map -> base_link (RTAB-Map + EKF). Ground truth is
/ground_truth/odom from Gazebo, recorded for scoring only. Both use the same frame (origin at the
take-off point, +x into the maze), so the raw error needs no alignment; the error after a
best-fit rigid alignment (rotation + translation) is reported too: it removes a constant offset
and shows the drift in shape alone.

Usage (stack running, any mode):
    python3 tools/pose_error.py [--duration 600] [--csv /tmp/pose.csv] [--min-altitude 0.3]
Ctrl+C (or --duration) prints:
    POSE ERROR rmse 0.043 m, mean 0.036, max 0.121, final 0.050 (aligned rmse 0.031) over N samples
Library use: ate(est, gt) with rows (t, x, y) -> dict.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np


def ate(est: np.ndarray, gt: np.ndarray) -> dict:
    """Absolute trajectory error between an estimate and ground truth.

    Args:
        est: (N, 3) rows of (t, x, y): the estimated trajectory.
        gt: (M, 3) rows of (t, x, y): ground truth, any rate; interpolated at the estimate's times.

    Returns:
        dict with rmse, mean, max, final (m), aligned_rmse (after a best-fit rigid 2D
        alignment), samples. Estimate samples outside the ground-truth time span are dropped.
    """
    est, gt = np.asarray(est, float), np.asarray(gt, float)
    gt = gt[np.argsort(gt[:, 0])]
    keep = (est[:, 0] >= gt[0, 0]) & (est[:, 0] <= gt[-1, 0])
    est = est[keep]
    if len(est) == 0:
        return {'rmse': math.nan, 'mean': math.nan, 'max': math.nan, 'final': math.nan,
                'aligned_rmse': math.nan, 'samples': 0}
    gx = np.interp(est[:, 0], gt[:, 0], gt[:, 1])
    gy = np.interp(est[:, 0], gt[:, 0], gt[:, 2])
    e = np.column_stack([est[:, 1], est[:, 2]])
    g = np.column_stack([gx, gy])
    err = np.linalg.norm(e - g, axis=1)
    # Best-fit rigid alignment of the estimate onto ground truth (Kabsch, 2D, no scale).
    ec, gc = e - e.mean(0), g - g.mean(0)
    u, _, vt = np.linalg.svd(ec.T @ gc)
    d = np.sign(np.linalg.det(u @ vt)) or 1.0
    rot = (u @ np.diag([1.0, d]) @ vt).T
    aligned = (rot @ ec.T).T + g.mean(0)
    aerr = np.linalg.norm(aligned - g, axis=1)
    return {'rmse': float(np.sqrt((err ** 2).mean())), 'mean': float(err.mean()),
            'max': float(err.max()), 'final': float(err[-1]),
            'aligned_rmse': float(np.sqrt((aerr ** 2).mean())), 'samples': int(len(err))}


def main() -> int:
    """Record both trajectories from the running stack and print the error."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--duration', type=float, default=0.0, help='stop after this (s, wall); 0 = Ctrl+C')
    ap.add_argument('--csv', help='write t, est_x, est_y, gt_x, gt_y rows here')
    ap.add_argument('--min-altitude', type=float, default=0.3, help='ignore samples below this height')
    args = ap.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = Node('pose_error', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buf = Buffer()
    TransformListener(buf, node)
    est, gt = [], []

    def on_gt(msg: Odometry) -> None:
        p = msg.pose.pose.position
        if p.z >= args.min_altitude:
            gt.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, p.x, p.y))

    def sample() -> None:
        try:
            tf = buf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return
        t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if tf.transform.translation.z >= args.min_altitude and (not est or t > est[-1][0]):
            est.append((t, tf.transform.translation.x, tf.transform.translation.y))

    node.create_subscription(Odometry, '/ground_truth/odom', on_gt, 50)
    node.create_timer(0.1, sample)
    t0 = time.time()
    try:
        while rclpy.ok() and (not args.duration or time.time() - t0 < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    if len(est) < 2 or len(gt) < 2:
        print('POSE ERROR: not enough samples (is the stack running and airborne?)')
        return 1
    r = ate(np.array(est), np.array(gt))
    print(f"POSE ERROR rmse {r['rmse']:.3f} m, mean {r['mean']:.3f}, max {r['max']:.3f}, "
          f"final {r['final']:.3f} (aligned rmse {r['aligned_rmse']:.3f}) over {r['samples']} samples")
    if args.csv:
        e = np.array(est)
        g = np.array(gt)
        g = g[np.argsort(g[:, 0])]
        rows = np.column_stack([e, np.interp(e[:, 0], g[:, 0], g[:, 1]),
                                np.interp(e[:, 0], g[:, 0], g[:, 2])])
        np.savetxt(args.csv, rows, delimiter=',', header='t,est_x,est_y,gt_x,gt_y', comments='')
    return 0


if __name__ == '__main__':
    sys.exit(main())
