#!/usr/bin/env python3
"""Average CPU use per stack node over a time window (Phase 6: Raspberry Pi budget).

    python3 tools/cpu_usage.py [--duration 60]
Samples the processes of the stack by name and prints mean CPU (% of one core) and memory per
node. Run during a mission. Measured on the dev machine: scale for the Raspberry Pi.
"""

from __future__ import annotations

import argparse
import time

import psutil

NODES = {
    'rtabmap (SLAM)': ['rtabmap_slam/rtabmap'],
    'icp_odometry': ['icp_odometry'],
    'rgbd_odometry': ['rgbd_odometry'],
    'ekf_node': ['ekf_node'],
    'range_to_z_node': ['range_to_z_node'],
    'explorer_node': ['explorer_node'],
    'controller_node': ['controller_node'],
    'safety_hook_node': ['safety_hook_node'],
    'survivor_detector_node': ['survivor_detector_node'],
    'depth -> points (RViz only)': ['point_cloud_xyz'],
    'sim_bridge_node (sim only)': ['sim_bridge_node'],
    'gz sim (sim only)': ['gz sim'],
}


def main() -> None:
    """Sample and print."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--duration', type=float, default=60.0)
    args = ap.parse_args()
    procs = {}
    for p in psutil.process_iter(['cmdline']):
        cmd = ' '.join(p.info['cmdline'] or [])
        for label, keys in NODES.items():
            if any(k in cmd for k in keys) and 'cpu_usage' not in cmd and 'ros2 launch' not in cmd:
                procs.setdefault(label, []).append(p)
    for ps in procs.values():
        for p in ps:
            p.cpu_percent(None)
    time.sleep(args.duration)
    print(f'{"node":32s} {"CPU % of one core":>18s} {"RSS MB":>8s}')
    total = 0.0
    for label in NODES:
        if label not in procs:
            continue
        cpu = mem = 0.0
        for p in procs[label]:
            try:
                cpu += p.cpu_percent(None)
                mem += p.memory_info().rss / 1e6
            except psutil.Error:
                pass
        if 'sim only' not in label and 'RViz' not in label:
            total += cpu
        print(f'{label:32s} {cpu:18.1f} {mem:8.0f}')
    print(f'{"TOTAL onboard stack":32s} {total:18.1f}')


if __name__ == '__main__':
    main()
