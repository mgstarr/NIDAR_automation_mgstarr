#!/usr/bin/env python3
"""Plot a saved map with its inflation, wall cost and an A* path (debugging tool).

Usage:
    python3 tools/plot_plan.py --map src/airmouse_nav/test/data/maps/seed1_lidar_plain.yaml \
        [--start X Y] [--goal X Y] [--out plan.png]

Without --start/--goal it plans between the two far corners of the largest free region.
Writes a PNG with three panels: map + inflation, wall cost, and the path.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src' / 'airmouse_nav'))
sys.path.insert(0, str(REPO / 'tools'))

from airmouse_nav.core.astar import plan  # noqa: E402
from airmouse_nav.core.grid_utils import GridInfo, build_planning_grid, cell_to_world  # noqa: E402
from map_io import load  # noqa: E402


def planner_params() -> dict:
    """Read the planner parameters from airmouse_bringup/config/planner.yaml."""
    cfg = REPO / 'src' / 'airmouse_bringup' / 'config' / 'planner.yaml'
    return yaml.safe_load(cfg.read_text())['/**']['ros__parameters']


def main() -> int:
    """Plan once on a saved map and write the plot."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--map', required=True, help='map YAML (map_server format)')
    ap.add_argument('--start', type=float, nargs=2, metavar=('X', 'Y'))
    ap.add_argument('--goal', type=float, nargs=2, metavar=('X', 'Y'))
    ap.add_argument('--out', default='plan.png', help='output PNG (default plan.png)')
    args = ap.parse_args()

    p = planner_params()
    m = load(args.map)
    info = GridInfo(m.resolution, m.origin[0], m.origin[1], *m.grid.shape)
    pg = build_planning_grid(m.grid, info, p['guarded_radius'], p['safety_margin'],
                             p['wall_cost_range'])
    if args.start is None or args.goal is None:
        sizes = np.bincount(pg.components.ravel())
        sizes[0] = 0
        cells = np.argwhere(pg.components == sizes.argmax())
        start = args.start or cell_to_world(info, *cells[cells.sum(1).argmin()])
        goal = args.goal or cell_to_world(info, *cells[cells.sum(1).argmax()])
    else:
        start, goal = tuple(args.start), tuple(args.goal)
    t0 = time.perf_counter()
    res = plan(pg, start, goal, wall_cost_weight=p['wall_cost_weight'])
    ms = (time.perf_counter() - t0) * 1e3

    extent = (info.origin_x, info.origin_x + info.cols * info.resolution,
              info.origin_y, info.origin_y + info.rows * info.resolution)
    base = np.select([m.grid < 0, m.grid >= 50], [0.6, 0.0], 1.0)
    fig, axes = plt.subplots(1, 3, figsize=(18, 6.5))
    titles = ['map (black wall, grey unknown) + inflation (red)', 'wall cost on free cells (grey = blocked)',
              f'path: {res.reason}, {res.length:.2f} m, {len(res.path)} points, {ms:.0f} ms']
    for ax, title in zip(axes, titles):
        ax.set_title(title, fontsize=10)
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
    axes[0].imshow(base, origin='lower', extent=extent, cmap='gray', vmin=0, vmax=1)
    inflation = np.ma.masked_where(~(pg.blocked & (base > 0.9)), np.ones_like(base))
    axes[0].imshow(inflation, origin='lower', extent=extent, cmap='autumn', alpha=0.5)
    cmap = plt.get_cmap('viridis').copy()
    cmap.set_bad('0.25')   # blocked cells in dark grey
    im = axes[1].imshow(np.ma.masked_where(pg.blocked, pg.wall_cost), origin='lower',
                        extent=extent, cmap=cmap, vmin=0, vmax=1)
    fig.colorbar(im, ax=axes[1], fraction=0.046)
    axes[2].imshow(np.where(pg.blocked, 0.3, 1.0) * np.where(m.grid >= 50, 0, 1), origin='lower',
                   extent=extent, cmap='gray', vmin=0, vmax=1)
    if res.path:
        xs, ys = zip(*res.path)
        axes[2].plot(xs, ys, '-o', color='tab:blue', ms=2.5, lw=1.5)
    axes[2].plot(*start, 'g^', ms=10, label='start')
    axes[2].plot(*goal, 'r*', ms=12, label='goal')
    axes[2].legend(loc='upper right')
    fig.suptitle(Path(args.map).stem)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f'{res.reason}: {res.length:.2f} m, {len(res.path)} points, {ms:.1f} ms -> {args.out}')
    return 0 if res.success else 1


if __name__ == '__main__':
    sys.exit(main())
