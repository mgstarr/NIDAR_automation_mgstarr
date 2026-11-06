#!/usr/bin/env python3
"""Score a saved map against the maze generator's ground-truth layout.

Both maps are in the map frame (origin = take-off point), so they align by their origins.

Metrics:
  wall_agreement: share of observed ground-truth wall faces (wall cells next to free space the
      map has seen) that have a map wall within `tolerance` metres;
  false_wall_cells: map wall cells farther than `tolerance` from any real wall or survivor
      (e.g. inside a corridor or doorway); false_wall_fraction = false_wall_cells / map wall cells;
  known_free_fraction: share of the real free space the map has seen.

Usage:
    python3 tools/score_map.py --map src/airmouse_nav/test/data/maps/seed1_lidar_plain.yaml \\
        --layout src/airmouse_description/worlds/generated/seed_1_plain/layout.yaml
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent))
import map_io  # noqa: E402

DEFAULT_TOLERANCE = 0.10    # m; two 0.05 m cells (owner-approved, Phase 2)
SURVIVOR_RADIUS = 0.20      # m; body radius of the generator's survivor model


def align(occ: map_io.OccupancyMap, ref: map_io.OccupancyMap) -> tuple[np.ndarray, tuple[int, int]]:
    """Place `occ` on a canvas covering both maps, in `ref`'s cell grid.

    Returns:
        (canvas, (row0, col0)): occ's grid on the canvas (unknown elsewhere) and the canvas
        cell where ref's cell (0, 0) lands.
    """
    assert abs(occ.resolution - ref.resolution) < 1e-9, 'maps must share a resolution'
    res = ref.resolution
    dc = int(round((occ.origin[0] - ref.origin[0]) / res))
    dr = int(round((occ.origin[1] - ref.origin[1]) / res))
    r_min, c_min = min(0, dr), min(0, dc)
    rows = max(ref.grid.shape[0], dr + occ.grid.shape[0]) - r_min
    cols = max(ref.grid.shape[1], dc + occ.grid.shape[1]) - c_min
    canvas = np.full((rows, cols), map_io.UNKNOWN, dtype=np.int8)
    canvas[dr - r_min:dr - r_min + occ.grid.shape[0], dc - c_min:dc - c_min + occ.grid.shape[1]] = occ.grid
    return canvas, (-r_min, -c_min)


def score(occ: map_io.OccupancyMap, truth: map_io.OccupancyMap,
          tolerance: float = DEFAULT_TOLERANCE,
          survivors: list[tuple[float, float]] | None = None) -> dict:
    """Compare a map with the ground truth. See the module docstring for the metrics.

    Args:
        survivors: map-frame (x, y) of survivor models. They are real obstacles, so map walls
            on them are not counted as false walls.
    """
    m, (r0, c0) = align(occ, truth)
    gt_wall = np.zeros(m.shape, dtype=bool)
    gt_free = np.zeros(m.shape, dtype=bool)
    rows, cols = truth.grid.shape
    gt_wall[r0:r0 + rows, c0:c0 + cols] = truth.grid == map_io.OCCUPIED
    gt_free[r0:r0 + rows, c0:c0 + cols] = truth.grid == map_io.FREE

    res = truth.resolution
    obstacles = gt_wall.copy()
    if survivors:
        rr, cc = np.indices(m.shape)
        ox, oy = truth.origin[0] - c0 * res, truth.origin[1] - r0 * res
        for x, y in survivors:
            obstacles |= np.hypot(ox + (cc + 0.5) * res - x, oy + (rr + 0.5) * res - y) <= SURVIVOR_RADIUS
    map_wall, map_known = m == map_io.OCCUPIED, m != map_io.UNKNOWN
    dist_to_obstacle = ndimage.distance_transform_edt(~obstacles) * res
    dist_to_map_wall = ndimage.distance_transform_edt(~map_wall) * res if map_wall.any() \
        else np.full(m.shape, np.inf)

    eight = np.ones((3, 3), dtype=bool)
    faces = gt_wall & ndimage.binary_dilation(gt_free, structure=eight)
    observed = faces & ndimage.binary_dilation(gt_free & map_known, structure=eight)
    agreed = observed & (dist_to_map_wall <= tolerance)
    false_walls = map_wall & (dist_to_obstacle > tolerance)
    n_obs, n_wall = int(observed.sum()), int(map_wall.sum())
    return {
        'wall_agreement': float(agreed.sum() / n_obs) if n_obs else 0.0,
        'observed_wall_faces': n_obs,
        'false_wall_cells': int(false_walls.sum()),
        'false_wall_fraction': float(false_walls.sum() / n_wall) if n_wall else 0.0,
        'known_free_fraction': float((gt_free & map_known).sum() / gt_free.sum()),
        'tolerance_m': tolerance,
    }


def main(argv: list[str] | None = None) -> int:
    """Print the scores as JSON."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--map', type=Path, required=True, help='saved map YAML')
    parser.add_argument('--layout', type=Path, required=True, help='ground-truth layout.yaml')
    parser.add_argument('--tolerance', type=float, default=DEFAULT_TOLERANCE)
    args = parser.parse_args(argv)
    layout_json = args.layout.with_name('layout.json')
    survivors = ([tuple(s['position'][:2]) for s in json.loads(layout_json.read_text())['survivors']]
                 if layout_json.exists() else None)
    result = score(map_io.load(args.map), map_io.load(args.layout), args.tolerance, survivors)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
