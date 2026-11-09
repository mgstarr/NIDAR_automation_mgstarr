"""Camera coverage: which free cells the camera has seen (no ROS imports).

Public API (explorer_node, viewpoint, grid_sim):

    cov = Coverage(info)                        # seen-grid with the same size/origin as /map
    cov.match(info)                             # follow a /map that grew or moved (keeps overlap)
    n_new = cov.update(grid, x, y, yaw, hfov, max_range)   # mark free cells in view as seen
    clusters = cov.view_frontiers(grid, min_size)          # free cells not yet seen, clustered
    rows, cols = ray_cells(grid, info, x, y, yaw, hfov, max_range, stop_at_unknown=False)

Ray casting: rays fan across `hfov` (one ray per cell width at max range), sampled every half
cell from the camera position (the drone's x, y). A ray stops at the first occupied cell (the
wall cell itself is returned too: it is "visible"), at max range, or at the grid edge. Unknown
cells do not stop a ray unless `stop_at_unknown` is set: the camera itself sees through
space the map has not filled in yet.
"""

from __future__ import annotations

import math

import numpy as np

from airmouse_nav.core.frontier import Cluster, cluster_cells, free_mask
from airmouse_nav.core.grid_utils import OCCUPIED_THRESHOLD, GridInfo


def ray_cells(grid: np.ndarray, info: GridInfo, x: float, y: float, yaw: float, hfov: float,
              max_range: float, stop_at_unknown: bool = False,
              coarse: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """Unique cells visible from (x, y, yaw): (rows, cols) arrays. See the module docstring.

    `coarse` halves the ray count and sample density (used to score many candidate poses).
    """
    res = info.resolution
    spacing = 2.0 if coarse else 1.0
    n_rays = max(3, int(math.ceil(hfov * max_range / (res * spacing))) + 1)
    angles = yaw + np.linspace(-hfov / 2, hfov / 2, n_rays)
    steps = np.arange(0.0, max_range + 1e-9, res / 2 * spacing)
    px = x + np.outer(np.cos(angles), steps)               # (rays, steps)
    py = y + np.outer(np.sin(angles), steps)
    cols = np.floor((px - info.origin_x) / res).astype(int)
    rows = np.floor((py - info.origin_y) / res).astype(int)
    inside = (rows >= 0) & (rows < info.rows) & (cols >= 0) & (cols < info.cols)
    vals = np.full(rows.shape, -1, dtype=np.int16)
    vals[inside] = grid[rows[inside], cols[inside]]
    stop = (vals >= OCCUPIED_THRESHOLD) | ~inside
    if stop_at_unknown:
        stop |= vals < 0
    # Index of the first stopping sample on each ray (len if none); keep samples up to it,
    # including an occupied stopping cell (but not an off-grid or unknown one).
    first = np.where(stop.any(axis=1), stop.argmax(axis=1), stop.shape[1])
    idx = np.arange(stop.shape[1])
    keep = idx[None, :] < first[:, None]
    hit = (idx[None, :] == first[:, None]) & inside & (vals >= OCCUPIED_THRESHOLD)
    keep |= hit
    r, c = rows[keep], cols[keep]
    if r.size == 0:
        return r, c
    flat = np.unique(r * info.cols + c)
    return flat // info.cols, flat % info.cols


class Coverage:
    """Grid of cells the camera has seen."""

    def __init__(self, info: GridInfo) -> None:
        self.info = info
        self.seen = np.zeros((info.rows, info.cols), dtype=bool)

    def match(self, info: GridInfo) -> None:
        """Re-grid to a new map size/origin (same resolution), keeping what overlaps."""
        if info == self.info:
            return
        new = np.zeros((info.rows, info.cols), dtype=bool)
        dr = int(round((self.info.origin_y - info.origin_y) / info.resolution))
        dc = int(round((self.info.origin_x - info.origin_x) / info.resolution))
        r0, c0 = max(0, dr), max(0, dc)
        r1 = min(info.rows, dr + self.info.rows)
        c1 = min(info.cols, dc + self.info.cols)
        if r1 > r0 and c1 > c0:
            new[r0:r1, c0:c1] = self.seen[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
        self.info, self.seen = info, new

    def update(self, grid: np.ndarray, x: float, y: float, yaw: float, hfov: float,
               max_range: float) -> int:
        """Mark free cells in the camera view as seen; returns how many were new."""
        rows, cols = ray_cells(grid, self.info, x, y, yaw, hfov, max_range)
        free = free_mask(grid)[rows, cols]
        rows, cols = rows[free], cols[free]
        new = int((~self.seen[rows, cols]).sum())
        self.seen[rows, cols] = True
        return new

    def unseen_free(self, grid: np.ndarray) -> np.ndarray:
        """Free cells not seen yet."""
        return free_mask(grid) & ~self.seen

    def view_frontiers(self, grid: np.ndarray, min_size: int = 6) -> list[Cluster]:
        """Clusters of free cells the camera has not seen yet."""
        return cluster_cells(self.unseen_free(grid), self.info, min_size)

    def seen_fraction(self, grid: np.ndarray) -> float:
        """Fraction of known free cells that were seen."""
        free = free_mask(grid)
        total = int(free.sum())
        return 1.0 if total == 0 else float((self.seen & free).sum()) / total
