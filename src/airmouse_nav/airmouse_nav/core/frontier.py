"""Map frontiers and clustering (no ROS imports).

Public API (explorer_node, grid_sim):

    mask = frontier_mask(grid)                          # free cells next to unknown cells
    clusters = cluster_cells(mask, info, min_size=6)    # 8-connected clusters, small ones dropped
    clusters = map_frontiers(grid, info, min_size=6)    # both steps

    Cluster: cells (N x 2 int array of (row, col)), size (N), centroid (x, y) in metres,
             anchor (row, col): the cluster cell closest to the centroid (always on the cluster)

Grid convention as in grid_utils: int8 (rows, cols), row 0 = lowest y, -1 unknown,
values >= 50 occupied, others free. "Next to" = 4-neighbour (up, down, left, right).
Clustering is an 8-connected breadth-first search; clusters come out in a fixed order (by
their first cell in row-major order), so results are deterministic.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from airmouse_nav.core.grid_utils import OCCUPIED_THRESHOLD, GridInfo, cell_to_world


@dataclass(frozen=True)
class Cluster:
    """A connected group of target cells (map frontier or view frontier)."""

    cells: np.ndarray                 # (N, 2) int (row, col)
    size: int
    centroid: tuple[float, float]     # metres, map frame
    anchor: tuple[int, int]           # cluster cell nearest the centroid


def free_mask(grid: np.ndarray) -> np.ndarray:
    """Known free cells."""
    return (grid >= 0) & (grid < OCCUPIED_THRESHOLD)


def frontier_mask(grid: np.ndarray) -> np.ndarray:
    """Free cells with at least one unknown 4-neighbour (cells off the grid do not count)."""
    unknown = grid < 0
    near = np.zeros_like(unknown)
    near[1:, :] |= unknown[:-1, :]
    near[:-1, :] |= unknown[1:, :]
    near[:, 1:] |= unknown[:, :-1]
    near[:, :-1] |= unknown[:, 1:]
    return free_mask(grid) & near


def cluster_cells(mask: np.ndarray, info: GridInfo, min_size: int = 6) -> list[Cluster]:
    """Group True cells into 8-connected clusters (BFS); drop clusters below `min_size` cells."""
    seen = np.zeros_like(mask, dtype=bool)
    rows, cols = mask.shape
    clusters = []
    for r0, c0 in np.argwhere(mask):
        if seen[r0, c0]:
            continue
        seen[r0, c0] = True
        queue = deque([(int(r0), int(c0))])
        members = []
        while queue:
            r, c = queue.popleft()
            members.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols and mask[nr, nc] and not seen[nr, nc]:
                        seen[nr, nc] = True
                        queue.append((nr, nc))
        if len(members) < min_size:
            continue
        cells = np.array(members, dtype=int)
        cx = info.origin_x + (cells[:, 1].mean() + 0.5) * info.resolution
        cy = info.origin_y + (cells[:, 0].mean() + 0.5) * info.resolution
        centres = np.column_stack([info.origin_x + (cells[:, 1] + 0.5) * info.resolution,
                                   info.origin_y + (cells[:, 0] + 0.5) * info.resolution])
        k = int(np.argmin(np.hypot(centres[:, 0] - cx, centres[:, 1] - cy)))
        clusters.append(Cluster(cells=cells, size=len(members), centroid=(float(cx), float(cy)),
                                anchor=(int(cells[k, 0]), int(cells[k, 1]))))
    return clusters


def map_frontiers(grid: np.ndarray, info: GridInfo, min_size: int = 6) -> list[Cluster]:
    """Map frontier clusters of an occupancy grid."""
    return cluster_cells(frontier_mask(grid), info, min_size)


def anchor_xy(info: GridInfo, cluster: Cluster) -> tuple[float, float]:
    """Map-frame centre of the cluster's anchor cell."""
    return cell_to_world(info, *cluster.anchor)
