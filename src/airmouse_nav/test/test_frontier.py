"""Unit tests for airmouse_nav.core.frontier on hand-made grids."""

import numpy as np

from airmouse_nav.core.frontier import cluster_cells, frontier_mask, map_frontiers
from airmouse_nav.core.grid_utils import GridInfo

INFO = GridInfo(0.05, 0.0, 0.0, 20, 30)


def test_frontier_is_free_next_to_unknown() -> None:
    """Free cells with an unknown 4-neighbour are frontiers; walls and inner free cells are not."""
    g = np.full((5, 6), -1, np.int8)
    g[1:4, 0:3] = 0          # free block
    g[2, 1] = 0
    g[1:4, 3] = 100          # wall on its right
    m = frontier_mask(g)
    assert m[1, 0] and m[3, 2] and m[1, 1]          # touch unknown above / below
    assert not m[2, 1]                                # inner free cell
    assert not m[:, 3].any()                          # wall cells are never frontiers
    assert not m[2, 2]                                # neighbour on the right is a wall, not unknown


def test_no_unknown_no_frontiers() -> None:
    """A fully known map has no frontiers."""
    assert not frontier_mask(np.zeros((5, 5), np.int8)).any()


def test_clusters_8_connected_and_min_size() -> None:
    """Diagonal neighbours join a cluster; clusters below min_size are dropped."""
    mask = np.zeros((20, 30), bool)
    for k in range(8):
        mask[2 + k, 2 + k] = True      # diagonal line of 8 cells: one cluster
    mask[15, 20:23] = True             # 3 cells: dropped at min_size 6
    clusters = cluster_cells(mask, INFO, min_size=6)
    assert len(clusters) == 1 and clusters[0].size == 8
    assert len(cluster_cells(mask, INFO, min_size=3)) == 2


def test_cluster_centroid_and_anchor() -> None:
    """Centroid is the mean cell centre in metres; the anchor is a cluster cell."""
    mask = np.zeros((20, 30), bool)
    mask[10, 4:14] = True
    (c,) = cluster_cells(mask, INFO, min_size=6)
    assert c.centroid[0] == np.float64(0.45) and abs(c.centroid[1] - 0.525) < 1e-12   # cols 4..13
    assert mask[c.anchor]


def test_map_frontiers_corridor_end() -> None:
    """An open corridor end next to unknown space gives one frontier cluster there."""
    g = np.full((20, 40), -1, np.int8)
    g[5:15, 0:20] = 0
    g[4, 0:20] = 100
    g[15, 0:20] = 100
    clusters = map_frontiers(g, GridInfo(0.05, 0, 0, 20, 40), min_size=6)
    assert len(clusters) == 1
    assert set(clusters[0].cells[:, 1]) == {19}
