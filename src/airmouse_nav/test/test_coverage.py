"""Unit tests for airmouse_nav.core.coverage ray casting."""

import math

import numpy as np

from airmouse_nav.core.coverage import Coverage, ray_cells
from airmouse_nav.core.grid_utils import GridInfo, world_to_cell

RES = 0.05
HFOV = math.radians(87)


def room(n: int = 100) -> tuple[np.ndarray, GridInfo]:
    """Free n x n room with a wall across it at x = 2.0 m."""
    g = np.zeros((n, n), np.int8)
    g[:, 40] = 100
    return g, GridInfo(RES, 0.0, 0.0, n, n)


def test_rays_stop_at_walls() -> None:
    """Cells behind a wall are not visible; the wall cell itself is."""
    g, info = room()
    rows, cols = ray_cells(g, info, 1.0, 2.5, 0.0, HFOV, 2.5)
    assert cols.max() == 40                 # the wall column, nothing beyond
    assert (g[rows, cols] == 100).any()


def test_fov_and_range() -> None:
    """Only cells inside the 87 degree cone and within range are visible."""
    g = np.zeros((100, 100), np.int8)
    info = GridInfo(RES, 0.0, 0.0, 100, 100)
    rows, cols = ray_cells(g, info, 2.5, 2.5, 0.0, HFOV, 2.0)
    xs = (cols + 0.5) * RES - 2.5
    ys = (rows + 0.5) * RES - 2.5
    d = np.hypot(xs, ys)
    assert d.max() <= 2.0 + RES
    ang = np.degrees(np.abs(np.arctan2(ys, xs)))[d > 0.3]
    assert ang.max() <= 87 / 2 + 6           # cone, allowing for cell size near the apex
    assert (xs >= -RES).all()                # nothing behind the camera
    r_behind, c_behind = world_to_cell(info, 1.5, 2.5)
    assert not ((rows == r_behind) & (cols == c_behind)).any()


def test_unknown_does_not_stop_rays_unless_asked() -> None:
    """The camera sees through unmapped space; with stop_at_unknown it does not."""
    g = np.zeros((100, 100), np.int8)
    g[:, 30:35] = -1
    info = GridInfo(RES, 0.0, 0.0, 100, 100)
    assert ray_cells(g, info, 1.0, 2.5, 0.0, HFOV, 2.5)[1].max() > 40
    assert ray_cells(g, info, 1.0, 2.5, 0.0, HFOV, 2.5, stop_at_unknown=True)[1].max() < 31


def test_coverage_marks_seen_free_cells_only() -> None:
    """update() marks free cells in view; walls are never 'seen'; view frontiers shrink."""
    g, info = room()
    cov = Coverage(info)
    before = sum(c.size for c in cov.view_frontiers(g))
    new = cov.update(g, 1.0, 2.5, 0.0, HFOV, 2.5)
    assert new > 0 and not cov.seen[:, 40].any() and not cov.seen[:, 41:].any()
    assert cov.update(g, 1.0, 2.5, 0.0, HFOV, 2.5) == 0
    assert sum(c.size for c in cov.view_frontiers(g)) == before - new


def test_match_keeps_overlap() -> None:
    """When the map grows, seen cells keep their map-frame position."""
    info = GridInfo(RES, 0.0, 0.0, 40, 40)
    cov = Coverage(info)
    cov.seen[10, 12] = True
    bigger = GridInfo(RES, -0.5, -0.25, 60, 60)
    cov.match(bigger)
    assert cov.seen.shape == (60, 60) and cov.seen[15, 22] and cov.seen.sum() == 1
