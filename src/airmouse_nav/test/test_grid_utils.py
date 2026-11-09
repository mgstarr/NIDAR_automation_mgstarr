"""Unit tests for airmouse_nav.core.grid_utils."""

import math

import numpy as np
import pytest

from airmouse_nav.core.grid_utils import (GridInfo, build_planning_grid, cell_to_world,
                                          clearance_map, grid_from_occupancy, in_bounds,
                                          obstacle_mask, world_to_cell)

RES = 0.05


def test_grid_from_occupancy_row_major() -> None:
    """Flat data fills rows from the lowest y up."""
    grid = grid_from_occupancy([0, 100, -1, 0, 0, 0], width=3, height=2)
    assert grid.shape == (2, 3)
    assert grid[0, 1] == 100 and grid[0, 2] == -1 and grid.dtype == np.int8


def test_grid_from_occupancy_bad_size() -> None:
    """A size mismatch is a programming error and raises."""
    with pytest.raises(ValueError):
        grid_from_occupancy([0, 0, 0], width=2, height=2)


@pytest.mark.parametrize('xy', [(0.0, 0.0), (1.234, -0.51), (-2.0, 3.97)])
def test_metres_cells_round_trip(xy: tuple[float, float]) -> None:
    """A point maps to a cell whose centre is within half a cell of it."""
    info = GridInfo(RES, -3.0, -2.0, 200, 200)
    row, col = world_to_cell(info, *xy)
    x, y = cell_to_world(info, row, col)
    assert abs(x - xy[0]) <= RES / 2 + 1e-9 and abs(y - xy[1]) <= RES / 2 + 1e-9
    assert world_to_cell(info, x, y) == (row, col)
    assert in_bounds(info, row, col)
    assert not in_bounds(info, -1, 0) and not in_bounds(info, 0, 200)


def test_unknown_counts_as_obstacle() -> None:
    """Unknown (-1) and occupied cells are obstacles; free cells are not."""
    mask = obstacle_mask(np.array([[0, -1, 100, 49, 50]], dtype=np.int8))
    assert mask.tolist() == [[False, True, True, False, True]]


def test_clearance_without_obstacles_is_infinite() -> None:
    """An empty grid has infinite clearance everywhere."""
    assert np.isinf(clearance_map(np.zeros((4, 4), bool), RES)).all()


def test_inflation_radius_around_single_obstacle() -> None:
    """Every cell within guarded_radius + safety_margin of an obstacle is blocked, others are not."""
    grid = np.zeros((41, 41), np.int8)
    grid[20, 20] = 100
    pg = build_planning_grid(grid, GridInfo(RES, 0.0, 0.0, 41, 41), 0.30, 0.10, 0.5)
    rr, cc = np.mgrid[0:41, 0:41]
    edge_dist = np.hypot(rr - 20, cc - 20) * RES - RES / 2
    assert pg.blocked[(edge_dist < 0.40 - 1e-6)].all()
    assert not pg.blocked[(edge_dist >= 0.40 + 1e-6)].any()
    assert math.isclose(pg.inflation_radius, 0.40)


def test_wall_cost_shape() -> None:
    """Wall cost is 1 at the inflation edge, falls with distance and is 0 from 0.5 m on."""
    grid = np.zeros((1, 40), np.int8)
    grid[0, 0] = 100
    pg = build_planning_grid(grid, GridInfo(RES, 0.0, 0.0, 1, 40), 0.30, 0.10, 0.5)
    free = ~pg.blocked[0]
    cost = pg.wall_cost[0][free]
    clearance = pg.clearance[0][free]
    assert cost[0] > 0.7 and np.all(np.diff(cost) <= 0)
    assert np.all(cost[clearance >= 0.5] == 0)
    assert np.all(pg.wall_cost[0][~free] == 1)


def test_shape_mismatch_raises() -> None:
    """GridInfo must match the grid."""
    with pytest.raises(ValueError):
        build_planning_grid(np.zeros((3, 3), np.int8), GridInfo(RES, 0, 0, 4, 3))


def test_unknown_is_blocked_but_not_inflated() -> None:
    """Unknown cells are blocked themselves; only occupied cells get the inflation band."""
    grid = np.zeros((41, 41), np.int8)
    grid[20, 20] = -1
    pg = build_planning_grid(grid, GridInfo(RES, 0.0, 0.0, 41, 41), 0.30, 0.10, 0.5)
    assert pg.blocked[20, 20]
    assert not pg.blocked[20, 21] and not pg.blocked[19, 20]
    assert pg.blocked.sum() == 1
