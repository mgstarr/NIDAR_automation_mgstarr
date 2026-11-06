"""Unit tests for airmouse_nav.core.astar: synthetic grids and the Phase 2 saved maps."""

import sys
import time
from pathlib import Path

import numpy as np
import pytest

from airmouse_nav.core.astar import PlanResult, plan
from airmouse_nav.core.grid_utils import (GridInfo, PlanningGrid, build_planning_grid,
                                          cell_to_world, world_to_cell)

RES = 0.05
MAPS_DIR = Path(__file__).parent / 'data' / 'maps'
TOOLS_DIR = Path(__file__).resolve().parents[3] / 'tools'


def corridor_grid(length_cells: int = 100, clear_cells: int = 20) -> np.ndarray:
    """A straight corridor along x, `clear_cells` wide, walls all around."""
    grid = np.full((clear_cells + 2, length_cells + 2), 100, np.int8)
    grid[1:-1, 1:-1] = 0
    return grid


def make_pg(grid: np.ndarray) -> PlanningGrid:
    """Planning grid with the PLAN.md defaults (0.30 + 0.10 m, cost range 0.5 m)."""
    return build_planning_grid(grid, GridInfo(RES, 0.0, 0.0, *grid.shape), 0.30, 0.10, 0.5)


def sample_path(path: list[tuple[float, float]], step: float = RES / 4) -> np.ndarray:
    """Points every `step` metres along a polyline."""
    pts = [np.asarray(path[0])]
    for a, b in zip(path, path[1:]):
        a, b = np.asarray(a), np.asarray(b)
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        pts += [a + (b - a) * t for t in np.linspace(0, 1, n + 1)[1:]]
    return np.array(pts)


def assert_clear(pg: PlanningGrid, result: PlanResult, skip_first: bool = False) -> None:
    """Every point along the path lies in a free (not blocked, not unknown) cell."""
    pts = sample_path(result.path)
    if skip_first:
        pts = pts[1:]
    for x, y in pts:
        row, col = world_to_cell(pg.info, x, y)
        assert not pg.blocked[row, col], f'path crosses blocked cell at ({x:.2f}, {y:.2f})'


def test_straight_corridor_is_centred() -> None:
    """In a 1 m corridor the path stays within +/-0.1 m of the centre line."""
    grid = corridor_grid()
    pg = make_pg(grid)
    centre_y = grid.shape[0] * RES / 2
    res = plan(pg, (0.6, centre_y - 0.05), (4.5, centre_y + 0.05))
    assert res.success and res.reason == 'ok'
    assert_clear(pg, res)
    assert np.all(np.abs(sample_path(res.path)[:, 1] - centre_y) <= 0.1)
    assert res.length == pytest.approx(3.9, abs=0.15)
    assert len(res.path) <= 4


def test_l_turn() -> None:
    """An L-shaped corridor: the path turns the corner without entering inflation."""
    grid = np.full((102, 102), 100, np.int8)
    grid[1:21, 1:101] = 0          # along x at the bottom
    grid[1:101, 81:101] = 0        # up along y at the right end
    pg = make_pg(grid)
    res = plan(pg, (0.6, 0.55), (4.6, 4.6))
    assert res.success
    assert_clear(pg, res)
    assert 7.0 < res.length < 8.5


def test_dead_end_branch_reached_and_avoided() -> None:
    """A goal at the end of a dead end is reached; a goal past it is not entered via the branch."""
    grid = corridor_grid(100)
    grid = np.vstack([grid, np.full((40, grid.shape[1]), 100, np.int8)])
    grid[21:58, 40:60] = 0         # dead-end branch going up from the middle
    pg = make_pg(grid)
    into = plan(pg, (0.6, 0.55), (2.5, 2.4))
    assert into.success
    assert_clear(pg, into)
    past = plan(pg, (0.6, 0.55), (4.5, 0.55))
    assert past.success
    assert np.all(sample_path(past.path)[:, 1] < 0.7)


def test_unreachable_goal() -> None:
    """Two separate rooms: no path, no exception."""
    grid = corridor_grid(100)
    grid[:, 50] = 100
    res = plan(make_pg(grid), (0.6, 0.55), (4.5, 0.55))
    assert not res.success and res.reason == 'no_path' and res.path == []


def test_unknown_cells_block_the_way() -> None:
    """A band of unknown cells across the corridor blocks it like a wall."""
    grid = corridor_grid(100)
    grid[:, 50:52] = -1
    assert plan(make_pg(grid), (0.6, 0.55), (4.5, 0.55)).reason == 'no_path'


@pytest.mark.parametrize('goal', [(2.5, 0.025), (2.5, 0.15), (99.0, 0.5)])
def test_goal_in_wall_inflation_or_off_grid(goal: tuple[float, float]) -> None:
    """A goal in a wall, in the inflation or off the grid is refused as goal_blocked."""
    res = plan(make_pg(corridor_grid()), (0.6, 0.55), goal)
    assert not res.success and res.reason == 'goal_blocked'


def test_start_inside_inflation_snaps() -> None:
    """A start 0.15 m from a wall plans from the nearest free cell."""
    pg = make_pg(corridor_grid())
    res = plan(pg, (1.0, 0.2), (4.0, 0.55))
    assert res.success and res.start_snapped
    sx, sy = res.path[0]
    assert abs(sx - 1.0) < 0.1 and 0.4 < sy < 0.5
    assert_clear(pg, res)


def test_start_blocked_when_no_free_cell_near() -> None:
    """A start deep inside a wall block, or off the grid, is start_blocked."""
    grid = np.full((60, 60), 100, np.int8)
    grid[1:21, 1:59] = 0
    pg = make_pg(grid)
    assert plan(pg, (1.5, 2.5), (1.5, 0.55)).reason == 'start_blocked'
    assert plan(pg, (-5.0, 0.5), (1.5, 0.55)).reason == 'start_blocked'


def test_start_equals_goal() -> None:
    """Start and goal in the same cell: success, zero length."""
    res = plan(make_pg(corridor_grid()), (2.0, 0.55), (2.0, 0.56))
    assert res.success and res.length == 0.0 and len(res.path) == 2


def test_deterministic() -> None:
    """The same input always gives the same path."""
    grid = np.zeros((120, 120), np.int8)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 100
    grid[30:90, 60] = 100
    pg = make_pg(grid)
    results = [plan(pg, (0.6, 3.0), (5.4, 3.0)) for _ in range(3)]
    assert results[0].success
    assert all(r.path == results[0].path and r.cost == results[0].cost for r in results)


def load_saved_maps() -> list:
    """Phase 2 saved maps as (name, PlanningGrid)."""
    sys.path.insert(0, str(TOOLS_DIR))
    from map_io import load  # noqa: E402  (tools/ is not a package)
    maps = []
    for yaml_path in sorted(MAPS_DIR.glob('*.yaml')):
        m = load(yaml_path)
        info = GridInfo(m.resolution, m.origin[0], m.origin[1], *m.grid.shape)
        maps.append((yaml_path.stem, build_planning_grid(m.grid, info)))
    return maps


SAVED_MAPS = load_saved_maps()


@pytest.mark.parametrize('name,pg', SAVED_MAPS, ids=[n for n, _ in SAVED_MAPS])
def test_saved_maps_random_reachable_pairs(name: str, pg: PlanningGrid) -> None:
    """On each saved map, 30 random pairs in the main free region all succeed and stay clear."""
    sizes = np.bincount(pg.components.ravel())
    sizes[0] = 0
    cells = np.argwhere(pg.components == sizes.argmax())
    rng = np.random.default_rng(42)
    for _ in range(30):
        a, b = cells[rng.integers(len(cells), size=2)]
        res = plan(pg, cell_to_world(pg.info, *a), cell_to_world(pg.info, *b))
        assert res.success, f'{name}: {res.reason}'
        assert_clear(pg, res)


def test_runtime_under_50ms() -> None:
    """Corner-to-corner plans on every saved map (about 310 x 310 cells) take under 50 ms."""
    worst = 0.0
    for _, pg in SAVED_MAPS:
        sizes = np.bincount(pg.components.ravel())
        sizes[0] = 0
        cells = np.argwhere(pg.components == sizes.argmax())
        a, b = cells[cells.sum(1).argmin()], cells[cells.sum(1).argmax()]
        start, goal = cell_to_world(pg.info, *a), cell_to_world(pg.info, *b)
        best = min(_timed(pg, start, goal) for _ in range(3))
        worst = max(worst, best)
    assert worst < 0.050, f'worst plan took {worst * 1e3:.1f} ms'


def _timed(pg: PlanningGrid, start: tuple[float, float], goal: tuple[float, float]) -> float:
    t0 = time.perf_counter()
    assert plan(pg, start, goal).success
    return time.perf_counter() - t0
