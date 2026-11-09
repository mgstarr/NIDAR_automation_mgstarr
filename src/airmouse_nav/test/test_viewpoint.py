"""Unit tests for airmouse_nav.core.viewpoint."""

import math

import numpy as np

from airmouse_nav.core.astar import plan
from airmouse_nav.core.coverage import ray_cells
from airmouse_nav.core.grid_utils import GridInfo, build_planning_grid, world_to_cell
from airmouse_nav.core.viewpoint import ExplorationPlanner, ExplorerParams, dijkstra_costs

RES = 0.05
P = ExplorerParams()


def corridor_with_unknown_room() -> tuple[np.ndarray, GridInfo]:
    """Known 1 m corridor along x (0..4 m); an unknown 2 x 2 m area beyond its open end."""
    g = np.full((60, 140), -1, np.int8)
    g[20:41, 0:81] = 100
    g[21:40, 1:81] = 0          # corridor, open at x = 4 m into unknown space
    return g, GridInfo(RES, 0.0, 0.0, 60, 140)


def test_dijkstra_matches_astar_cost() -> None:
    """One Dijkstra search gives the same travel cost as A* to a goal."""
    g, info = corridor_with_unknown_room()
    pg = build_planning_grid(g, info)
    costs = dijkstra_costs(pg, (0.5, 1.5))
    res = plan(pg, (0.5, 1.5), (3.5, 1.5), simplify=False)
    r, c = world_to_cell(info, 3.5, 1.5)
    assert res.success and abs(costs[r, c] - res.cost) < 1e-6
    assert np.isinf(costs[0, 0])


def test_chosen_viewpoint_sees_its_cluster() -> None:
    """The chosen goal faces the frontier at the corridor end and sees target cells."""
    g, info = corridor_with_unknown_room()
    ex = ExplorationPlanner(P)
    d = ex.replan(g, info, (0.5, 1.5, 0.0), 0.0, g == 0)   # corridor already seen
    assert not d.done and d.goal is not None and d.plan.success
    rows, cols = ray_cells(g, info, d.goal.x, d.goal.y, d.goal.yaw, P.camera_hfov,
                           P.coverage_range)
    assert ((g[rows, cols] < 0)).sum() > 50              # it sees the unknown area
    assert d.goal.gain > 0 and d.goal.score == d.goal.gain - d.goal.cost


def test_commitment_keeps_goal() -> None:
    """Replanning from the same state keeps the same goal."""
    g, info = corridor_with_unknown_room()
    ex = ExplorationPlanner(P)
    seen = np.zeros(g.shape, bool)
    a = ex.replan(g, info, (0.5, 1.5, 0.0), 0.0, seen).goal
    b = ex.replan(g, info, (0.6, 1.5, 0.0), 1.0, seen).goal
    assert (a.x, a.y) == (b.x, b.y)


def test_stuck_goal_is_blacklisted() -> None:
    """No progress for stuck_timeout -> goal blacklisted, a different goal is chosen."""
    g, info = corridor_with_unknown_room()
    ex = ExplorationPlanner(P)
    seen = np.zeros(g.shape, bool)
    first = ex.replan(g, info, (0.5, 1.5, 0.0), 0.0, seen).goal
    ex.replan(g, info, (0.5, 1.5, 0.0), 5.0, seen)
    d = ex.replan(g, info, (0.5, 1.5, 0.0), 11.0, seen)
    assert ex.blacklist and (first.x, first.y) in ex.blacklist
    assert d.goal is None or math.hypot(d.goal.x - first.x, d.goal.y - first.y) >= 0.3


def test_done_when_everything_known_and_seen() -> None:
    """A closed, fully known room that has been fully seen leaves nothing to explore."""
    g = np.full((40, 40), 100, np.int8)
    g[1:39, 1:39] = 0
    info = GridInfo(RES, 0.0, 0.0, 40, 40)
    seen = g == 0
    d = ExplorationPlanner(P).replan(g, info, (1.0, 1.0, 0.0), 0.0, seen)
    assert d.done and d.goal is None


def test_unreachable_clusters_wait_before_done() -> None:
    """Clusters that exist but cannot be reached do not end exploration at once."""
    g = np.full((40, 80), -1, np.int8)
    g[10:31, 0:41] = 100
    g[11:30, 1:40] = 0                     # closed room ...
    g[11:30, 50:70] = 0                    # ... and a separate known pocket next to unknown
    info = GridInfo(RES, 0.0, 0.0, 40, 80)
    seen = g == 0
    seen[11:30, 50:70] = False
    ex = ExplorationPlanner(P)
    d1 = ex.replan(g, info, (1.0, 1.0, 0.0), 0.0, seen)
    assert not d1.done and d1.goal is None and 'waiting' in d1.reason
    d2 = ex.replan(g, info, (1.0, 1.0, 0.0), P.stuck_timeout + 1.0, seen)
    assert d2.done
