"""A* path planner on the 2D grid (no ROS imports).

Public API (Phase 5's explorer calls this directly):

    from airmouse_nav.core.grid_utils import build_planning_grid
    from airmouse_nav.core.astar import plan, PlanResult

    pg = build_planning_grid(grid, info, guarded_radius, safety_margin, wall_cost_range)
    result: PlanResult = plan(pg, start_xy=(x0, y0), goal_xy=(x1, y1),
                              wall_cost_weight=2.0, simplify=True, start_snap_radius=0.5)

    result.success       bool
    result.path          list[(x, y)] in metres, map frame; first = start cell centre (or the
                         snapped start), last = goal cell centre; empty on failure
    result.length        float, metres along `path`
    result.cost          float, search cost (sum of step length x (1 + weight x wall cost))
    result.reason        'ok' | 'start_blocked' | 'goal_blocked' | 'no_path'
    result.start_snapped True if the start was inside inflation and moved to the nearest free cell
    result.expanded      int, cells expanded (for profiling)

Normal failures never raise; they return success=False with a reason:
  - 'start_blocked': start off the grid, or no free cell within `start_snap_radius`;
  - 'goal_blocked':  goal off the grid, or in a blocked cell (obstacle, unknown, inflation);
  - 'no_path':       no route between them through free cells.

Search: 8-connected A* with the octile-distance heuristic; a diagonal move needs both side
neighbours free (no corner cutting). Moving into a cell costs step_length x (1 +
wall_cost_weight x wall_cost[cell]), so paths prefer corridor centres. Ties break on a fixed
order, so the same input always gives the same path.

Simplification: from each kept point, the path jumps to a far later point (found by galloping
then binary search) that has a clear line of sight (every cell on the segment free) and whose segment is no closer to the
walls than the original path between them (max wall cost not higher). This removes staircase
points while keeping corridor centring and not cutting corners tight.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from airmouse_nav.core.grid_utils import (PlanningGrid, cell_to_world, in_bounds,
                                          world_to_cell)

SQRT2 = math.sqrt(2.0)
COST_TOLERANCE = 0.05          # simplification may raise the max wall cost by at most this

REASON_OK = 'ok'
REASON_START_BLOCKED = 'start_blocked'
REASON_GOAL_BLOCKED = 'goal_blocked'
REASON_NO_PATH = 'no_path'


@dataclass
class PlanResult:
    """Outcome of one planning call."""

    success: bool
    path: list[tuple[float, float]] = field(default_factory=list)
    length: float = 0.0
    cost: float = 0.0
    reason: str = REASON_OK
    start_snapped: bool = False
    expanded: int = 0


def _nearest_free(pg: PlanningGrid, row: int, col: int, max_dist: float) -> tuple[int, int] | None:
    """Nearest unblocked cell to (row, col) within max_dist metres; ties go to the lowest (row, col)."""
    info = pg.info
    reach = int(math.ceil(max_dist / info.resolution))
    r0, r1 = max(0, row - reach), min(info.rows, row + reach + 1)
    c0, c1 = max(0, col - reach), min(info.cols, col + reach + 1)
    free = np.argwhere(~pg.blocked[r0:r1, c0:c1])
    if free.size == 0:
        return None
    rr, cc = free[:, 0] + r0, free[:, 1] + c0
    d2 = (rr - row) ** 2 + (cc - col) ** 2
    ok = d2 * info.resolution ** 2 <= max_dist ** 2 + 1e-12
    if not ok.any():
        return None
    order = np.lexsort((cc[ok], rr[ok], d2[ok]))
    k = order[0]
    return int(rr[ok][k]), int(cc[ok][k])


def _search(blocked: np.ndarray, mult: np.ndarray, start: int, goal: int, cols: int,
            res: float) -> tuple[list[int], float, int]:
    """A* on a padded, flattened grid. Returns (cell indices start..goal, cost, expanded)."""
    blk = blocked.tolist()
    mlt = mult.tolist()
    n = len(blk)
    g = [math.inf] * n
    parent = [-1] * n
    closed = bytearray(n)
    gr, gc = divmod(goal, cols)
    d_diag = (SQRT2 - 2.0) * res
    straight = ((-cols, res), (cols, res), (-1, res), (1, res))
    diagonal = ((-cols - 1, -cols, -1), (-cols + 1, -cols, 1),
                (cols - 1, cols, -1), (cols + 1, cols, 1))
    diag_len = SQRT2 * res
    g[start] = 0.0
    sr, sc = divmod(start, cols)
    dx, dy = abs(sc - gc), abs(sr - gr)
    h0 = res * (dx + dy) + d_diag * (dx if dx < dy else dy)
    heap = [(h0, h0, start)]
    push, pop = heapq.heappush, heapq.heappop
    expanded = 0
    while heap:
        _, _, cur = pop(heap)
        if closed[cur]:
            continue
        if cur == goal:
            break
        closed[cur] = 1
        expanded += 1
        gcur = g[cur]
        for off, step in straight:
            nb = cur + off
            if blk[nb] or closed[nb]:
                continue
            ng = gcur + step * mlt[nb]
            if ng < g[nb]:
                g[nb] = ng
                parent[nb] = cur
                r, c = divmod(nb, cols)
                dx, dy = abs(c - gc), abs(r - gr)
                h = res * (dx + dy) + d_diag * (dx if dx < dy else dy)
                push(heap, (ng + h, h, nb))
        for off, side_a, side_b in diagonal:
            nb = cur + off
            if blk[nb] or closed[nb] or blk[cur + side_a] or blk[cur + side_b]:
                continue
            ng = gcur + diag_len * mlt[nb]
            if ng < g[nb]:
                g[nb] = ng
                parent[nb] = cur
                r, c = divmod(nb, cols)
                dx, dy = abs(c - gc), abs(r - gr)
                h = res * (dx + dy) + d_diag * (dx if dx < dy else dy)
                push(heap, (ng + h, h, nb))
    if g[goal] == math.inf:
        return [], math.inf, expanded
    cells = [goal]
    while cells[-1] != start:
        cells.append(parent[cells[-1]])
    cells.reverse()
    return cells, g[goal], expanded


def _segment_cells(r0: int, c0: int, r1: int, c1: int) -> tuple[np.ndarray, np.ndarray]:
    """Every cell (rows, cols) that the segment between two cell centres passes through.

    Exact: the segment is split where it crosses grid lines, and the cell holding each piece's
    midpoint is taken, plus the cells at the crossing points themselves (so passing exactly
    through a cell corner also counts the cells meeting there).
    """
    y0, x0, y1, x1 = r0 + 0.5, c0 + 0.5, r1 + 0.5, c1 + 0.5
    dy, dx = y1 - y0, x1 - x0
    ts = [np.array([0.0, 1.0])]
    if dx:
        xs = np.arange(min(c0, c1) + 1, max(c0, c1) + 1, dtype=float)
        ts.append((xs - x0) / dx)
    if dy:
        ys = np.arange(min(r0, r1) + 1, max(r0, r1) + 1, dtype=float)
        ts.append((ys - y0) / dy)
    t = np.unique(np.concatenate(ts))
    mids = (t[:-1] + t[1:]) / 2
    rows = [np.floor(y0 + mids * dy)]
    cols = [np.floor(x0 + mids * dx)]
    # Passing exactly through a cell corner: count all four cells that meet there.
    py, px = y0 + t * dy, x0 + t * dx
    corner = np.isclose(py, np.round(py)) & np.isclose(px, np.round(px))
    cy, cx = np.round(py[corner]), np.round(px[corner])
    for oy in (0, -1):
        for ox in (0, -1):
            rows.append(cy + oy)
            cols.append(cx + ox)
    rows = np.concatenate(rows).astype(np.intp)
    cols = np.concatenate(cols).astype(np.intp)
    return rows, cols


def _simplify(pg: PlanningGrid, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Keep only the points needed for clear, no-closer-to-walls straight segments."""
    if len(cells) <= 2:
        return list(cells)
    cost = pg.wall_cost
    cell_cost = [float(cost[rc]) for rc in cells]

    def shortcut_ok(i: int, j: int) -> bool:
        rows, cols = _segment_cells(*cells[i], *cells[j])
        if pg.blocked[rows, cols].any():
            return False
        return cost[rows, cols].max() <= max(cell_cost[i:j + 1]) + COST_TOLERANCE

    kept = [cells[0]]
    anchor, last = 0, len(cells) - 1
    while anchor < last:
        # Gallop forward (1, 2, 4, ... points ahead), then binary-search the furthest good one.
        good, step = anchor + 1, 1
        bad = None
        while True:
            j = min(anchor + 2 * step, last)
            if j == good:
                break
            if shortcut_ok(anchor, j):
                good, step = j, step * 2
                if j == last:
                    break
            else:
                bad = j
                break
        if bad is not None:
            while bad - good > 1:
                mid = (good + bad) // 2
                if shortcut_ok(anchor, mid):
                    good = mid
                else:
                    bad = mid
        kept.append(cells[good])
        anchor = good
    return kept


def plan(pg: PlanningGrid, start_xy: tuple[float, float], goal_xy: tuple[float, float],
         wall_cost_weight: float = 2.0, simplify: bool = True,
         start_snap_radius: float = 0.5) -> PlanResult:
    """Plan a path from start_xy to goal_xy (map frame, metres) on a PlanningGrid.

    Args:
        pg: Grid from `grid_utils.build_planning_grid`.
        start_xy: Drone position (x, y).
        goal_xy: Goal position (x, y).
        wall_cost_weight: How strongly paths prefer corridor centres (`wall_cost_weight`).
        simplify: Drop points that have a clear line of sight to a later point.
        start_snap_radius: If the start is blocked, look this far (m) for the nearest free cell.

    Returns:
        PlanResult (see the module docstring). Never raises for blocked or unreachable input.
    """
    info = pg.info
    sr, sc = world_to_cell(info, *start_xy)
    gr, gc = world_to_cell(info, *goal_xy)
    if not in_bounds(info, gr, gc) or pg.blocked[gr, gc]:
        return PlanResult(success=False, reason=REASON_GOAL_BLOCKED)
    snapped = False
    if not in_bounds(info, sr, sc) or pg.blocked[sr, sc]:
        if not in_bounds(info, sr, sc):
            return PlanResult(success=False, reason=REASON_START_BLOCKED)
        free = _nearest_free(pg, sr, sc, start_snap_radius)
        if free is None:
            return PlanResult(success=False, reason=REASON_START_BLOCKED)
        sr, sc = free
        snapped = True

    if pg.components[sr, sc] != pg.components[gr, gc]:
        return PlanResult(success=False, reason=REASON_NO_PATH, start_snapped=snapped)

    # Pad with a ring of blocked cells so neighbour lookups never leave the grid.
    blocked = np.pad(pg.blocked, 1, constant_values=True)
    mult = np.pad(1.0 + wall_cost_weight * pg.wall_cost, 1, constant_values=1.0)
    cols = info.cols + 2
    start = (sr + 1) * cols + sc + 1
    goal = (gr + 1) * cols + gc + 1
    flat, cost, expanded = _search(blocked.ravel(), mult.ravel(), start, goal, cols,
                                   info.resolution)
    if not flat:
        return PlanResult(success=False, reason=REASON_NO_PATH, start_snapped=snapped,
                          expanded=expanded)
    cells = [((i // cols) - 1, (i % cols) - 1) for i in flat]
    if simplify:
        cells = _simplify(pg, cells)
    path = [cell_to_world(info, r, c) for r, c in cells]
    if len(path) == 1:
        path = path * 2
    length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    return PlanResult(success=True, path=path, length=length, cost=cost, reason=REASON_OK,
                      start_snapped=snapped, expanded=expanded)
