"""Viewpoint selection for exploration (no ROS imports).

Public API (explorer_node, grid_sim):

    params = ExplorerParams(...)                         # explorer.yaml values (angles in rad)
    ex = ExplorationPlanner(params)
    d = ex.replan(grid, info, pose_xyyaw, now, seen)     # -> Decision
        d.goal: Viewpoint | None     (x, y, yaw, gain m^2, cost m, score)
        d.plan: PlanResult | None    A* path to the goal (Phase 3 library)
        d.done: True when no frontier/view cluster is left, or when the remaining clusters
                have had no reachable viewpoint for stuck_timeout (then d.goal is None)
        d.map_clusters, d.view_clusters: frontier clusters (for RViz)
    ex.goal_reached()                                    # drop the current goal
    costs = dijkstra_costs(pg, xy, wall_cost_weight)     # travel cost (m) to every cell

How a replan works:
  1. Planning grid from the map (unknown cells within `clear_radius` of the drone count as free).
  2. Targets = unknown cells + free cells the camera has not seen.
     Map frontiers (free next to unknown) and view frontiers (free, unseen) are clustered
     (frontier.cluster_cells, min_cluster_size cells).
  3. Candidates: reachable safe cells 0.6-1.5 m (sample_radii) from each cluster's anchor
     cell, one per direction sector (sample_directions) and radius band; heading = facing the
     anchor. Falls back to cells within coverage_range when the band has none.
  4. gain = m^2 of target cells visible from the candidate pose (coverage.ray_cells, rays stop
     at walls); cost = Dijkstra travel cost (one search from the drone per replan, same step cost
     as A*) + turn_cost x |heading change|; score = gain_weight x gain - cost.
  5. Commitment: the current goal is kept (re-scored) unless it was reached, became invalid
     (blocked, unreachable or no gain left) or a new candidate beats its score by
     goal_switch_margin (fraction of |score|).
  6. Stuck: if the travel cost to the goal has not dropped by PROGRESS_STEP within
     stuck_timeout, the goal is blacklisted (no candidates within BLACKLIST_RADIUS) and dropped.
  7. A* plans the path to the chosen goal only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from airmouse_nav.core.astar import PlanResult, plan
from airmouse_nav.core.coverage import ray_cells
from airmouse_nav.core.frontier import Cluster, cluster_cells, free_mask, frontier_mask
from airmouse_nav.core.grid_utils import (GridInfo, PlanningGrid, build_planning_grid,
                                          cell_to_world, clear_unknown_around, in_bounds,
                                          world_to_cell)

PROGRESS_STEP = 0.1       # m of travel cost that counts as progress (stuck rule)
BLACKLIST_RADIUS = 0.3    # m around a blacklisted goal where no candidate is taken


@dataclass(frozen=True)
class ExplorerParams:
    """Explorer parameters (explorer.yaml + planner.yaml). Angles in radians."""

    camera_hfov: float = math.radians(87.0)
    coverage_range: float = 2.5
    min_cluster_size: int = 6
    gain_weight: float = 1.0
    goal_switch_margin: float = 0.2
    stuck_timeout: float = 10.0
    turn_cost: float = 0.2                    # m per radian of heading change
    sample_radii: tuple[float, ...] = (0.6, 1.0, 1.5)
    sample_directions: int = 8
    guarded_radius: float = 0.30
    safety_margin: float = 0.10
    wall_cost_weight: float = 2.0
    wall_cost_range: float = 0.5
    clear_radius: float = 0.5                 # unknown within this of the drone counts as free


@dataclass
class Viewpoint:
    """A candidate pose and its score."""

    x: float
    y: float
    yaw: float
    gain: float = 0.0     # m^2
    cost: float = 0.0     # m
    score: float = 0.0


@dataclass
class Decision:
    """Result of one replan."""

    goal: Viewpoint | None
    plan: PlanResult | None
    done: bool
    map_clusters: list[Cluster] = field(default_factory=list)
    view_clusters: list[Cluster] = field(default_factory=list)
    reason: str = ''


def dijkstra_costs(pg: PlanningGrid, xy: tuple[float, float],
                   wall_cost_weight: float = 2.0) -> np.ndarray:
    """Travel cost (m, same step costs as A*) from xy to every cell; inf where unreachable.

    If xy is blocked, the search starts from the nearest free cell within 0.5 m (as A* does).
    """
    info = pg.info
    r, c = world_to_cell(info, *xy)
    if not in_bounds(info, r, c):
        return np.full((info.rows, info.cols), np.inf)
    if pg.blocked[r, c]:
        free = np.argwhere(~pg.blocked)
        if free.size == 0:
            return np.full((info.rows, info.cols), np.inf)
        d2 = (free[:, 0] - r) ** 2 + (free[:, 1] - c) ** 2
        k = int(np.argmin(d2))
        if d2[k] * info.resolution ** 2 > 0.25:
            return np.full((info.rows, info.cols), np.inf)
        r, c = int(free[k, 0]), int(free[k, 1])
    rows, cols = info.rows, info.cols
    ok = ~pg.blocked
    mult = 1.0 + wall_cost_weight * pg.wall_cost
    idx = np.arange(rows * cols).reshape(rows, cols)
    src, dst, w = [], [], []
    res = info.resolution
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
        r0, r1 = max(0, -dr), rows - max(0, dr)
        c0, c1 = max(0, -dc), cols - max(0, dc)
        a = ok[r0:r1, c0:c1]
        b = ok[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
        both = a & b
        if dr and dc:   # diagonal: both side cells must be free (no corner cutting)
            both &= ok[r0 + dr:r1 + dr, c0:c1] & ok[r0:r1, c0 + dc:c1 + dc]
        ia = idx[r0:r1, c0:c1][both]
        ib = idx[r0 + dr:r1 + dr, c0 + dc:c1 + dc][both]
        step = res * (math.sqrt(2.0) if dr and dc else 1.0)
        ma = mult[r0:r1, c0:c1][both]
        mb = mult[r0 + dr:r1 + dr, c0 + dc:c1 + dc][both]
        src += [ia, ib]
        dst += [ib, ia]
        w += [step * mb, step * ma]          # cost of entering the destination cell
    if not src:
        out = np.full((rows, cols), np.inf)
        out[r, c] = 0.0
        return out
    graph = coo_matrix((np.concatenate(w), (np.concatenate(src), np.concatenate(dst))),
                       shape=(rows * cols, rows * cols)).tocsr()
    dist = dijkstra(graph, directed=True, indices=int(idx[r, c]))
    return dist.reshape(rows, cols)


def view_gain(grid: np.ndarray, info: GridInfo, target: np.ndarray, x: float, y: float,
              yaw: float, p: ExplorerParams) -> float:
    """m^2 of target cells visible from (x, y, yaw)."""
    rows, cols = ray_cells(grid, info, x, y, yaw, p.camera_hfov, p.coverage_range)
    return float(target[rows, cols].sum()) * info.resolution ** 2


def sample_candidates(cluster: Cluster, info: GridInfo, pg: PlanningGrid, costs: np.ndarray,
                      p: ExplorerParams) -> list[Viewpoint]:
    """Reachable, unblocked poses near a cluster, each facing the cluster's anchor.

    Takes reachable safe cells whose distance to the anchor is within
    [min(sample_radii), max(sample_radii)], spread over `sample_directions` direction sectors
    and the radius bands (the nearest-to-band cell per sector and band). If none qualify, the
    reachable safe cells within coverage_range are used the same way (narrow corridors leave
    only a thin safe strip).
    """
    ar, ac = cluster.anchor
    ax, ay = cell_to_world(info, ar, ac)
    reach = int(math.ceil(p.coverage_range / info.resolution))
    r0, c0 = max(0, ar - reach), max(0, ac - reach)
    win = np.isfinite(costs[r0:ar + reach + 1, c0:ac + reach + 1]) & \
        ~pg.blocked[r0:ar + reach + 1, c0:ac + reach + 1]
    cells = np.argwhere(win)
    if cells.size == 0:
        return []
    cells += (r0, c0)
    dx = (cells[:, 1] - ac) * info.resolution
    dy = (cells[:, 0] - ar) * info.resolution
    dist = np.hypot(dx, dy)
    lo, hi = min(p.sample_radii), max(p.sample_radii)
    use = (dist >= lo) & (dist <= hi)
    if not use.any():
        use = dist <= p.coverage_range
    cells, dist, dx, dy = cells[use], dist[use], dx[use], dy[use]
    sector = ((np.arctan2(dy, dx) + math.pi) / (2 * math.pi) * p.sample_directions).astype(int)
    sector = np.minimum(sector, p.sample_directions - 1)
    bands = np.digitize(dist, sorted(p.sample_radii))
    out, taken = [], set()
    for k in np.lexsort((cells[:, 1], cells[:, 0], dist)):
        key = (int(sector[k]), int(bands[k]))
        if key in taken:
            continue
        taken.add(key)
        x, y = cell_to_world(info, int(cells[k, 0]), int(cells[k, 1]))
        out.append(Viewpoint(x, y, math.atan2(ay - y, ax - x),
                             cost=float(costs[cells[k, 0], cells[k, 1]])))
    return out


class ExplorationPlanner:
    """Stateful viewpoint chooser: commitment, blacklist and stuck detection."""

    def __init__(self, params: ExplorerParams) -> None:
        self.p = params
        self.goal: Viewpoint | None = None
        self.blacklist: list[tuple[float, float]] = []
        self.best_cost = math.inf
        self.last_progress = 0.0
        self.no_view_since: float | None = None   # clusters exist but no viewpoint reachable

    def goal_reached(self) -> None:
        """Forget the current goal (its view has been taken)."""
        self.goal = None

    def _score(self, v: Viewpoint, grid, info, target, costs, yaw_now) -> Viewpoint:
        r, c = world_to_cell(info, v.x, v.y)
        v.cost = float(costs[r, c]) + self.p.turn_cost * abs(
            math.remainder(v.yaw - yaw_now, 2 * math.pi))
        v.gain = view_gain(grid, info, target, v.x, v.y, v.yaw, self.p)
        v.score = self.p.gain_weight * v.gain - v.cost
        return v

    def replan(self, grid: np.ndarray, info: GridInfo, pose: tuple[float, float, float],
               now: float, seen: np.ndarray) -> Decision:
        """Choose (or keep) a viewpoint and plan a path to it. `seen` = coverage grid."""
        p = self.p
        x, y, yaw = pose
        g = clear_unknown_around(grid, info, (x, y), p.clear_radius)
        pg = build_planning_grid(g, info, p.guarded_radius, p.safety_margin, p.wall_cost_range)
        costs = dijkstra_costs(pg, (x, y), p.wall_cost_weight)
        free = free_mask(g)
        target = (g < 0) | (free & ~seen)
        min_gain = p.min_cluster_size * info.resolution ** 2
        map_cl = cluster_cells(frontier_mask(g), info, p.min_cluster_size)
        view_cl = cluster_cells(free & ~seen, info, p.min_cluster_size)

        cands = []
        for cl in map_cl + view_cl:
            for v in sample_candidates(cl, info, pg, costs, p):
                cands.append(self._score(v, g, info, target, costs, yaw))
        cands = [v for v in cands if v.gain >= min_gain]

        def best_allowed() -> Viewpoint | None:
            ok = [v for v in cands if all(math.hypot(v.x - bx, v.y - by) >= BLACKLIST_RADIUS
                                          for bx, by in self.blacklist)]
            return max(ok, key=lambda v: (v.score, -v.x, -v.y)) if ok else None

        best = best_allowed()

        # Commitment and stuck rule for the current goal.
        reason = ''
        if self.goal is not None:
            r, c = world_to_cell(info, self.goal.x, self.goal.y)
            valid = (in_bounds(info, r, c) and not pg.blocked[r, c]
                     and np.isfinite(costs[r, c]))
            if valid:
                self._score(self.goal, g, info, target, costs, yaw)
                valid = self.goal.gain >= min_gain
            if not valid:
                self.goal, reason = None, 'goal invalid'
            else:
                if self.goal.cost < self.best_cost - PROGRESS_STEP:
                    self.best_cost, self.last_progress = self.goal.cost, now
                elif now - self.last_progress > p.stuck_timeout:
                    self.blacklist.append((self.goal.x, self.goal.y))
                    self.goal, reason = None, 'stuck: goal blacklisted'
                    best = best_allowed()
            if self.goal is not None and best is not None and (
                    best.score > self.goal.score + p.goal_switch_margin * abs(self.goal.score)):
                self.goal, reason = None, 'better goal'
        if self.goal is None:
            if best is None:
                # Done only when nothing is left to see, or when what is left has been
                # unreachable for stuck_timeout (the map may just be momentarily blocked
                # around the drone).
                if not map_cl and not view_cl:
                    return Decision(None, None, True, map_cl, view_cl, 'nothing left to see')
                if self.no_view_since is None:
                    self.no_view_since = now
                if now - self.no_view_since > p.stuck_timeout:
                    return Decision(None, None, True, map_cl, view_cl,
                                    f'{len(map_cl) + len(view_cl)} clusters left, none reachable')
                return Decision(None, None, False, map_cl, view_cl,
                                reason or 'no reachable viewpoint (waiting)')
            self.goal = best
            self.best_cost, self.last_progress = best.cost, now

        self.no_view_since = None
        result = plan(pg, (x, y), (self.goal.x, self.goal.y), wall_cost_weight=p.wall_cost_weight)
        if not result.success:
            self.blacklist.append((self.goal.x, self.goal.y))
            self.goal = None
            return Decision(None, result, False, map_cl, view_cl, f'A* failed: {result.reason}')
        return Decision(self.goal, result, False, map_cl, view_cl, reason)
