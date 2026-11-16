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
  3. Candidates: safe, reachable cells with a clear line of sight to a cluster anchor (the
     anchor cell, plus the first, middle and last cells of large clusters) within coverage_range; one per
     direction sector (sample_directions) and distance band (sample_radii, then out to
     coverage_range); heading = facing the anchor.
  4. gain = m^2 of target cells visible from the candidate pose (coverage.ray_cells, rays stop
     at walls); cost = Dijkstra travel cost (one search from the drone per replan, same step cost
     as A*) + turn_cost x |heading change|; score = gain_weight x gain - cost.
  5. Commitment: the current goal is kept (re-scored) unless it was reached, became invalid
     (blocked, unreachable or no gain left) or a new candidate beats its score by
     goal_switch_margin (fraction of |score|).
     A reached viewpoint is remembered; candidates within VISITED_RADIUS and VISITED_ANGLE of
     it are never taken again. Cells nearer than camera_min_range give no gain.
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

def relaxed_margins(safety_margin: float) -> tuple[float, ...]:
    """Safety margins to try in order: full, half, none (planning falls back only if needed)."""
    return (safety_margin, safety_margin / 2.0, 0.0) if safety_margin > 0 else (0.0,)


PROGRESS_STEP = 0.1       # m of travel cost that counts as progress (stuck rule)
BLACKLIST_RADIUS = 0.3    # m around a blacklisted goal where no candidate is taken
VISITED_RADIUS = 0.3      # m: a candidate this close to a visited viewpoint ...
VISITED_ANGLE = math.radians(30.0)   # ... with a heading this close is not taken again


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
    camera_min_range: float = 0.3             # m: cells nearer than this are not seen (no gain)


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
    d2 = ((info.origin_x + (cols + 0.5) * info.resolution - x) ** 2
          + (info.origin_y + (rows + 0.5) * info.resolution - y) ** 2)
    far = d2 >= p.camera_min_range ** 2
    return float(target[rows[far], cols[far]].sum()) * info.resolution ** 2


def cluster_anchors(cluster: Cluster) -> list[tuple[int, int]]:
    """Cells of a cluster to aim at: its anchor, plus the first, middle and last cells of large clusters."""
    anchors = [cluster.anchor]
    if cluster.size > 60:
        for q in (0, cluster.size // 2, cluster.size - 1):
            cell = (int(cluster.cells[q, 0]), int(cluster.cells[q, 1]))
            if cell not in anchors:
                anchors.append(cell)
    return anchors


def sample_candidates(cluster: Cluster, info: GridInfo, pg: PlanningGrid, costs: np.ndarray,
                      p: ExplorerParams, grid: np.ndarray | None = None) -> list[Viewpoint]:
    """Reachable, unblocked poses that can SEE the cluster, each facing one of its anchors.

    For each anchor (cluster_anchors), the pool is every safe, reachable cell with a clear
    line of sight to it within coverage_range (rays cast from the anchor, stopped by walls;
    visibility is symmetric) and at least camera_min_range away. From the pool, the cell
    nearest the anchor is taken in each direction sector (sample_directions) and distance band
    (sample_radii, then out to coverage_range). Without `grid` (no line-of-sight test) every
    reachable cell in range is in the pool.
    """
    out = []
    edges = sorted(p.sample_radii)
    for ar, ac in cluster_anchors(cluster):
        ax, ay = cell_to_world(info, ar, ac)
        if grid is not None:
            rows, cols = ray_cells(grid, info, ax, ay, 0.0, 2 * math.pi, p.coverage_range,
                                   coarse=True)
        else:
            reach = int(math.ceil(p.coverage_range / info.resolution))
            rr, cc = np.mgrid[max(0, ar - reach):min(info.rows, ar + reach + 1),
                              max(0, ac - reach):min(info.cols, ac + reach + 1)]
            rows, cols = rr.ravel(), cc.ravel()
        ok = np.isfinite(costs[rows, cols]) & ~pg.blocked[rows, cols]
        rows, cols = rows[ok], cols[ok]
        if rows.size == 0:
            continue
        dx = (cols - ac) * info.resolution
        dy = (rows - ar) * info.resolution
        dist = np.hypot(dx, dy)
        use = (dist >= p.camera_min_range) & (dist <= p.coverage_range)
        rows, cols, dist, dx, dy = rows[use], cols[use], dist[use], dx[use], dy[use]
        if rows.size == 0:
            continue
        sector = ((np.arctan2(dy, dx) + math.pi) / (2 * math.pi) * p.sample_directions).astype(int)
        sector = np.minimum(sector, p.sample_directions - 1)
        bands = np.digitize(dist, edges)
        taken = set()
        for k in np.lexsort((cols, rows, dist)):
            key = (int(sector[k]), int(bands[k]))
            if key in taken:
                continue
            taken.add(key)
            x, y = cell_to_world(info, int(rows[k]), int(cols[k]))
            out.append(Viewpoint(x, y, math.atan2(ay - y, ax - x),
                                 cost=float(costs[rows[k], cols[k]])))
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
        self.visited: list[tuple[float, float, float]] = []   # viewpoints already taken
        self.margin_used = params.safety_margin               # margin of the last replan

    def goal_reached(self) -> None:
        """The current goal's view has been taken: remember it (never re-pick the same
        pose: its predicted gain may not have materialised) and forget the goal."""
        if self.goal is not None:
            self.visited.append((self.goal.x, self.goal.y, self.goal.yaw))
        self.goal = None

    def _was_visited(self, v: Viewpoint) -> bool:
        return any(math.hypot(v.x - x, v.y - y) < VISITED_RADIUS
                   and abs(math.remainder(v.yaw - yaw, 2 * math.pi)) < VISITED_ANGLE
                   for x, y, yaw in self.visited)

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
        free = free_mask(g)
        target = (g < 0) | (free & ~seen)
        min_gain = p.min_cluster_size * info.resolution ** 2
        map_cl = cluster_cells(frontier_mask(g), info, p.min_cluster_size)
        view_cl = cluster_cells(free & ~seen, info, p.min_cluster_size)

        # Safety margin: the full value first. If no viewpoint at all is reachable (on a SLAM
        # map, smeared walls can close a 1 m corridor), step it down to half and then to zero
        # (the guarded radius alone) rather than giving up.
        for margin in relaxed_margins(p.safety_margin):
            pg = build_planning_grid(g, info, p.guarded_radius, margin, p.wall_cost_range)
            costs = dijkstra_costs(pg, (x, y), p.wall_cost_weight)
            cands = []
            for cl in map_cl + view_cl:
                cands += sample_candidates(cl, info, pg, costs, p, g)
            if cands:
                break
        self.margin_used = margin
        for v in cands:      # cost first (cheap); gain (a ray cast) only when it can matter
            v.cost += p.turn_cost * abs(math.remainder(v.yaw - yaw, 2 * math.pi))
        cands.sort(key=lambda v: (v.cost, v.x, v.y))
        # No view can gain more than the camera's whole field of view.
        max_gain = 0.5 * p.camera_hfov * p.coverage_range ** 2

        def best_allowed() -> Viewpoint | None:
            """Best-scoring allowed candidate. Candidates are tried in order of rising cost
            and the search stops once even a full-view gain could not beat the best so far
            (same result as scoring every candidate)."""
            top = None
            for v in cands:
                if top is not None and p.gain_weight * max_gain - v.cost < top.score:
                    break
                if self._was_visited(v) or any(
                        math.hypot(v.x - bx, v.y - by) < BLACKLIST_RADIUS
                        for bx, by in self.blacklist):
                    continue
                if v.score == 0.0 and v.gain == 0.0:
                    self._score(v, g, info, target, costs, yaw)
                if v.gain >= min_gain and (top is None or v.score > top.score):
                    top = v
            return top

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
