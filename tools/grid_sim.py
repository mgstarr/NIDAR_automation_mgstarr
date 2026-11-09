#!/usr/bin/env python3
"""Fast pure-Python mission simulator (Phase 5): no Gazebo, no ROS.

The ground-truth maze comes from the maze generator (same seed = same maze as in Gazebo).
A simulated camera (camera_hfov, 0.3-3.0 m) and, optionally, a 360-degree LiDAR (0.05-3.0 m)
reveal it into a map (-1 unknown, 0 free, 100 occupied). The same core modules as the explorer
node then run the mission: take-off, 360-degree look-around, EXPLORE (viewpoints + A*),
RETURN_HOME, LAND. The drone moves kinematically along each A* path at `speed`, turning at
max_yaw_rate (heading modes as in PLAN.md).

Usage:
    python3 tools/grid_sim.py --seeds 1-50 [--lidar false] [--quiet]
    python3 tools/grid_sim.py --seeds 3 --plot /tmp/seed3.png      # map + coverage + track

Checks per run: every room >= 95% seen, every survivor hit by a camera ray, ends at home
(within 0.3 m) and landed, never in a cell that was blocked or unknown on the planning grid when
it entered it, and true wall clearance always >= guarded_radius.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src' / 'airmouse_nav'))

from airmouse_nav.core.astar import plan  # noqa: E402
from airmouse_nav.core.coverage import Coverage, ray_cells  # noqa: E402
from airmouse_nav.core.grid_utils import (GridInfo, build_planning_grid,  # noqa: E402
                                          clear_unknown_around, world_to_cell)
from airmouse_nav.core.mission import Mission, State, Trigger, should_return  # noqa: E402
from airmouse_nav.core.viewpoint import (ExplorationPlanner, ExplorerParams,  # noqa: E402
                                         dijkstra_costs)

_spec = importlib.util.spec_from_file_location(
    'generate_maze', REPO / 'src' / 'airmouse_description' / 'scripts' / 'generate_maze.py')
gm = importlib.util.module_from_spec(_spec)
sys.modules['generate_maze'] = gm
_spec.loader.exec_module(gm)

MARGIN = 1.0              # m of open floor simulated around the arena (outside the entry)
SURVIVOR_RADIUS = 0.20
CAMERA_MIN_RANGE = 0.3
SENSOR_RANGE = 3.0        # depth / LiDAR range used for mapping (Grid/RangeMax)
DT = 0.25                 # s per simulation step
LANDING_TIME = 15.0       # s (owner approved)


@dataclass
class SimResult:
    """Outcome of one simulated mission."""

    seed: int
    use_lidar: bool
    state: str = ''
    time: float = 0.0
    path_length: float = 0.0
    rooms_seen: int = 0
    rooms: int = 0
    survivors_seen: int = 0
    survivors: int = 0
    coverage: float = 0.0           # seen fraction of true free cells inside the arena
    home_error: float = math.inf
    violations: list[str] = field(default_factory=list)
    min_clearance: float = math.inf
    wall_time: float = 0.0
    track: list[tuple[float, float]] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        """All Phase 5 checks passed."""
        return (self.state == 'DONE' and self.rooms_seen == self.rooms
                and self.survivors_seen == self.survivors and self.home_error <= 0.3
                and not self.violations)


class World:
    """Ground truth: walls, survivors, rooms on a grid with a margin around the arena."""

    def __init__(self, seed: int) -> None:
        self.maze = gm.generate(gm.MazeConfig(seed=seed))
        occ, (x0, y0) = gm.rasterize(self.maze)
        res = self.maze.config.resolution
        m = int(round(MARGIN / res))
        self.occ = np.pad(occ, m)
        self.info = GridInfo(res, x0 - MARGIN, y0 - MARGIN, *self.occ.shape)
        rows, cols = np.mgrid[0:self.info.rows, 0:self.info.cols]
        cx = self.info.origin_x + (cols + 0.5) * res
        cy = self.info.origin_y + (rows + 0.5) * res
        self.survivor_id = np.full(self.occ.shape, -1, int)
        for k, s in enumerate(self.maze.survivors):
            disc = (cx - s['x']) ** 2 + (cy - s['y']) ** 2 <= SURVIVOR_RADIUS ** 2
            self.occ |= disc
            self.survivor_id[disc] = k
        self.truth = np.where(self.occ, 100, 0).astype(np.int8)
        self.bounds = self.maze.bounds()
        self.rooms = []
        for k in range(len(self.maze.rooms)):
            xa, xb, ya, yb = self.maze.room_box(k)
            self.rooms.append(((cx >= xa) & (cx <= xb) & (cy >= ya) & (cy <= yb)) & ~self.occ)
        xa, xb, ya, yb = self.bounds
        self.arena = (cx >= xa) & (cx <= xb) & (cy >= ya) & (cy <= yb)
        from scipy import ndimage
        self.clearance = ndimage.distance_transform_edt(~self.occ) * res - res / 2

    def sense(self, grid: np.ndarray, x: float, y: float, yaw: float, hfov: float,
              lidar: bool) -> set[int]:
        """Reveal truth into `grid` from the camera (and LiDAR). Returns survivor ids hit."""
        hits = set()
        views = [(hfov, True)] + ([(2 * math.pi, False)] if lidar else [])
        for fov, camera in views:
            rows, cols = ray_cells(self.truth, self.info, x, y, yaw, fov, SENSOR_RANGE)
            if camera:
                d = np.hypot(self.info.origin_x + (cols + 0.5) * self.info.resolution - x,
                             self.info.origin_y + (rows + 0.5) * self.info.resolution - y)
                near = d < CAMERA_MIN_RANGE
                rows, cols = rows[~near], cols[~near]
                ids = self.survivor_id[rows, cols]
                hits |= {int(i) for i in ids[ids >= 0]}
            grid[rows, cols] = self.truth[rows, cols]
        return hits


def run(seed: int, use_lidar: bool = True, params: ExplorerParams | None = None,
        speed: float = 0.5, time_limit: float = 1800.0, record: bool = False) -> SimResult:
    """Simulate one full mission."""
    t_wall = time.time()
    p = params or ExplorerParams()
    w = World(seed)
    info = w.info
    grid = np.full(w.truth.shape, -1, np.int8)
    cov = Coverage(info)
    ex = ExplorationPlanner(p)
    mission = Mission()
    res = SimResult(seed, use_lidar, rooms=len(w.rooms), survivors=len(w.maze.survivors))
    x, y, yaw, t = 0.0, 0.0, 0.0, 0.0
    home = (0.0, 0.0)
    seen_surv: set[int] = set()
    max_yaw_rate = 0.8
    path: list[tuple[float, float]] = []
    goal_yaw = None
    next_replan = 0.0
    look_left = 2 * math.pi
    home_costs, home_costs_t = None, -math.inf
    fence = (w.bounds[0] + p.guarded_radius, w.bounds[1] - p.guarded_radius,
             w.bounds[2] + p.guarded_radius, w.bounds[3] - p.guarded_radius)

    def sense() -> None:
        nonlocal seen_surv
        seen_surv |= w.sense(grid, x, y, yaw, p.camera_hfov, use_lidar)
        cov.update(grid, x, y, yaw, p.camera_hfov, p.coverage_range)

    def turn_towards(target: float) -> None:
        nonlocal yaw
        err = math.remainder(target - yaw, 2 * math.pi)
        yaw += max(-max_yaw_rate * DT, min(max_yaw_rate * DT, err))

    def step_along(goal_yaw_now: float | None) -> bool:
        """Move DT along `path`; returns True when the path end is reached."""
        nonlocal x, y
        dist = speed * DT
        while path and dist > 0:
            gx, gy = path[0]
            d = math.hypot(gx - x, gy - y)
            if d <= dist:
                res.path_length += d
                x, y = gx, gy
                dist -= d
                path.pop(0)
            else:
                x += (gx - x) * dist / d
                y += (gy - y) * dist / d
                res.path_length += dist
                dist = 0
        if path:
            heading = math.atan2(path[0][1] - y, path[0][0] - x)
            near_end = math.hypot(path[-1][0] - x, path[-1][1] - y) < 1.0
            turn_towards(goal_yaw_now if (use_lidar and near_end and goal_yaw_now is not None)
                         else heading)
        return not path

    def check_cell(pg) -> None:
        r, c = world_to_cell(info, x, y)
        if w.clearance[r, c] < p.guarded_radius - 1e-6:
            res.violations.append(f't={t:.0f}s clearance {w.clearance[r, c]:.2f} m')
        if pg is not None and pg.blocked[r, c] and not near_start(pg):
            res.violations.append(f't={t:.0f}s entered a blocked/unknown cell at ({x:.2f},{y:.2f})')
        res.min_clearance = min(res.min_clearance, float(w.clearance[r, c]))

    start_cell = {'xy': None}

    def near_start(pg) -> bool:
        # The first point of a path may be the drone's snapped-out-of-inflation start.
        return start_cell['xy'] is not None and math.hypot(x - start_cell['xy'][0],
                                                           y - start_cell['xy'][1]) < 0.5

    mission.handle(Trigger.START)
    sense()
    pg_now = None
    while t < time_limit + 600 and mission.state not in (State.DONE, State.ABORT):
        st = mission.state
        if st == State.TAKEOFF:
            t += 5.0                      # climb to 1.2 m at 0.3 m/s (+ margin)
            mission.handle(Trigger.ALTITUDE_REACHED)
            continue
        if st == State.EXPLORE and look_left > 0:
            turn = min(max_yaw_rate * DT, look_left)
            yaw += turn
            look_left -= turn
            t += DT
            sense()
            continue
        if st == State.EXPLORE:
            if not (fence[0] <= x <= fence[1] and fence[2] <= y <= fence[3]):
                mission.handle(Trigger.GEOFENCE_BREACH)
                break
            if t >= next_replan or not path:
                g = outside_blocked(grid, w)
                d = ex.replan(g, info, (x, y, yaw), t, cov.seen)
                next_replan = t + 1.0
                if d.done:
                    mission.handle(Trigger.EXPLORATION_DONE)
                    path = []
                    continue
                if d.goal is not None and d.plan is not None and d.plan.success:
                    path = list(d.plan.path)
                    goal_yaw = d.goal.yaw
                    start_cell['xy'] = path[0]
                    pg_now = build_planning_grid(clear_unknown_around(g, info, (x, y), p.clear_radius),
                                                 info, p.guarded_radius, p.safety_margin,
                                                 p.wall_cost_range)
                # Return budget (Dijkstra from home, refreshed every 10 s, or at once when the
                # drone is in a cell the last search did not reach).
                r, c = world_to_cell(info, x, y)
                if (t - home_costs_t > 10.0 or home_costs is None
                        or not np.isfinite(home_costs[r, c])):
                    hpg = build_planning_grid(clear_unknown_around(g, info, home, p.clear_radius),
                                              info, p.guarded_radius, p.safety_margin,
                                              p.wall_cost_range)
                    home_costs, home_costs_t = dijkstra_costs(hpg, home, p.wall_cost_weight), t
                hc = home_costs[r, c]
                if not np.isfinite(hc):      # e.g. drone inside inflation: straight line x 2
                    hc = 2.0 * math.hypot(x - home[0], y - home[1])
                if should_return(t, time_limit, hc, speed, 1.5, LANDING_TIME, 120.0):
                    mission.handle(Trigger.BUDGET_LOW)
                    path = []
                    continue
            if path:
                if step_along(goal_yaw):
                    ex.goal_reached()
                    # turn to the viewpoint heading on arrival
                    while goal_yaw is not None and abs(math.remainder(goal_yaw - yaw, 2 * math.pi)) > 0.05:
                        turn_towards(goal_yaw)
                        t += DT
                        sense()
            else:
                turn_towards(yaw + 0.5)       # nothing planned: look around
            t += DT
            sense()
            check_cell(pg_now)
        elif st == State.RETURN_HOME:
            if math.hypot(x - home[0], y - home[1]) <= 0.15:
                mission.handle(Trigger.AT_HOME)
                continue
            if not path or t >= next_replan:
                g = outside_blocked(grid, w)
                gg = clear_unknown_around(g, info, (x, y), p.clear_radius)
                gg = clear_unknown_around(gg, info, home, p.clear_radius)
                pg_now = build_planning_grid(gg, info, p.guarded_radius, p.safety_margin,
                                             p.wall_cost_range)
                r = plan(pg_now, (x, y), home, wall_cost_weight=p.wall_cost_weight)
                next_replan = t + 1.0
                if not r.success:
                    res.violations.append(f'no path home: {r.reason}')
                    mission.handle(Trigger.ABORT)
                    break
                path = list(r.path) + [home]
                start_cell['xy'] = path[0]
            step_along(None)
            t += DT
            sense()
            check_cell(pg_now)
        elif st == State.LAND:
            t += 5.0
            mission.handle(Trigger.LANDED)
        if record:
            res.track.append((x, y))

    res.state = mission.state.value
    res.time = t
    res.home_error = math.hypot(x - home[0], y - home[1])
    res.survivors_seen = len(seen_surv)
    seen = cov.seen
    res.rooms_seen = sum(1 for room in w.rooms
                         if (seen & room).sum() >= 0.95 * room.sum())
    true_free = w.arena & ~w.occ
    res.coverage = float((seen & true_free).sum()) / max(1, int(true_free.sum()))
    res.wall_time = time.time() - t_wall
    if record:
        res.map, res.seen, res.world = grid, seen, w
    return res


def outside_blocked(grid: np.ndarray, w: World) -> np.ndarray:
    """Map copy with everything outside the arena marked occupied (never explore outside)."""
    g = grid.copy()
    g[~w.arena] = 100
    return g


def parse_seeds(text: str) -> list[int]:
    """'1-50' or '3,5,7' -> list of ints."""
    out = []
    for part in text.split(','):
        if '-' in part:
            a, b = part.split('-')
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def main() -> int:
    """Run missions over seeds and print a summary."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--seeds', default='1-10')
    ap.add_argument('--lidar', default='true')
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--plot', help='PNG of the last seed: map, coverage and track')
    args = ap.parse_args()
    lidar = args.lidar.lower() == 'true'
    results = []
    for seed in parse_seeds(args.seeds):
        r = run(seed, lidar, record=bool(args.plot))
        results.append(r)
        if not args.quiet or not r.ok:
            print(f'seed {seed:3d} lidar={lidar!s:5} {r.state:5} {"OK  " if r.ok else "FAIL"} '
                  f'time {r.time:6.0f} s  path {r.path_length:6.1f} m  rooms {r.rooms_seen}/{r.rooms} '
                  f'survivors {r.survivors_seen}/{r.survivors}  coverage {100 * r.coverage:5.1f}%  '
                  f'home {r.home_error:.2f} m  min clear {r.min_clearance:.2f} m  '
                  f'({r.wall_time:.0f} s wall)' + (f'  {r.violations[:2]}' if r.violations else ''),
                  flush=True)
    ok = sum(r.ok for r in results)
    print(f'SUMMARY {ok}/{len(results)} OK; mean time {np.mean([r.time for r in results]):.0f} s, '
          f'mean coverage {100 * np.mean([r.coverage for r in results]):.1f}%')
    if args.plot and results:
        plot(results[-1], args.plot)
    return 0 if ok == len(results) else 1


def plot(r: SimResult, out: str) -> None:
    """Map (grey unknown, black walls), seen cells (green) and the track (blue)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    info = r.world.info
    img = np.where(r.map < 0, 0.6, np.where(r.map >= 50, 0.0, 1.0))
    rgb = np.dstack([img, img, img])
    rgb[r.seen] = [0.75, 1.0, 0.75]
    ext = (info.origin_x, info.origin_x + info.cols * info.resolution,
           info.origin_y, info.origin_y + info.rows * info.resolution)
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(rgb, origin='lower', extent=ext)
    xs, ys = zip(*r.track)
    ax.plot(xs, ys, '-', color='tab:blue', lw=1)
    for s in r.world.maze.survivors:
        ax.plot(s['x'], s['y'], 'o', color='orange', ms=6)
    ax.set_title(f'seed {r.seed}: {r.state}, {r.time:.0f} s, rooms {r.rooms_seen}/{r.rooms}, '
                 f'survivors {r.survivors_seen}/{r.survivors}')
    fig.savefig(out, dpi=100)


if __name__ == '__main__':
    sys.exit(main())
