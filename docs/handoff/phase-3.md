# Handoff: Phase 3 → Phase 4

Phase 3 (A* Path Planner) delivers a planner library (`core/grid_utils.py`, `core/astar.py`),
a test node that turns RViz goals into `/nav/path`, and a plotting tool.

## Exit criteria

- [x] All planner tests pass (29 planner tests; 289 in the repo)
- [x] Runtime target met: worst corner-to-corner plan 31 ms on the ≈ 310 × 310 saved maps
  (target < 50 ms), in plain Python (numba was approved as a fallback but is not needed)
- [ ] RViz goal test works on 3 mazes (owner)
- [ ] Tagged `phase-3`

## Commands

Inside the dev container (`/ws`), after `colcon build --symlink-install && source install/setup.bash`:

```bash
# Terminal 1: sim + RViz          Terminal 2: mapping
ros2 launch airmouse_bringup sim.launch.py seed:=3 use_ground_truth:=true
ros2 launch airmouse_bringup localization.launch.py use_ground_truth:=true
# Terminal 3: planner test node (reads planner.yaml)
ros2 launch airmouse_bringup navigation.launch.py use_sim_time:=true
# Terminal 4: take off and fly around a little by keyboard so the map fills in
python3 tools/takeoff.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link
# Then use "2D Goal Pose" in RViz: the blue "Planned path" appears; tick "Planning grid" to see
# the inflation. The node logs length, points, timing and failure reasons.

# Plot a plan on a saved map (PNG with map + inflation, wall cost, path)
python3 tools/plot_plan.py --map src/airmouse_nav/test/data/maps/seed5_lidar_plain.yaml \
    [--start X Y] [--goal X Y] --out plan.png

# Tests
pytest src tools -q
```

## Public API

### `airmouse_nav.core.grid_utils` (no ROS)

| Function / class | Inputs | Output |
|---|---|---|
| `GridInfo(resolution, origin_x, origin_y, rows, cols)` | origin = map-frame lower-left corner of cell (0, 0) | frozen dataclass |
| `grid_from_occupancy(data, width, height)` | `OccupancyGrid.data` (row-major, row 0 = lowest y) | int8 array (height, width); `ValueError` on size mismatch |
| `world_to_cell(info, x, y)` | metres | `(row, col)`, may be off-grid |
| `cell_to_world(info, row, col)` | cell | `(x, y)` of the cell centre |
| `in_bounds(info, row, col)` | cell | bool |
| `obstacle_mask(grid)` | int8 grid | bool: value ≥ 50 **or unknown (-1)** |
| `clearance_map(obstacles, resolution)` | bool mask | metres from each cell centre to the nearest obstacle cell edge |
| `build_planning_grid(grid, info, guarded_radius=0.30, safety_margin=0.10, wall_cost_range=0.5)` | grid + info | `PlanningGrid` |

`PlanningGrid` fields: `info`; `blocked` (bool: obstacle, unknown, or clearance <
`guarded_radius + safety_margin`); `wall_cost` (0..1: 1 at the inflation edge, falling linearly
to 0 at `wall_cost_range` from an obstacle; 1 on blocked cells); `clearance` (m);
`components` (int32 free-region ids, 0 = blocked); `inflation_radius`. Build it once per map
update and plan many times on it (≈ 3–5 ms to build on a 310 × 310 map).

### `airmouse_nav.core.astar` (no ROS)

```python
plan(pg: PlanningGrid, start_xy, goal_xy, wall_cost_weight=2.0, simplify=True,
     start_snap_radius=0.5) -> PlanResult
```

`PlanResult`: `success`, `path` (list of `(x, y)` map-frame metres; first = start cell centre
or snapped start, last = goal cell centre; empty on failure), `length` (m), `cost`, `reason`,
`start_snapped`, `expanded`. Never raises on normal failures:

| `reason` | When |
|---|---|
| `ok` | success |
| `start_blocked` | start off the grid, or no free cell within `start_snap_radius` (0.5 m) |
| `goal_blocked` | goal off the grid, or in an obstacle, unknown or inflated cell |
| `no_path` | start and goal in different free regions (instant check), or the search finds no route |

Search: 8-connected, octile heuristic, no corner cutting; moving into a cell costs
`step length × (1 + wall_cost_weight × wall_cost)`; fixed tie-breaking, so it is deterministic.
Simplification keeps a later point only if the straight segment crosses no blocked cell (exact
cell traversal) and is not closer to walls than the raw path (max wall cost not higher, + 0.05).

### `goal_planner_node`

Subscribes `/map`, `/goal_pose` (any frame with a TF to `map`; RViz uses `odom`); start pose =
TF `map → base_link`. Publishes `/nav/path` (`nav_msgs/Path`, frame `map`, one pose per point,
heading along the path; an empty path on failure) and `/nav/planning_grid` (debug, latched:
100 = blocked, 0–99 = wall cost). Parameters: `planner.yaml` + `map_frame`, `robot_frame`.

## Measured runtimes (dev machine, plain Python, best of 5)

Corner-to-corner of the largest free region on each saved map:

| Map | Size (cells) | Path | Plan time |
|---|---|---|---|
| `seed1_lidar_plain` | 310 × 310 | 27.6 m | 25.8 ms |
| `seed2_nolidar_textured` | 309 × 310 | 10.5 m | 7.1 ms |
| `seed3_lidar_textured` | 309 × 310 | 25.5 m | 26.6 ms |
| `seed4_nolidar_textured` | 309 × 311 | 23.6 m | 19.8 ms |
| `seed5_lidar_plain` | 310 × 309 | 32.9 m | 31.4 ms |
| `seed6_nolidar_plain` | 308 × 309 | 18.3 m | 16.3 ms |

Live (seed 3, headless, after a short survey): grid build 2–4 ms, A* 6–24 ms per goal.
30 random reachable pairs per saved map: all succeed and never touch a blocked cell.

## Parameter changes

- New parameter `wall_cost_range: 0.5` m in `planner.yaml` (owner approved): where the wall cost
  reaches 0 (the middle of a 1 m corridor).
- `guarded_radius 0.30`, `safety_margin 0.10`, `wall_cost_weight 2.0` as in PLAN.md.
- `start_snap_radius 0.5` m is a function argument (not a ROS parameter).

## Deviations and open issues

- **The free strip in a 1 m corridor is only 0.20 m wide** (0.5 m half-width − 0.40 m
  inflation). A drone more than 0.10 m off the centre line is "inside inflation"; the planner
  then starts from the nearest free cell (logged as "start moved out of inflation"). In the live
  test this happened on every goal. Phase 4's controller must track the centre within ±0.10 m,
  or the final guarded radius (PLAN.md open decision, Phase 4) needs a second look.
- **Survivors in 1 m corridors block them completely** (0.20 m radius + 0.40 m inflation).
  Correct for safety; Phase 5 should know a corridor can be closed by a person.
- **Right after take-off the map around the drone is mostly unknown**, so goals and even the
  start can be blocked until the drone has looked around. Phase 5 needs an initial look-around
  (yaw in place) before the first plan.
- Goals inside inflation are refused (`goal_blocked`), as the doc says (owner approved); Phase 5
  viewpoints must be picked from free cells.
- The wall cost is integrated along each step (`step × (1 + w × cost)`), our reading of "step
  length + weight × wall cost" that keeps the units consistent.
- Extra debug topic `/nav/planning_grid`; RViz layout gained "Planned path" and "Planning grid"
  (off by default) displays. `navigation.launch.py` now starts `goal_planner_node`.
- `tools/batch/kill_sim.sh` also stops `goal_planner_node`.
