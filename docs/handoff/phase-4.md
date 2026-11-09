# Handoff: Phase 4 → Phase 5

Phase 4 (Pure Pursuit Controller) delivers the chain `/nav/path` → `controller_node` →
`/nav/cmd_vel_raw` → `safety_hook_node` → `/cmd_vel` → sim adapter. RViz goals are planned
(Phase 3) and flown; take-off and landing are paths.

## Exit criteria

- [x] All controller tests pass (19 pure-pursuit tests, 13 safety-hook tests, 1 clearance test;
  321 of 322 repo tests pass, see the runtime note under open issues)
- [x] 20 goals across 3 mazes reached with no wall contact: **21/21** (automated, owner
  approved; table below)
- [x] Minimum wall clearance always above `guarded_radius` (0.30 m): **0.328 m** (worst of 3 mazes)
- [x] Take-off to 1.2 m (measured 1.23 m) and landing (0.06–0.07 m = resting on the floor) work
  through paths
- [ ] Owner's visual RViz check
- [ ] Tagged `phase-4`

## Commands

Inside the dev container (`/ws`), after `colcon build --symlink-install && source install/setup.bash`:

```bash
# Terminal 1: sim + RViz            Terminal 2: mapping (ground truth)
ros2 launch airmouse_bringup sim.launch.py seed:=3 use_ground_truth:=true
ros2 launch airmouse_bringup localization.launch.py use_ground_truth:=true
# Terminal 3: goal planner + controller + safety hook (same seed: the geofence comes from it)
ros2 launch airmouse_bringup navigation.launch.py use_sim_time:=true seed:=3
# Terminal 4: take off, (click goals in RViz), land
python3 tools/fly_path.py takeoff
python3 tools/fly_path.py land
# Optional: minimum wall clearance during the run (Ctrl+C prints the summary)
python3 tools/clearance.py --layout src/airmouse_description/worlds/generated/seed_3_plain

# Automated exit test: 7 goals on each of 3 mazes, headless (~10 min), results in /tmp/goal_batch.log
bash tools/batch/goal_batch.sh       # one maze: tools/batch/goal_run.sh <seed> <goals> <texture> <lidar>
```

Do not run `teleop_twist_keyboard` together with the navigation launch: both publish `/cmd_vel`.

## Public API: `airmouse_nav.core.pure_pursuit` (no ROS)

```python
pp = PurePursuit(PursuitParams(...))      # fields = controller.yaml values (yaw tolerance in rad)
pp.set_path([(x, y, z), ...], goal_yaw)   # map frame; goal_yaw None = keep heading at the end
pp.clear()                                 # drop the path (zero command)
cmd = pp.update(Pose(x, y, z, yaw), dt)    # Command(vx, vy, vz, yaw_rate, reached, cross_track, active)
world_to_body(vx, vy, yaw) / body_to_world(vx, vy, yaw)
is_stale(now, pose_time, path_time, pose_timeout, path_timeout, following) -> 'pose' | 'path' | None
```

- **Path z** is the target altitude (P-control `altitude_gain`, limited to
  `max_vertical_speed`), taken at the look-ahead point. While the altitude error is above
  0.25 m the drone does not move sideways, but it holds the path's x, y. So take-off is
  `[(x, y, z_now), (x, y, flight_altitude)]` and landing is `[(x, y, z_now), (x, y, 0)]`.
- **Final yaw**: the yaw of the path's **last pose** (`controller_node` ignores an all-zero
  quaternion). Within `lookahead_max` of the end the drone turns to it; elsewhere it faces the
  direction of travel. A goal is reached when within `goal_tolerance_xy` (horizontally and
  vertically) and `goal_tolerance_yaw`.
- **After a goal** the drone **holds** the goal point (gain 1.0 /s, at most 0.2 m/s, plus
  altitude), until a new path arrives. After a landing goal (z < 0.1 m) the command is zero.
- **Stale data:** pose older than `pose_timeout` → zero. Path older than `path_timeout` **while
  following** → path dropped, zero. Anyone who publishes paths (Phase 5) must republish at
  least every 2 s while the drone flies. A reached goal does not go stale.
- **Corners** sharper than 20°: the look-ahead point stops at the corner, and the speed
  is limited so the corner can be taken with ≤ 0.05 m overshoot.
- `controller_node` also publishes `/nav/controller_state` (`std_msgs/String`: `idle`,
  `following`, `reached`, `stale:pose`, `stale:path`). Phase 5 can use it to know when a
  goal is reached.

## Parameters (`config/controller.yaml`, shared `/**`)

PLAN.md defaults kept: `flight_altitude 1.2`, `max_speed 0.6`, `max_accel 0.5`,
`max_yaw_rate 0.8`, `lookahead_min/max 0.3/0.8`, `goal_tolerance_xy 0.15`,
`goal_tolerance_yaw 10°`, `pose_timeout 0.3`, `path_timeout 2.0`, `control_rate 30`.

Added:

| Parameter | Value | Why |
|---|---|---|
| `yaw_gain` | 1.5 /s | heading P-gain (owner approved) |
| `altitude_gain` | 1.0 /s | altitude P-gain (owner approved) |
| `max_vertical_speed` | 0.3 m/s | climb/descent limit (owner approved) |
| `max_yaw_accel` | 0.5 rad/s² | **Measured:** in Gazebo a sudden yaw-rate reversal of 0.6 rad/s drops the drone 0.3 m, and one of 0.8 rad/s drops it to the floor (rotor saturation). Ramps of 0.5–1.0 rad/s² keep altitude. Half the safe value. |

`explorer.yaml` gained a `safety_hook_node` section with `arena_bounds [xmin, xmax, ymin, ymax]`.
`navigation.launch.py seed:=N` overrides it from the generated maze; on site it must be measured.
`goal_planner_node` gained `replan_period` (1.0 s) and reads `flight_altitude` and
`goal_tolerance_xy`.

Internal constants (in `pure_pursuit.py`, documented there): corner angle 20°, corner
overshoot 0.05 m, braking planned at 50% of `max_accel` (covers the 0.3–0.4 s response lag of
the Gazebo plugin), hold gain 1.0 /s and 0.2 m/s, climb-first threshold 0.25 m.

## Measured results

**Unit tests** (point mass with a 0.35 s velocity lag, 0.5 m/s): maximum cross-track error
0.000 m on the straight path and 0.063 m on the L-turn, U-turn and zigzag. Stops 0.14 m from
the goal (inside the 0.15 m tolerance).

**Gazebo, automated** (`tools/batch/goal_batch.sh`): take-off, a 360° look-around, 7 goals
picked at random from the live map (0.5–6 m path), then landing:

| Maze | Goals reached | Min wall clearance | Tracking error p95 / mean (max) |
|---|---|---|---|
| seed 1, plain, LiDAR | 7/7 | 0.328 m | 0.143 / 0.041 m (0.249) |
| seed 3, textured, LiDAR | 7/7 | 0.363 m | 0.116 / 0.030 m (0.202) |
| seed 5, plain, camera only | 7/7 | 0.431 m | 0.023 / 0.008 m (0.108) |

Tracking error is measured against the latest `/nav/path`, which is replanned every second.
Its maximum includes moments right after a replan, when the new path starts at a cell
centre. Wall clearance is measured independently from ground truth (`tools/clearance.py`:
exact distance to wall cells and survivor discs).

## Deviations and open issues

- **The Gazebo drone does not hold position or altitude on its own** (velocity control only).
  Hence the position hold after a goal, during climbs, and the altitude P-control.
- **Yaw reversals made the sim drone fall** before `max_yaw_accel` existed. A real flight
  controller should cope, but keep the slew limit.
- **RTAB-Map adds map data only when the drone moves or turns.** After take-off, the map
  around the drone is tiny until it turns. The test driver turns 360° first; **Phase 5's
  explorer should start with a look-around.**
- **Unknown cells within 0.5 m of the drone count as free when planning**
  (`clear_unknown_around` in `goal_planner_node`). The camera cannot see within 0.3 m of
  itself, so on camera-only maps the drone's own spot was "unknown" and every plan failed with
  `start_blocked`. Phase 5 should do the same, ideally for the whole flown track.
- **Unknown cells are blocked but no longer inflated** (`grid_utils.build_planning_grid`,
  owner approved during the visual check). Inflating unknown space by 0.40 m left only 70
  plannable cells near the seed-3 entry, none reachable from the drone (0/14 sampled goals).
  Without it, 79/79 were reachable. Only occupied cells get the `guarded_radius + safety_margin`
  band. Hidden walls are inflated once seen: the camera faces the direction of travel and paths
  are replanned every 1 s.
- **Planned paths start at the drone's position.** When the start is inside inflation, the
  goal planner adds the drone's position as the first point, so the move out of the band is
  part of the path.
- Seed 5 (camera only): the reachable map stayed small, so its 7 goals were short hops
  (0.5–1 m) near the entry. Seeds 1 and 3 had 2–6 m goals through corridors and turns.
- An earlier run (before the position hold) had one stuck goal pressed against a wall
  (clearance exactly 0.300 m); it did not recur in the final 21 goals.
- `test_astar.py::test_runtime_under_50ms` fails **on battery power only** (58 ms; CPU
  throttled). It measures 31 ms on mains power.
- New tools: `tools/fly_path.py`, `tools/clearance.py`, `tools/batch/goal_driver.py`,
  `goal_run.sh`, `goal_batch.sh`; `kill_sim.sh` also stops the navigation nodes, tools and
  `ros2 bag`.
