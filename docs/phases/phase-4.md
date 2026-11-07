# Phase 4 – Pure Pursuit Controller

Nov 2, 2026 · @Mahima

> Markdown export of the PDF *Phase 4 – Pure Pursuit Controller*.

**Goal:** the drone smoothly follows any A* path, holds altitude and heading, takes off and lands
through paths, and hovers safely when data goes stale.

## Before you start (entry checklist)

- [ ] Tag `phase-3` exists; `docs/handoff/phase-3.md` read
- [ ] `goal_planner_node` turns RViz goals into `/nav/path` on the live `/map`
- [ ] Planner tests pass

## Working rules

Same as every phase: read `docs/PLAN.md` + this file; explain the approach and wait for
approval; one module at a time with tests; ask before inventing values; never rename
contract topics or parameters without approval.

## Tasks

1. `airmouse_nav/core/pure_pursuit.py` (no ROS imports; works in the `map` frame, 2D + altitude):
   - find the look-ahead point on the path; look-ahead distance grows with speed between
     `lookahead_min` and `lookahead_max`;
   - command velocity toward that point (holonomic: the drone can move sideways);
   - **speed profile:** slow down for sharp turns (limit by path curvature) and near the goal;
     respect `max_speed` and `max_accel`;
   - **yaw:** P-controller toward the target heading (direction of travel, or the goal pose's
     yaw at the end), limited by `max_yaw_rate`;
   - **altitude:** P-controller toward the path's z. Take-off and landing are paths with
     z = `flight_altitude` or z = 0;
   - goal reached within `goal_tolerance_xy` and `goal_tolerance_yaw`.
2. `airmouse_nav/nodes/controller_node.py`:
   - reads `/nav/path` and the pose from TF (`map → base_link`);
   - runs at `control_rate` (30 Hz); rotates the velocity into the body frame; publishes
     `/nav/cmd_vel_raw`;
   - **stale data:** pose older than `pose_timeout` or path older than `path_timeout` → zero
     velocity (hover) and a warning.
3. `airmouse_nav/nodes/safety_hook_node.py`:
   - copies `/nav/cmd_vel_raw` to `/cmd_vel`;
   - clamps speeds to the configured maximums;
   - **geofence:** removes any velocity component that would leave `arena_bounds`;
   - clearly marked as the slot for the future CBF filter.
4. Connect `goal_planner_node` → `controller_node` → `safety_hook_node` → sim adapter, so
   RViz goals are flown.
5. `tools/clearance.py`: from ground-truth pose and layout, report the minimum wall clearance
   during a run.

## Tests

- A small point-mass simulator in the test folder: follow a straight path, an L-turn and a
  U-turn. Cross-track error under 0.10 m at 0.5 m/s; stops within tolerance at the goal
- Stale pose and stale path each give a zero command
- Geofence removes outward velocity at the boundary and keeps inward velocity
- Body-frame rotation is correct for several yaw angles

## Gazebo check

Click goals in RViz. The drone flies the drawn path through corridors and doorways, turns
smoothly, stops at the goal and hovers. Take-off and landing work through paths.

## Exit criteria

- [ ] All controller tests pass
- [ ] 20 RViz goals across 3 mazes reached with no wall contact
- [ ] Minimum wall clearance always above `guarded_radius`
- [ ] Take-off to 1.2 m and landing work
- [ ] Handoff note written, committed, tagged `phase-4`

## Handoff to Phase 5

Write `docs/handoff/phase-4.md` with:

- the public API of `pure_pursuit`, and how path z and the final pose yaw are used;
- tuned controller parameters (final values) and why;
- measured tracking error and wall clearance;
- how to launch the goal → plan → fly chain;
- deviations and open issues.

Phase 5 receives: a verified chain from `/nav/path` to drone motion, so the explorer only has to
publish paths.

**Start prompt for Phase 5** (paste into Claude Code):

```
Read docs/PLAN.md, docs/phases/phase-5.md and docs/handoff/phase-4.md.
Run the Phase 5 entry checklist and report the result.
Then explain your approach for Phase 5 and wait for my approval.
```
