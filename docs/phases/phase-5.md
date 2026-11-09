# Phase 5 – Exploration and Mission State Machine

Nov 2, 2026 · @Mahima

> Markdown export of the PDF *Phase 5 – Exploration and Mission State Machine*.

**Goal:** the drone explores the whole maze on its own, makes sure the camera has seen every
free area, returns to the entry and lands. Ground-truth pose is still used in this phase.

## Before you start (entry checklist)

- [ ] Tag `phase-4` exists; `docs/handoff/phase-4.md` read
- [ ] RViz goal → A* → controller → drone works with no wall contact
- [ ] Take-off and landing through paths work
- [ ] Saved maps available in `test/data/maps/`

## Working rules

Same as every phase: read `docs/PLAN.md` + this file; explain the approach and wait for
approval; one module at a time with tests; ask before inventing values; never rename
contract topics or parameters without approval.

## Tasks

1. `core/frontier.py`:
   - **map frontiers:** free cells next to unknown cells;
   - group into clusters with an 8-connected breadth-first search; drop clusters smaller than
     `min_cluster_size`;
   - each cluster reports its cells, centroid and size.
2. `core/coverage.py`:
   - a coverage grid with the same size and origin as `/map`;
   - each update, cast rays across `camera_hfov` (87°) from the current pose up to
     `coverage_range` (2.5 m); rays stop at walls; free cells a ray passes are marked "seen";
   - **view frontiers:** free cells not yet seen, clustered the same way.
3. `core/viewpoint.py`:
   - for each cluster, sample candidate positions in safe free space around it, each with a
     heading facing the cluster;
   - **gain** = target cells (unknown or unseen) visible from that pose, using the same ray
     casting;
   - **cost** = travel cost from the drone + a small turning cost; score = `gain_weight` ×
     gain − cost;
   - **one Dijkstra search from the drone per replan** costs every candidate at once; A*
     (Phase 3 library) is used only for the chosen goal;
   - **goal commitment:** keep the current goal until reached, invalid, or beaten by
     `goal_switch_margin`;
   - **stuck rule:** no progress for `stuck_timeout` → blacklist the goal and pick another.
4. `core/mission.py` (state machine):
   - IDLE → TAKEOFF (on `/mission/start`) → EXPLORE (altitude reached) → RETURN_HOME → LAND
     (at home) → DONE;
   - ABORT from any state on `/mission/abort`, stale localization
     (`localization_lost_timeout`) or geofence breach: hover, then land in place;
   - EXPLORE → RETURN_HOME when no map or view frontiers remain, OR when remaining time −
     (home travel time × `return_safety_factor` + landing time) < `return_reserve`. Home
     travel cost comes from a Dijkstra search from home, refreshed periodically;
   - `/mission/return_home` forces RETURN_HOME (emergency recall);
   - time limit = `mission_time_limit`; an optional `/battery` input is left for later.
5. **Heading modes:** `use_lidar` false → face direction of travel, turn to the viewpoint
   heading on arrival; `use_lidar` true → may start turning toward the viewpoint heading
   before arrival.
6. `nodes/explorer_node.py`: replans at `replan_period`, on goal reached, or on a large map
   change. Publishes `/nav/path`, `/nav/goal`, `/nav/frontiers`, `/nav/coverage`,
   `/mission/state`; serves the three mission services.
7. `launch/mission.launch.py`: sim + mapping + explorer + controller + safety hook + RViz in
   one command.
8. `tools/grid_sim.py`: a fast pure-Python simulator that reveals a ground-truth maze through
   a simulated camera (and LiDAR) and runs the explorer loop without Gazebo.

## Tests

- Frontier detection and clustering on hand-made grids
- Coverage ray casting stops at walls and marks the right cells
- The chosen viewpoint actually sees its cluster
- State-machine transition table: every state × trigger, including abort from each state
- Return-budget trigger fires at the right time
- `grid_sim` over 50 seeds: every room seen, ends at home, never enters blocked or unknown
  cells

## Gazebo check

`ros2 launch airmouse_bringup mission.launch.py use_ground_truth:=true`, then call
`/mission/start`. In RViz: frontiers shrink, the coverage layer fills, the drone visits every
room, flies home and lands; `/mission/state` ends at DONE.

## Exit criteria

- [ ] All tests pass, including `grid_sim` on 50 seeds
- [ ] 10 of 10 random mazes completed on ground truth, with and without LiDAR
- [ ] Every survivor model fell inside camera coverage in every run
- [ ] Abort and recall work mid-mission
- [ ] Mission times recorded as the baseline
- [ ] Handoff note written, committed, tagged `phase-5`

## Handoff to Phase 6

Write `docs/handoff/phase-5.md` with:

- how to run a full mission and the `grid_sim` tool;
- baseline results per maze: time, coverage, survivors seen, path length;
- final explorer parameter values and why;
- every place the stack depends on ground truth (Phase 6 must remove each one);
- deviations and open issues.

Phase 6 receives: a complete autonomous mission working on perfect positioning, and the list
of ground-truth dependencies to replace.

**Start prompt for Phase 6** (paste into Claude Code):

```
Read docs/PLAN.md, docs/phases/phase-6.md and docs/handoff/phase-5.md.
Run the Phase 6 entry checklist and report the result.
Then explain your approach for Phase 6 and wait for my approval.
```
