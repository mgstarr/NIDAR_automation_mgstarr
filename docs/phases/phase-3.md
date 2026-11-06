# Phase 3 – A* Path Planner

Nov 2, 2026 · @Mahima

> Markdown export of the PDF *Phase 3 – A\* Path Planner*.

**Goal:** a fast, safe, deterministic planner on the 2D grid, written as a library the explorer
will call, plus a test node so paths can be checked in RViz.

## Before you start (entry checklist)

- [ ] Tag `phase-2` exists; `docs/handoff/phase-2.md` read
- [ ] Sim + mapping launch gives a live `/map` with `use_ground_truth:=true`
- [ ] At least 5 saved maps in `src/airmouse_nav/test/data/maps/`

## Working rules

Same as every phase: read `docs/PLAN.md` + this file; explain the approach and wait for
approval; one module at a time with tests; ask before inventing values; never rename
contract topics or parameters without approval.

## Tasks

1. `airmouse_nav/core/grid_utils.py` (no ROS imports):
   - convert an occupancy grid (data + resolution + origin) to a numpy array; convert between
     metres and cells;
   - inflation: block every cell within `guarded_radius + safety_margin` of an obstacle;
   - wall cost: from a distance transform, a cost that rises near walls, so paths prefer
     corridor centres (weight `wall_cost_weight`);
   - unknown cells count as blocked.
2. `airmouse_nav/core/astar.py` (no ROS imports):
   - 8-connected search with an octile-distance heuristic (straight + diagonal moves);
   - cost = step length + `wall_cost_weight` × wall cost;
   - path simplification: drop points that have a clear line of sight to a later point;
   - returns a result object: success flag, path in metres, length, cost and failure reason
     (start blocked, goal blocked, no path). Normal failures never raise;
   - if the start cell is inside inflation (drone near a wall), plan from the nearest free cell;
   - the docstring states the public function signature; Phase 5 will call it directly.
3. `airmouse_nav/nodes/goal_planner_node.py` (test tool): a goal from RViz "2D Goal Pose" →
   A* on the live `/map` → publish `/nav/path`.
4. `tools/plot_plan.py`: plot a saved map with its inflation, wall cost and a planned path.

## Tests

- Synthetic grids: straight corridor, L-turn, dead end, unreachable goal, goal inside a wall,
  start inside inflation
- The path never crosses a blocked or unknown cell
- In a 1 m corridor the path stays within ±0.1 m of the centre line
- Same input always gives the same path
- On the saved maps: random reachable goal pairs all succeed
- Runtime under 50 ms on a 300 × 300 grid on the dev machine

## Gazebo check

With sim + mapping running, click goals in RViz. The drawn path is centred, avoids walls and
replans for each new goal. The drone does not move yet.

## Exit criteria

- [ ] All planner tests pass
- [ ] Runtime target met
- [ ] RViz goal test works on 3 mazes
- [ ] Handoff note written, committed, tagged `phase-3`

## Handoff to Phase 4

Write `docs/handoff/phase-3.md` with:

- the public API of `grid_utils` and `astar` (signatures, inputs, outputs, failure reasons);
- how to run `goal_planner_node` and `plot_plan.py`;
- measured runtimes and any parameter changes;
- deviations and open issues.

Phase 4 receives: a planner library and a node that turns RViz goals into `/nav/path`, ready to
be followed by the controller.

**Start prompt for Phase 4** (paste into Claude Code):

```
Read docs/PLAN.md, docs/phases/phase-4.md and docs/handoff/phase-3.md.
Run the Phase 4 entry checklist and report the result.
Then explain your approach for Phase 4 and wait for my approval.
```
