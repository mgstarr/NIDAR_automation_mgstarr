# Phase 2 – Mapping with Ground-Truth Pose

Nov 2, 2026 · @Mahima

> Markdown export of the PDF *Phase 2 – Mapping with Ground-Truth Pose*.

**Goal:** a clean, live 2D occupancy map (`/map`), built while the position comes from
Gazebo's ground truth. This isolates mapping from localization errors.

## Before you start (entry checklist)

- [ ] Tag `phase-1` exists; `docs/handoff/phase-1.md` read
- [ ] `sim.launch.py` starts the maze and drone; the drone flies by keyboard
- [ ] `/camera/*`, `/scan` (when on), `/camera/imu`, `/range/*` and `/ground_truth/odom`
  publish at their rates
- [ ] Ground-truth layout files exist for generated mazes

## Working rules

Same as every phase: read `docs/PLAN.md` + this file; explain the approach and wait for
approval; one module at a time with tests; ask before inventing values; never rename
contract topics or parameters without approval.

## Tasks

1. `airmouse_bringup/launch/localization.launch.py`:
   - RTAB-Map in mapping mode with **external odometry** from `/ground_truth/odom`
     when `use_ground_truth` is true (RTAB-Map's own odometry stays off in this phase);
   - the `odom → base_link` transform comes from ground truth in this phase.
2. RTAB-Map grid settings in `airmouse_bringup/config/rtabmap.yaml`:
   - 2D grid built from depth points in a height band of 0.2–2.2 m (`grid_height_band`);
   - also uses `/scan` when `use_lidar` is true;
   - resolution 0.05 m; depth range capped at 3 m; unknown cells stay unknown (-1);
   - publishes `/map` at 1–2 Hz.
3. **Map scoring tool** `tools/score_map.py`: compares a saved map with the generator's
   ground-truth layout (aligned by the known take-off pose). Reports wall-cell agreement
   and false walls.
4. **Map snapshot tool** `tools/save_map.py`: saves `/map` (image + YAML, same format as
   the ground-truth layout) into `src/airmouse_nav/test/data/maps/`, for offline tests in
   Phases 3 and 5.

## Tests

- `score_map.py` unit tests: a layout scored against itself is perfect; a map with an added
  false wall is detected
- `save_map.py` round-trip test: save, load, compare

## Gazebo check

Launch sim + localization with `use_ground_truth:=true`. Fly through a maze by
keyboard. RViz shows `/map` growing in real time with straight walls and open doorways.

## Exit criteria

- [ ] `/map` updates at 1–2 Hz during flight
- [ ] Wall-cell agreement ≥ 90% on 3 mazes (initial target; adjust with the team)
- [ ] No false walls inside corridors or doorways
- [ ] At least 5 saved maps from different seeds, with LiDAR on and off, stored in
  `test/data/maps/`
- [ ] Handoff note written, committed, tagged `phase-2`

## Handoff to Phase 3

Write `docs/handoff/phase-2.md` with:

- the launch commands for sim + mapping;
- the saved map file format (fields, resolution, origin convention) and the list of saved
  maps;
- map scores per maze;
- RTAB-Map parameters changed from defaults and why;
- deviations and open issues.

Phase 3 receives: a live `/map` in simulation and a set of saved maps for offline planner
tests.

**Start prompt for Phase 3** (paste into Claude Code):

```
Read docs/PLAN.md, docs/phases/phase-3.md and docs/handoff/phase-2.md.
Run the Phase 3 entry checklist and report the result.
Then explain your approach for Phase 3 and wait for my approval.
```
