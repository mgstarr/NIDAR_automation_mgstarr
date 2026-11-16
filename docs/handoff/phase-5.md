# Handoff: Phase 5 → Phase 6

Phase 5 (Exploration and Mission State Machine) delivers the autonomous mission on ground-truth
pose: take-off, 360° look-around, frontier / next-best-view exploration, return home, landing,
with abort and recall.

## Exit criteria

- [x] All tests pass (unit tests: frontier, coverage, viewpoint, mission table, return budget)
- [~] `grid_sim` on 50 seeds: **every mission completes and every survivor is seen in 100/100
  runs**; the strict "every room ≥ 95% seen" check passes 48/50 with LiDAR and 33/50 camera
  only (table below; open issue 1)
- [~] 10 of 10 random mazes in Gazebo on ground truth: seed 1 completed 3 times (table below);
  the full 10-maze batch was cut short by a computer restart and is **re-run together with the
  Phase 6 batch** (results appended to this note when it finishes)
- [x] Every survivor fell inside camera coverage in every completed run
- [ ] Abort and recall mid-mission in Gazebo: in the same batch (state-machine table tested)
- [x] Mission times recorded (baseline below)
- [x] Owner's visual check: the owner ran the mission demo; the owner asked Claude to verify
  the result, which the Gazebo runs below do
- [ ] Tagged `phase-5` (after the batch)

## Commands

```bash
# Full mission, one command (Gazebo + RViz + mapping + explorer + controller + safety + detector)
ros2 launch airmouse_bringup mission.launch.py use_ground_truth:=true seed:=1
#   options: use_lidar:=false  wall_texture:=textured  gui:=false rviz:=false
#            real_time_factor:=2.0  detection:=false
ros2 service call /mission/start std_srvs/srv/Trigger
ros2 service call /mission/return_home std_srvs/srv/Trigger    # emergency recall
ros2 service call /mission/abort std_srvs/srv/Trigger          # hover, then land in place
ros2 topic echo /mission/state

# Fast simulator, no Gazebo (same core code as the explorer node)
python3 tools/grid_sim.py --seeds 1-50 [--lidar false] [--plot /tmp/seed.png]
bash tools/batch/grid_sim_batch.sh 1 50 true 12      # parallel; results in /tmp/gridsim_true.log

# One headless Gazebo mission with scoring; log in /tmp/mission_<seed>_<lidar>.log
bash tools/batch/mission_run.sh <seed> <lidar> [texture] [rtf] [timeout_s]
python3 tools/mission_report.py --bag /tmp/mission_bag_<seed>_<lidar> --seed <seed>
```

## Baseline results

**grid_sim, 50 seeds per mode** (kinematic drone at 0.5 m/s; camera 87°, 0.3–3.0 m; LiDAR 3.0 m):

| | LiDAR on | Camera only |
|---|---|---|
| Missions DONE (explored, at home ≤ 0.15 m, landed) | 50/50 | 50/50 |
| All survivors hit by a camera ray | 50/50 | 50/50 |
| Arena free cells seen | mean 99.8% (min 99.3%) | mean 99.7% (min 99.1%) |
| Every room ≥ 95% of its seeable cells | 48/50 | 33/50 |
| Mission time | mean 674 s (590–795) | mean 829 s (722–968) |
| Path length | mean 284 m | mean 289 m |
| Min wall clearance | ≥ 0.40 m | ≥ 0.39 m |
| Entered a blocked/unknown cell | never | never |

**Gazebo, ground truth, seed 1, LiDAR, plain walls** (`tools/mission_report.py`):

| Run | State | Flight time | Path | Survivors in coverage | Arena seen | Rooms | Home error | Min clearance |
|---|---|---|---|---|---|---|---|---|
| first full run | DONE | 1138 s | 325 m | 6/6 | 99.8% | 7/7 | 0.13 m | 0.300 m |
| with detector, CPU shared with a 10-job batch | DONE | 1307 s | 354 m | 6/6 (5/6 detected) | 99.8% | 7/7 | 0.11 m | 0.300 m (contact) |

Gazebo missions take about twice the grid_sim time: the real controller slows for corners and
turns to each viewpoint heading.

## Explorer parameters (`config/explorer.yaml`) and why

PLAN.md defaults kept: `replan_period 1.0`, `camera_hfov 87`, `coverage_range 2.5`,
`gain_weight 1.0`, `goal_switch_margin 0.2`, `stuck_timeout 10`, `mission_time_limit 1800`,
`return_reserve 120`, `return_safety_factor 1.5`, `localization_lost_timeout 3`.
Owner-approved additions: `min_cluster_size 6` cells (0.3 m), `landing_time 15` s,
`large_map_change 0.02`. Gain is in m² and cost in m (+ 0.2 m per radian of turn).

How the explorer differs from the phase document, and why:

- **Candidates need a line of sight.** Viewpoints are safe, reachable cells that can see a
  cluster anchor within `coverage_range` (one per direction sector and distance band out to
  2.5 m). Sampling by distance alone put candidates behind walls (zero gain) and left rooms
  under-covered.
- **Bounded search.** Candidates are scored in order of rising cost and the search stops when a
  full-field-of-view gain could no longer win. Same choice, about 5× less work (median replan
  ≈ 195 ms on a loaded dev machine, 70–90 ms unloaded with the older sampler).
- **A reached viewpoint is never taken again** (within 0.3 m and 30°), and cells nearer than the
  camera's 0.3 m minimum range give no gain. Without this the drone re-picked its own position
  forever (27 of 50 seeds stalled).
- **"Done" needs no clusters left, or clusters unreachable for `stuck_timeout`.** A momentary
  "no reachable viewpoint" (drone inside inflation on a noisy map) used to end exploration
  at 26% coverage in Gazebo. While waiting, the node turns the drone in place.
- **Look-around first:** four 90° turns after take-off (RTAB-Map adds data only when moving).
- **Never explore outside the arena:** cells outside `arena_bounds` count as occupied.
- **Unknown cells within 0.5 m of the drone count as free** when planning (camera blind zone).
- **Room metric:** a room is "seen" at ≥ 95% of its *seeable* cells: cells an ideal explorer
  could see when grown from the start under the same safety rules (`grid_sim.World.seeable`).

## Ground-truth dependencies (Phase 6 must replace each)

| Where | Dependency | Replacement |
|---|---|---|
| `sim.launch.py` | bridges Gazebo's pose to TF `odom → base_link` when `use_ground_truth` | EKF publishes it |
| `localization.launch.py` | RTAB-Map takes odometry from `/ground_truth/odom` | RTAB-Map odometry → EKF → `/odom` |
| `controller_node`, `safety_hook_node`, `explorer_node`, `goal_planner_node` | read TF `map → base_link` (indirect) | nothing to change: the TF source changes |
| `explorer_node` | assumes the pose is always valid | hover / ABORT on bad localization |
| `tools/takeoff.py`, `tools/survey_flight.py` | fly on `/ground_truth/odom` (test tools) | not part of the stack; use `fly_path.py` |
| `tools/clearance.py`, `mission_report.py`, `pose_error.py` | ground truth for **scoring only** | allowed |

## Open issues

1. **Rooms under 95%** (2/50 with LiDAR, 17/50 camera only, one or two rooms each): mostly the
   shadow behind a survivor seen from the door. Every survivor is still seen.
2. **Wall clearance is at the limit in Gazebo:** 0.300–0.301 m on the worst runs, and one
   contact while the CPU was shared with a batch. A Phase 4 recording showed a *planned* path
   0.24 m from a true wall near a wall end: the map can lack the last cells of a thin wall seen
   end-on. Phase 7 should measure this over 20+ mazes; knobs: `safety_margin`, corner speed, the
   RTAB-Map noise filter.
3. **Replan time** ≈ 0.2 s on the dev machine; the Raspberry Pi will need fewer candidates or a
   slower replan (Phase 7).
4. `tools/batch/mission_run.sh` polls `/mission/state` every 5 s, so logged state times are ± 5 s.
