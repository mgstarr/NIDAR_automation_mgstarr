# Phase 6 – Real Localization (RTAB-Map + EKF)

Nov 2, 2026 · @Mahima

> Markdown export of the PDF *Phase 6 – Real Localization (RTAB-Map + EKF)*.

**Goal:** remove ground truth. The drone estimates its own position from its sensors, exactly as
the real drone will, and still completes the full mission.

## Before you start (entry checklist)

- [ ] Tag `phase-5` exists; `docs/handoff/phase-5.md` read
- [ ] Full mission completes on ground truth, with and without LiDAR
- [ ] The handoff note lists every ground-truth dependency to replace

## Working rules

Same as every phase: read `docs/PLAN.md` + this file; explain the approach and wait for
approval; one module at a time with tests; ask before inventing values; never rename
contract topics or parameters without approval.

## Tasks

1. **Odometry from sensors (RTAB-Map):**
   - `use_lidar` true: ICP odometry from `/scan` (aligns each laser scan with the previous
     ones; cheap and robust);
   - `use_lidar` false: RGB-D visual odometry from the camera.
2. **EKF** (`robot_localization`, `airmouse_bringup/config/ekf.yaml`) fuses RTAB-Map odometry
   (x, y, yaw, velocities), the IMU (angular velocity, acceleration) and the downward
   rangefinder for height. A tiny adapter node turns `/range/down` into a z measurement. The
   EKF publishes `/odom` and `odom → base_link`; RTAB-Map publishes `map → odom` and corrects
   drift on loop closure.
3. Replace every ground-truth dependency listed in the Phase 5 handoff. With
   `use_ground_truth:=false`, ground truth is only recorded for scoring and never feeds the
   stack.
4. **Localization-loss handling:** odometry lost or its covariance too large → the mission
   state machine makes the drone hover; no recovery within `localization_lost_timeout` →
   ABORT.
5. `tools/pose_error.py`: compares the estimated pose with ground truth over a run and reports
   drift (absolute trajectory error).
6. Tune RTAB-Map and the EKF for the Raspberry Pi 5 budget: 424×240 at 15 Hz, limited feature
   count. Record CPU use per node.

## Tests

- Range-to-z adapter unit test
- Launch test: with ground truth off, every contract topic is alive and the TF chain
  `map → odom → base_link` is complete
- `pose_error.py` unit test on a synthetic trajectory

## Gazebo check

Full mission with `use_ground_truth:=false`, with and without LiDAR, on plain and textured
walls. In RViz the map stays straight, walls are not doubled after loops, and the drone still
returns to the entry.

## Exit criteria

- [ ] 10 of 10 missions complete with LiDAR (plain and textured walls)
- [ ] Missions without LiDAR complete on textured walls; plain-wall results documented honestly
- [ ] Position error under 0.10 m RMS (initial target)
- [ ] Map agreement still ≥ the Phase 2 target
- [ ] Localization-loss failsafe demonstrated (drop the camera or LiDAR topic mid-flight)
- [ ] Handoff note written, committed, tagged `phase-6`

## Handoff to Phase 7

Write `docs/handoff/phase-6.md` with:

- how to run SLAM-based missions in each mode;
- results per mode: success, pose error, map agreement, mission time;
- final RTAB-Map and EKF parameters, and CPU use per node;
- known weaknesses (for example visual-only on plain walls);
- deviations and open issues.

Phase 7 receives: a complete stack running without ground truth, ready for large-scale
benchmarking.

**Start prompt for Phase 7** (paste into Claude Code):

```
Read docs/PLAN.md, docs/phases/phase-7.md and docs/handoff/phase-6.md.
Run the Phase 7 entry checklist and report the result.
Then explain your approach for Phase 7 and wait for my approval.
```
