# Handoff: Phase 6 → Phase 7 (status: NOT closed)

Phase 6 (Real Localization) removes ground truth from the stack. With `use_ground_truth:=false`
the drone localizes with RTAB-Map odometry + EKF, and ground truth is recorded for scoring only.

**Status on 2026-11-17: LiDAR mode works end to end; camera-only mode does not yet; the
10-mission exit batch has not been run.** Do not tag `phase-6` until the batch is in.

## Exit criteria

- [ ] 10 of 10 missions complete with LiDAR (plain and textured walls): **1 of 1 so far**
  (seed 1, plain; table below). Batch: `tools/batch/phase56_batch.sh` (≈ 9 h, not yet run)
- [ ] Missions without LiDAR complete on textured walls: **no**. Visual odometry is lost within
  the first minute and the failsafe aborts (details below)
- [~] Position error under 0.10 m RMS: 0.045–0.063 m in 7-minute runs; **0.123 m** over a full
  25-minute mission (LiDAR)
- [x] Map agreement ≥ the Phase 2 target (90%): 96.5% wall agreement (full mission, seed 1)
- [x] Localization-loss failsafe: seen working in the camera-only runs (odometry lost → hover →
  recovered → resume; lost for 3 s → ABORT → landed). A scripted sensor-drop test is in the batch
- [ ] Handoff complete, tagged `phase-6`

## How to run

```bash
# Full mission on the drone's own localization (LiDAR ICP odometry + EKF + RTAB-Map)
ros2 launch airmouse_bringup mission.launch.py seed:=1            # use_ground_truth defaults to false
ros2 service call /mission/start std_srvs/srv/Trigger
#   use_lidar:=false -> RGB-D visual odometry (not reliable yet); wall_texture:=textured
#   use_ground_truth:=true -> Phase 5 behaviour

python3 tools/pose_error.py                 # estimated vs true pose (Ctrl+C prints the result)
python3 tools/check_contract.py             # contract topics + TF map -> odom -> base_link
python3 tools/cpu_usage.py --duration 60    # CPU per node
AIRMOUSE_SIM_TESTS=1 python3 -m pytest src/airmouse_bringup/test/test_slam_launch.py -s
GT=false bash tools/batch/mission_run.sh <seed> <lidar> [texture] [rtf] [timeout_s]
bash tools/batch/slam_trial.sh <label> <seed> <lidar> <texture> <seconds> ["Key=value,..."]
```

## What runs with ground truth off (`localization.launch.py`)

| Node | Role |
|---|---|
| `rtabmap_odom/icp_odometry` (`use_lidar` true) | scan matching on `/scan` → `/rtabmap/odom` |
| `rtabmap_odom/rgbd_odometry` (`use_lidar` false) | visual odometry on colour + depth → `/rtabmap/odom` |
| `range_to_z_node` | `/range/down` → `/range/down/pose` (z; offset 0.06 m; −inf = on the floor) |
| `robot_localization/ekf_node` (`ekf.yaml`) | fuses `/rtabmap/odom` (x, y, yaw, vx, vy, vyaw), `/camera/imu` (roll, pitch, rates, ax, ay), `/range/down/pose` (z) → `/odom` + TF `odom → base_link` at 50 Hz |
| `rtabmap_slam/rtabmap` | mapping on `/odom`; publishes `/map` and TF `map → odom` |

## Results

Full mission, seed 1, plain walls, LiDAR, own localization, detector on:

| Metric | Value |
|---|---|
| Final state | DONE (flight time 1512 s, path 443 m) |
| Arena seen / rooms | 99.9% / 7 of 7 |
| Survivors in camera coverage / detected | 6 of 6 / 5 of 6 (59 detection frames, 1 with no survivor in view) |
| Pose error (map → base_link vs truth) | 0.123 m RMS, max 0.232, final 0.143 (0.074 after rigid alignment) |
| Odometry-only error (odom → base_link) | 0.126 m RMS |
| Map: wall agreement / false wall cells / free seen | 96.5% / 16.6% / 99.5% |
| Landing error | 0.17 m |
| Min wall clearance | 0.299 m (**contact**) |

Seven-minute tuning runs (seed 1, LiDAR):

| RTAB-Map setting | Map-frame RMS | Odometry-only RMS |
|---|---|---|
| ICP neighbour-link refining on (first try) | 0.209 m (0.293 m over a full mission) | 0.019 m |
| Refining off, `Icp/CorrespondenceRatio 0.4` (**adopted**) | 0.063 m | 0.062 m |
| Refining off, visual registration only, no proximity | 0.045 m | 0.045 m |

CPU per node during a LiDAR mission (dev machine, shared with a 10-process batch, so upper
bounds; % of one core): rtabmap 52, icp_odometry 22, ekf_node 15, range_to_z_node 21,
explorer_node 50, controller_node 30, safety_hook_node 28, survivor_detector_node 46;
onboard total ≈ 264% (2.6 cores). To be re-measured unloaded and scaled to the Pi in Phase 7.

## Final parameters and why

- **ICP odometry** (`rtabmap.yaml` `icp_odometry`): `Reg/Force3DoF true` (a 2D scan gives x, y,
  yaw), frame-to-map, 5 cm voxels, point-to-plane, `Odom/ResetCountdown 1`.
- **RGB-D odometry** (`rgbd_odometry`): frame-to-map, `Vis/MaxFeatures 600`, `Vis/MinInliers 12`.
- **EKF** (`ekf.yaml`): 50 Hz, 3D mode; the sim IMU's roll/pitch are fused.
- **RTAB-Map SLAM** (set in `localization.launch.py`): `Kp/MaxFeatures 400`,
  `Vis/MaxFeatures 600`, `Reg/Strategy 1` with LiDAR, `RGBD/ProximityBySpace true`,
  **`RGBD/NeighborLinkRefining false`** (ICP on flat 2D scans in 6 DoF moved the map frame,
  0.14 m in height), `Icp/CorrespondenceRatio 0.4`. Tuning hook:
  `AIRMOUSE_SLAM_OVERRIDES="Key=value,..."`.
- **Localization loss** (`explorer.yaml`): `odometry_timeout 0.5 s`, `max_position_std 0.5 m`.
  **Provisional: the owner has not confirmed these values.**

## Changes outside localization made in this phase

- **Altitude comes from the odom frame** (EKF rangefinder height) in `controller_node` and
  `explorer_node`, not from the map frame, which SLAM corrections can shift.
- **Stepwise safety-margin fallback** (`viewpoint.relaxed_margins`): 0.10 → 0.05 → 0.00 m, used
  only when no viewpoint or no way home exists at the larger margin. SLAM maps draw walls
  3–4 cells thick instead of 2, which closes 1 m corridors at the full 0.40 m inflation.
- **Landing point:** the nearest safe cell within 0.6 m when the home cell is blocked on the map.
- **Turn-on-arrival (camera-only mode)** is dropped after 8 s or when localization is lost.

## Known weaknesses and open issues

1. **Camera-only (visual odometry) does not complete missions.** Observed: (a) when the start
   faces a wall 0.5 m away, the wall comes inside the depth camera's 0.3 m minimum range during
   take-off and every feature loses its depth; (b) with an open start, odometry drops out
   during in-place turns near walls for more than 3 s and the failsafe aborts; pose error
   reached 0.8 m before the abort. Feeding the EKF motion as the odometry guess and halving
   the yaw rate did not help. Plain walls were not tried. Camera-only works on ground truth.
2. **Position error grows with distance** (0.126 m RMS after 443 m): RTAB-Map's loop closures
   do not yet correct the odometry drift.
3. **Walls are 3–4 cells thick on SLAM maps** (16.6% of wall cells more than 0.10 m from a
   true wall). Hence the margin fallback, and **wall contact (0.299 m) in the full mission**.
4. **The real D435i IMU has no orientation output:** `imu_filter_madgwick` must run before the
   EKF on the drone (the simulated IMU provides orientation).
5. `range_to_z_node` does not correct for tilt (under 2% at 10°).
6. The detector raised false detections on textured walls (8 frames in one ground-truth run).
