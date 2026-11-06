# Handoff: Phase 2 → Phase 3

Phase 2 (Mapping with Ground-Truth Pose) delivers a live `/map` from RTAB-Map in simulation
and six saved maps for offline planner tests.

## Exit criteria

- [x] `/map` updates at 1–2 Hz during flight (RTAB-Map runs at 2 Hz sim time; measured
  3.7–3.9 Hz wall clock with the sim at ≈ 2× real time)
- [x] Wall-cell agreement ≥ 90% on 3 mazes: 99.97–100% on all 6
- [x] No false walls inside corridors or doorways: 0 on seeds 1–3; 1–2 single 5 cm cells on
  seeds 4–6 (see open issues)
- [x] At least 5 saved maps from different seeds, LiDAR on and off: 6 maps in
  `src/airmouse_nav/test/data/maps/`
- [ ] Owner's visual RViz check
- [ ] Tagged `phase-2`

## Commands

Inside the dev container (`/ws`), after `colcon build --symlink-install && source install/setup.bash`:

```bash
# Terminal 1: sim (RViz included; the Map display shows /map)
ros2 launch airmouse_bringup sim.launch.py seed:=3 use_ground_truth:=true
# Terminal 2: mapping
ros2 launch airmouse_bringup localization.launch.py use_ground_truth:=true   # use_lidar:=false for camera only
# Terminal 3: fly (take off, then keyboard), or let the survey tool fly the whole maze
python3 tools/takeoff.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link
python3 tools/survey_flight.py --layout src/airmouse_description/worlds/generated/seed_3_plain

# Save and score the live map
python3 tools/save_map.py --name my_map --seed 3 --use-lidar true --wall-texture plain
python3 tools/score_map.py --map src/airmouse_nav/test/data/maps/my_map.yaml \
    --layout src/airmouse_description/worlds/generated/seed_3_plain/layout.yaml

# Batch (headless, unattended): 6 mazes, results in /tmp/batch.log (~9 min per maze)
bash tools/batch/map_batch.sh          # one run: tools/batch/map_run.sh <seed> <lidar> <texture> <rtf>
```

`localization.launch.py` arguments: `use_lidar` (true), `use_ground_truth` (false; must be
true until Phase 6), `use_sim_time` (true). New `sim.launch.py` argument:
`real_time_factor` (1.0; batch runs use 2.0).

## Saved map format

Files: `src/airmouse_nav/test/data/maps/<name>.pgm` + `<name>.yaml` (map_server format; read
and write with `tools/map_io.py`).

- PGM (binary P5): 0 = occupied, 254 = free, 205 = unknown; the first image row is the
  **top** (highest y).
- YAML: `image`, `mode: trinary`, `resolution: 0.05` (m per cell), `origin: [x, y, 0]` =
  map-frame position of the **lower-left** corner of the image, `negate: 0`,
  `occupied_thresh: 0.65`, `free_thresh: 0.196`, plus `airmouse: {seed, use_lidar, wall_texture}`.
- In code (`map_io.load`): int8 grid, row 0 = lowest y, values 100 / 0 / -1 (same as
  `nav_msgs/OccupancyGrid.data`).
- Map frame: origin at the take-off point (entry cell centre), +x into the maze. Ground-truth
  layouts use the same frame, so the two align by origin.

## Saved maps and scores (tolerance 0.10 m)

| Map | LiDAR | Walls | Wall agreement | False wall cells | Free space seen |
|---|---|---|---|---|---|
| `seed1_lidar_plain` | on | plain | 100.00% | 0 | 99.6% |
| `seed2_nolidar_textured` | off | textured | 99.97% | 0 | 96.6% |
| `seed3_lidar_textured` | on | textured | 100.00% | 0 | 98.8% |
| `seed4_nolidar_textured` | off | textured | 100.00% | 1 | 95.7% |
| `seed5_lidar_plain` | on | plain | 100.00% | 1 | 98.8% |
| `seed6_nolidar_plain` | off | plain | 99.98% | 2 | 94.6% |

Each survey flight took ≈ 500 s wall clock (≈ 1000 s sim time).

## RTAB-Map parameters changed from defaults, and why (`config/rtabmap.yaml`)

| Parameter | Default | Ours | Why |
|---|---|---|---|
| `Rtabmap/DetectionRate` | 1 | 2 | `/map` at 1–2 Hz (contract) |
| `Kp/MaxFeatures` | 500 | -1 | no visual words, so no loop closure: the pose is exact in Phase 2; saves CPU |
| `Grid/3D` | true | false | 2D grid only |
| `Grid/RangeMax` | 5.0 | 3.0 | D435i reliable range; same cap for the LiDAR (owner approved) |
| `Grid/MapFrameProjection` | false | true | heights measured from the floor, not from the flying drone |
| `Grid/NormalsSegmentation` | true | false | use the plain height band instead |
| `Grid/MaxGroundHeight` | 0.0 | 0.2 | `grid_height_band` lower edge |
| `Grid/MaxObstacleHeight` | 0.0 | 2.2 | `grid_height_band` upper edge (net ceiling at 2.44 m ignored) |
| `Grid/RayTracing` | false | true | mark free space between the sensor and each obstacle |
| `Grid/NoiseFilteringRadius` / `MinNeighbors` | 0.0 / 5 | 0.1 / 5 | remove isolated depth specks |
| `Grid/Sensor` | 1 | 2 with LiDAR, 1 without | set by the launch file from `use_lidar` |
| `Grid/CellSize` | 0.05 | 0.05 | `grid_resolution` (unchanged, stated explicitly) |

ROS-side: `map_always_update: true` (republish while hovering), `approx_sync: true`,
`--delete_db_on_start`. `Reg/Force3DoF` is explicitly **false**: with it on, RTAB-Map drops
the drone's tilt, so when the drone leans while speeding up or braking, floor points are lifted
into the wall band. That produced up to 12% false walls before the fix.

## Deviations and open issues

- **Survey flight tool** (`tools/survey_flight.py`, owner approved) builds the saved maps
  instead of keyboard flight. It uses the true layout and pose; it is not the navigation
  stack. It skips moves within 0.6 m of a survivor.
- **Scorer counts survivors as real obstacles** (radius 0.20 m from `layout.json`), because
  the ground-truth image holds walls only.
- **Remaining false walls:** 1–2 isolated 5 cm cells on three camera-heavy maps; Phase 3's
  inflation covers them, but they can show up as tiny obstacles.
- **Camera-only maps see 94.6–96.6% of the free space** (vs 98.8–99.6% with LiDAR): corners
  behind the camera stay unknown. Phase 5's coverage logic handles this.
- `/map` publishes at 2 Hz sim time. On the Raspberry Pi this may need to drop (Phase 7).
- New launch argument `real_time_factor` and new files `tools/map_io.py`,
  `tools/survey_flight.py`, `tools/batch/*.sh`.
