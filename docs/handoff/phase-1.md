# Handoff: Phase 1 → Phase 2

Phase 1 (Simulation World and Drone) delivers a launchable Gazebo simulation with all contract
sensor topics, ground-truth odometry, and ground-truth layout files for scoring maps.

## Exit criteria

- [x] Maze generator tests pass for 50 seeds (`src/airmouse_description/test/`, 204 tests)
- [x] Drone hovers at 1.2 m and moves in the commanded direction (forward, left, up, yaw left);
  measured with ground truth, see below
- [x] Every contract sensor topic publishes at its rate (table below)
- [x] Works with `use_lidar` true and false, and with plain and textured walls
- [ ] Visual Gazebo/RViz check confirmed by the owner (see "Gazebo check")
- [ ] Tagged `phase-1`

## Commands

Inside the dev container (`/ws`), after `colcon build --symlink-install && source install/setup.bash`:

```bash
# Generate a maze only (optional; the launch file does it too)
ros2 run airmouse_description generate_maze.py --seed 3 --wall-texture textured

# Launch the sim (Gazebo + bridge + RViz). Arguments and defaults:
#   seed:=0  wall_texture:=plain|textured  use_lidar:=true  use_ground_truth:=false
#   use_sim_time:=true  gui:=true (false = headless)  rviz:=true
ros2 launch airmouse_bringup sim.launch.py seed:=3 use_ground_truth:=true

# Second terminal: take off to 1.2 m (test tool), then drive with the keyboard
python3 tools/takeoff.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link

# Tests
pytest src -q        # 254 tests: maze generator, adapter, scaffold
```

`use_ground_truth:=true` is needed for an odom → base_link TF (and so for RViz) until the EKF
exists (Phase 6). RViz's fixed frame is `odom` for now.

## Where files are

- Generated mazes: `src/airmouse_description/worlds/generated/seed_<N>_<plain|textured>/`
  (git-ignored, regenerated at every launch; same seed → same maze):
  - `world.sdf`: Gazebo world;
  - `layout.pgm` + `layout.yaml`: ground-truth occupancy, 0.05 m cells, map_server format
    (0 = wall, 254 = free), origin in the world/map frame;
  - `layout.json`: entry, `arena_bounds`, rooms (centre, min, max) and survivors;
  - `wall_*.png`: wall textures (textured only).
- Drone model: `src/airmouse_description/models/airmouse/model.sdf`.
- Bridge: `src/airmouse_bringup/config/bridge.yaml`; adapter settings: `config/sim_bridge.yaml`.

Frames: world = map frame, origin at the entry cell centre (take-off point), +x into the maze.
TF: `odom → base_link` (ground truth, when enabled); static `base_link → {camera_link,
lidar_link, range_down_link, range_up_link}` read from `model.sdf` by the launch file;
`camera_link → camera_optical_frame` (images, depth and camera_info use the optical frame).

## Measured topic rates (`ros2 topic hz`, seed 3, GTX 1650)

| Topic | Target | LiDAR on, plain | LiDAR off, textured |
|---|---|---|---|
| `/camera/color/image_raw` | 15 Hz | 14.7 | 15.1 |
| `/camera/depth/image_rect_raw` | 15 Hz | 14.7 | 15.2 |
| `/camera/color/camera_info`, `/camera/depth/camera_info` | 15 Hz | 14.7 | 15.2 |
| `/camera/imu` | ≥ 100 Hz | 232 | 251 |
| `/scan` | 10 Hz (LiDAR on) | 9.6 | none (correct) |
| `/range/down`, `/range/up` | ≥ 20 Hz | 29.5 / 29.8 | 30.4 / 30.3 |
| `/ground_truth/odom` | 50 Hz | 48.5 | 50.0 |
| `/camera/depth/points` (extra, for RViz) | — | 11.9 | 13.5 |

Hovering at 1.20 m: `/range/down` = 1.13 m, `/range/up` = 1.18 m (net ceiling at 2.44 m), as
expected from the sensor offsets (±0.06 m).

## Adapter frame convention

Gazebo's `MulticopterVelocityControl` reads the commanded linear velocity in the **vehicle
heading (body) frame**: x forward, y left, z up; `angular.z` = yaw rate. Measured: at yaw ≈ 90°,
body "left" moved the drone along world −x; at yaw ≈ 130–150°, body "left" moved it along world
≈ 205°. So `sim_bridge_node` passes `/cmd_vel` through (`plugin_frame: body` in
`sim_bridge.yaml`); a `world` mode (rotate by yaw) exists and is unit-tested for 6 yaw angles.

Direction test from hover (ground truth, 0.3 m/s): forward → +0.17 m x; left → +0.20 m y;
up (0.1 m/s) → +0.30 m z; yaw left (0.4 rad/s) → +86° heading.

## Deviations from the phase document, and why

1. **Corridor geometry:** walls 0.10 m thick on a 1.10 m pitch, so corridors are exactly 1.0 m
   clear; 13 × 13 cells = 14.4 m arena. Rooms are 2 × 2 cells = 2.1 m clear (owner approved).
2. **Drone built from simple shapes**, not the X3 Fuel mesh (no internet needed). Rotor, motor
   and controller values are copied from Gazebo's `multicopter_velocity_control.sdf`; body
   mass 1.6 kg with X3 inertia scaled by 1.6/1.5.
3. **Rangefinders** are 1-beam `gpu_lidar` sensors (Gazebo has no single-beam range sensor),
   converted to `sensor_msgs/Range` by `sim_bridge_node`. VL53L1X values for both (0.04–4 m,
   27° FOV, ±1 cm); 30 Hz so the measured rate stays above 20 Hz (owner approved).
4. **Noise:** depth/image σ = 0.01, IMU gyro σ = 0.009 rad/s, accel σ = 0.021 m/s², LiDAR
   σ = 0.03 m, 360 samples (1°). IMU at 250 Hz (one sample per 4 ms physics step).
5. **Depth image** is 32-bit float metres (Gazebo's format), not the RealSense 16-bit mm;
   RTAB-Map accepts both.
6. **Extra topics** (no contract name changed): `/camera/depth/points` (from
   `depth_image_proc`), `/range/*/scan`, `/airmouse/cmd_twist`, `/airmouse/enable`.
7. **New files:** `config/sim_bridge.yaml`, `airmouse_nav/bridges/sim_bridge_node.py`,
   `tools/takeoff.py`, extra frame `camera_optical_frame`.
8. **Survivors:** orange 1.4 m cylinder + head (1.7 m), static, no download (owner approved).
9. **Net ceiling:** a thin, 80 %-transparent box at 2.44 m with collision, so the upward
   sensor sees it.
10. **When `use_lidar` is false**, the LiDAR is removed from the model, but the bridge still
    creates a silent `/scan` publisher.

## Open issues

- **Yaw drifts during translation** (≈ 20° over 0.5 m sideways): the plugin's yaw gain is
  low (X3 value 0.15). Phase 4's controller closes the loop on heading; if that is not
  enough, raise `attitudeGain` z in `model.sdf`.
- After a yaw-rate command stops, the heading overshoots by tens of degrees for the same
  reason.
- `/range/down` reads +inf while the drone sits on the floor (the beam starts at floor
  level, below the 0.04 m minimum); fine once airborne.
- The point cloud runs at about 12–13 Hz (depth_image_proc on the CPU).
- The plugin keeps flying the last command if `/cmd_vel` stops; Phase 4's stale-data rule
  (zero command) covers this.
