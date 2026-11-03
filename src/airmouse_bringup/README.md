# airmouse_bringup

All launch files and all parameter files for the AirMouse stack. No code lives here.

## Launch files

| File | Starts | Phase |
|---|---|---|
| `sim.launch.py` | Gazebo maze world, simulated drone, ros_gz bridge | 1 |
| `localization.launch.py` | RTAB-Map (+ EKF from Phase 6) | 2, 6 |
| `navigation.launch.py` | explorer, controller and safety hook nodes | 3–5 |
| `rviz.launch.py` | RViz with `rviz/airmouse.rviz` | 1 |
| `mission.launch.py` | everything: sim, localization, navigation, RViz | 5, 8 |

Shared launch arguments (every file; defaults from `docs/PLAN.md` › Parameters reference):

- `use_lidar` (default `true`): use the RPLIDAR C1 2D scan.
- `use_ground_truth` (default `false`): use Gazebo's exact pose instead of SLAM (Phases 2–5).
- `use_sim_time` (default `true` in `sim`, otherwise `false`): use the simulation clock.

## sim.launch.py

```bash
ros2 launch airmouse_bringup sim.launch.py seed:=3 use_ground_truth:=true
```

Arguments: `seed` (0), `wall_texture` (`plain`/`textured`), `use_lidar` (true),
`use_ground_truth` (false; true bridges Gazebo's exact odom -> base_link TF), `use_sim_time`
(true), `gui` (true; false runs Gazebo headless), `rviz` (true).

It generates the maze, starts Gazebo, spawns the drone (without LiDAR when `use_lidar` is
false), runs `ros_gz_bridge` with `config/bridge.yaml`, `sim_bridge_node`, a depth -> point
cloud node (`/camera/depth/points`) and the static TFs read from the model file.

## Config files

`rtabmap.yaml`, `ekf.yaml`, `explorer.yaml`, `planner.yaml`, `controller.yaml`,
`bridge.yaml` (Gazebo -> contract topic names) and `sim_bridge.yaml` (sim adapter). Every tunable number in the stack lives in one of these files; names and
defaults come from `docs/PLAN.md` › Parameters reference. They are empty in Phase 0 and
filled by the phase that first needs them. `planner.yaml` is loaded into every node that
plans, because the planner is a library and not a node.

## Build and run

From the workspace root (inside the dev container):

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch airmouse_bringup sim.launch.py --show-args
```

## Topics

See `docs/PLAN.md` › Interface contract.
