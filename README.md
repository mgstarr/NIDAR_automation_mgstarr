# AirMouse Navigation

Autonomous navigation for the NIDAR AirMouse drone: it explores an unknown indoor maze,
builds a live 2D map, makes sure its camera sees every room, returns to the entry and
lands. Everything is developed and tested in Gazebo first.

- Master plan (single source of truth): [docs/PLAN.md](docs/PLAN.md)
- Phase documents: [docs/phases/](docs/phases/) · Handoff notes: [docs/handoff/](docs/handoff/)

This folder is both the git root and the ROS 2 workspace root.

## 1. One-time host setup

Ubuntu with an NVIDIA GPU, plus:

- Docker Engine, with your user in the `docker` group (log out and in after adding it);
- NVIDIA Container Toolkit (`docker run --rm --gpus all ubuntu nvidia-smi` must work);
- VS Code with the **Dev Containers** extension (optional; plain `docker compose` also works).

## 2. Open the dev container

The container is Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic, with GPU access and
windows forwarded to your screen. The repo is mounted at `/ws` inside it.

**VS Code:** open this folder, then *Ctrl+Shift+P → Dev Containers: Reopen in Container*.
The first build downloads about 5 GB.

**Without VS Code:**

```bash
xhost +SI:localuser:$(id -un)        # allow the container to open windows (once per login)
echo RENDER_GID=$(getent group render | cut -d: -f3) > .devcontainer/.env   # host GPU group id
docker compose -f .devcontainer/docker-compose.yml up -d --build
docker compose -f .devcontainer/docker-compose.yml exec dev bash
# when done:
docker compose -f .devcontainer/docker-compose.yml down
```

## 3. Build and test (inside the container)

```bash
cd /ws
colcon build --symlink-install
source install/setup.bash
pytest src/airmouse_nav/test -v
ros2 pkg list | grep airmouse        # shows the three packages
```

## 4. Check graphics

```bash
gz sim shapes.sdf     # a Gazebo window with a few shapes
rviz2                 # an RViz window
```

## Packages

| Package | Contents |
|---|---|
| `src/airmouse_description` | drone model, maze worlds, maze generator |
| `src/airmouse_bringup` | all launch files, all parameter files, RViz layout |
| `src/airmouse_nav` | navigation algorithms (`core/`), ROS 2 nodes, flight-controller bridges, tests |
| `tools/` | offline scripts: map scoring, plotting, benchmarks |

## Run a full mission (navigation + mapping + survivor detection)

```bash
# Terminal 1: everything in one command (Gazebo, RViz, mapping, explorer, controller, detector)
ros2 launch airmouse_bringup mission.launch.py seed:=1 use_ground_truth:=true
# Terminal 2: start the mission
ros2 service call /mission/start std_srvs/srv/Trigger
```

- `use_ground_truth:=false` (the default) runs on the drone's own localization (LiDAR scan
  matching + EKF + RTAB-Map). Camera-only localization (`use_lidar:=false`) is not reliable yet.
- RViz shows the map, the planned path, frontiers, camera coverage and the **Survivor
  detection** image (camera frames with boxes around detected survivors).
- Status per phase: [docs/handoff/](docs/handoff/).

## Project structure

```
.devcontainer/            Docker dev container (ROS 2 Jazzy, Gazebo Harmonic, PyTorch CPU, Ultralytics)
docs/
  PLAN.md                 master plan (single source of truth)
  phases/phase-N.md       the phase documents
  handoff/phase-N.md      what each phase delivered, results, open issues
external/
  NIDAR-Automation/       detection team's repository, copied as-is (snapshot of
                          github.com/AryanGoyal17/NIDAR-Automation at a49d362): YOLO11n training,
                          evaluation, detector CLI and models/survivor_yolo11n_demo.pt
src/
  airmouse_description/   the simulated world: maze generator (human-shaped survivors), drone model
  airmouse_bringup/       launch files, all YAML configuration, RViz layout
    launch/               sim, localization, navigation, detection, mission (all-in-one), rviz
    config/               rtabmap, ekf, planner, controller, explorer, detection, bridge
  airmouse_nav/           navigation
    core/                 pure Python, no ROS: grid_utils, astar, pure_pursuit, frontier,
                          coverage, viewpoint, mission
    nodes/                explorer_node (mission + exploration), controller_node,
                          safety_hook_node, goal_planner_node (RViz click test tool)
    bridges/              sim_bridge (Gazebo adapter), range_to_z (height for the EKF),
                          fc_bridge (flight-controller hook, not built yet)
  airmouse_detection/     ROS 2 wrapper around the detection team's model:
                          camera -> YOLO11n -> /detection/survivors, /detection/image, /detection/json
tools/                    offline tools and test runners (see tools/README.md)
  batch/                  headless mission runs and exit-test batches
```

Data flow: camera, LiDAR, IMU and rangefinders → **localization** (RTAB-Map odometry + EKF →
`/odom`; RTAB-Map → `/map`) → **explorer** (frontiers, coverage, next viewpoint, A*) →
`/nav/path` → **controller** (pure pursuit) → **safety hook** → `/cmd_vel` → drone. In parallel:
camera → **survivor detector** → `/detection/*`. Not built yet: placing detected survivors on
the map (tagging).
