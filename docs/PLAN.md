# AirMouse Navigation – Phased Implementation Plan

Nov 2, 2026 · @Mahima

> Markdown export of the PDF *AirMouse Navigation – Phased Implementation Plan*. The PDF is
> the original; if the two ever disagree, the PDF wins and this file is corrected.

Nine phases (0–8) take the navigation stack from an empty repository to a Gazebo-tested
package the controls team can run with one command. Each phase ends with tests and a
visible check in Gazebo or RViz before the next one starts.

## How to use this document

This document is the single source of truth for the navigation work. Claude Code should
read all of it before starting, then follow these rules.

1. **One phase at a time, in order.** Never start a phase before the previous phase's exit
   criteria pass.
2. **Approach before code.** At the start of each phase, explain the plan: files, functions,
   what each does and how. Wait for approval before writing code.
3. **Small steps inside a phase.** Write one module at a time, with its tests, then pause for
   review.
4. **Exit criteria are the definition of done.** A phase is finished only when every exit check
   passes and the Gazebo check has been shown.
5. **Ask, don't invent.** If a hardware value, interface or rule is not in this document, ask.
   Never guess.
6. **Explain new terms.** The owner is new to drones and ROS 2. Explain an unfamiliar term
   briefly the first time it appears.
7. **Keep the contract.** Topic names, message types, frames and parameter names in this
   document are fixed. Changing one needs approval, and this document gets updated.
8. **Out of scope for now:** obstacle-avoidance safety filter (CBF), survivor detection,
   survivor tagging, and real-hardware drivers. The design leaves clean hooks for each.

## Scope and end result

The end result is a tagged release the controls team can launch with one command. In it, a
simulated drone autonomously explores a random maze, builds a live 2D map, returns to
the entry and lands, all visible in Gazebo and RViz.

### In scope

- Localization and mapping (configuring RTAB-Map + EKF; no custom SLAM code)
- Exploration: frontiers, camera coverage, viewpoint selection, mission state machine
- A* path planner
- Pure pursuit path-following controller
- Gazebo simulation: maze generator, simulated drone, sensors
- Tests, benchmarks and documentation

### What the controls team receives (Phase 8)

| Item | Purpose |
|---|---|
| Docker dev container | Same ROS 2 Jazzy + Gazebo Harmonic setup on any Ubuntu machine with an NVIDIA GPU |
| One-command launch | `ros2 launch airmouse_bringup mission.launch.py` starts sim, localization, navigation and RViz |
| Interface contract | Exact topics, message types and frames, so their controller or flight model can plug in |
| Test procedure | Step-by-step scenarios with expected results |
| Benchmark report | Mission time, coverage and wall clearance over many random mazes |
| Parameter reference | Every tunable value, its default and its effect |
| Known limitations | What is not done yet and what to watch for |

## Fixed facts

### Mission constraints (from the NIDAR AirMouse brief)

- The arena is a covered maze, at most 15 m × 15 m, built on a modular grid.
  - Corridors are at least 1 m wide. Rooms are 2 m × 2 m. Clear height is 2.44 m (8 ft).
  - The roof is a net.
  - The layout is unknown before the mission.
- One point is both the entry and the exit. Launch is from a 2 ft × 2 ft area.
- No GPS and no external network (no internet, GSM or Wi-Fi). Communication is over
  the team's local link only.
- Fully autonomous. The operator may only start the mission and trigger an abort or
  emergency stop.
- Maximum flight time is 30 minutes. Scored on total time, autonomy, map accuracy and
  survivor localization.
- Up to 6 survivors (people or dummies) are in rooms. The cameras must see every room.
- The 2D map and drone pose must stream live to the ground station during flight. There
  is no post-flight processing.
- The drone must never touch walls, ceiling or obstacles.
- Required failsafes: low battery, lost link, geofence breach, mission abort, emergency
  recall.

### Drone hardware (all values are parameters in code)

| Item | Value |
|---|---|
| Onboard computer | Raspberry Pi 5, 8 GB, CPU only (4 cores) |
| Primary sensor | Intel RealSense D435i, forward-facing; depth field of view 87° × 58°; reliable 0.3–3 m; built-in IMU; run at 424×240, 15 Hz |
| Optional sensor | RPLIDAR C1 2D LiDAR: 360°, 12 m, 10 Hz. Electronics decides whether it is fitted; the stack must work with and without it (`use_lidar`) |
| Height sensor | Downward rangefinder (model to be confirmed) |
| Upward sensor | VL53L1X ToF, for the net ceiling |
| Flight controller | Own STM32 firmware for stabilization. Accepts velocity and yaw-rate setpoints; link protocol not decided |
| Mass / size | About 1.6 kg; guarded radius a parameter (default 0.30 m) |

### Software environment

- Target: ROS 2 Jazzy, Gazebo Harmonic, Python 3 (rclpy) with numpy.
- **Dev machine:** Ubuntu 22.04 with an NVIDIA RTX 4060. Jazzy does not run natively on
  22.04, so all development runs inside a **Docker dev container** (Ubuntu 24.04 + Jazzy +
  Harmonic) with GPU access. The same container is what the controls team receives, so
  everyone runs an identical setup.
- The Raspberry Pi 5 runs Ubuntu 24.04 + Jazzy natively, matching the container.

## System architecture

Navigation code turns one map and pose into flight commands. (In the PDF diagram, the
navigation block is what this plan builds; everything else is configured or provided.)

```
 Depth camera (D435i)     2D LiDAR (optional)      IMU + rangefinders
          │                       │                        │
 ┌────────▼───────────────────────▼──────┐   ┌────────────▼───────────────┐
 │ RTAB-Map                              │──▶│ EKF (robot_localization)   │   Localization
 │ builds /map, corrects drift           │   │ fuses odometry, IMU, height│   (configured)
 └────────┬──────────────────────────────┘   └────────────┬───────────────┘
          │ /map + pose                                    │ pose
 ┌────────▼──────────────┐   ┌─────────────────────▼──┐   ┌──────────────────┐
 │ Explorer node         │──▶│ Controller node        │──▶│ Safety hook      │  Navigation
 │ frontiers, coverage,  │   │ pure pursuit at 30 Hz  │   │ pass-through now │  (built in
 │ viewpoints, mission   │   │ holds altitude and yaw │   │ CBF filter later │  this plan)
 │ states; plans with A* │   └────────────────────────┘   └────────┬─────────┘
 └────────┬──────────────┘                                         │ /cmd_vel
          │ map, coverage, mission state                           │
 ┌────────▼──────────────────────┐                ┌────────────────▼──────────────┐
 │ Ground station (teammate 3)   │                │ Drone                         │
 │ shows live map, pose and      │                │ Gazebo model in simulation,   │
 │ mission state                 │                │ STM32 bridge on hardware      │
 └───────────────────────────────┘                └───────────────────────────────┘
```

*Navigation stack · data flow from sensors to drone.* Sensors feed localization, which
produces one map and pose. The explorer picks the next viewpoint and plans with A*. The
controller follows the path, and the safety hook passes commands to the drone. The ground
station gets the same map and mission state.

## Interface contract

These names are fixed. In simulation, the sensor topics come from Gazebo through
`ros_gz_bridge` (the package that copies Gazebo data onto ROS 2 topics). On the real
drone, the same names come from the hardware drivers, so no navigation code changes.

### Topics

| Topic | Type | Producer → consumer | Rate |
|---|---|---|---|
| `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/*/camera_info` | `sensor_msgs/Image`, `CameraInfo` | camera (sim or RealSense) → RTAB-Map, coverage | 15 Hz |
| `/camera/imu` | `sensor_msgs/Imu` | camera → EKF | ≥ 100 Hz |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR → RTAB-Map (only if `use_lidar`) | 10 Hz |
| `/range/down` | `sensor_msgs/Range` | downward rangefinder → EKF, controller | ≥ 20 Hz |
| `/range/up` | `sensor_msgs/Range` | VL53L1X → (future safety filter) | ≥ 20 Hz |
| `/ground_truth/odom` | `nav_msgs/Odometry` | Gazebo → TF (sim only, when `use_ground_truth`) | 50 Hz |
| `/odom` | `nav_msgs/Odometry` | EKF → controller | 50 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | RTAB-Map → explorer, planner, ground station | 1–2 Hz |
| `/nav/coverage` | `nav_msgs/OccupancyGrid` | explorer → RViz, ground station | 1 Hz |
| `/nav/frontiers` | `visualization_msgs/MarkerArray` | explorer → RViz | 1 Hz |
| `/nav/goal` | `geometry_msgs/PoseStamped` | explorer → RViz (chosen viewpoint) | on change |
| `/nav/path` | `nav_msgs/Path` | explorer → controller | on replan |
| `/nav/cmd_vel_raw` | `geometry_msgs/TwistStamped` | controller → safety hook | 30 Hz |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | safety hook → drone (Gazebo plugin or flight-controller bridge) | 30 Hz |
| `/mission/state` | `std_msgs/String` | explorer → everyone | on change |

**Velocity command convention.** `/cmd_vel` is in the body frame (`base_link`) using ROS's
standard axes (REP 103): x forward, y left, z up, in m/s. Yaw rate is `angular.z` in rad/s.
Until the CBF filter exists, the safety hook is a pass-through from `/nav/cmd_vel_raw` to
`/cmd_vel`.

### Services

| Service | Type | Effect |
|---|---|---|
| `/mission/start` | `std_srvs/Trigger` | IDLE → TAKEOFF |
| `/mission/abort` | `std_srvs/Trigger` | Any state → ABORT (stop, hover, land in place) |
| `/mission/return_home` | `std_srvs/Trigger` | Emergency recall → RETURN_HOME |

### Frames (TF)

`map → odom → base_link → {camera_link, lidar_link, range_down_link, range_up_link}`

- `map → odom`: RTAB-Map.
- `odom → base_link`: the EKF (or ground truth in sim when `use_ground_truth` is true).
- Sensor frames: static transforms from the drone model.
- The map origin is the take-off point, with x along the entry direction. The arena grid
  aligns with this frame.

## Repository structure and coding standards

The repository root is the ROS 2 workspace root. It is also the git root.

```
<repo root>/
├── .devcontainer/              # Docker dev container (Jazzy + Harmonic + GPU)
├── docs/                       # this plan, interface contract, test procedure, reports
├── src/
│   ├── airmouse_description/   # drone model + maze worlds
│   │   ├── models/airmouse/    # SDF model: body, depth camera, optional LiDAR, IMU, rangefinders
│   │   ├── worlds/             # generated maze worlds
│   │   └── scripts/generate_maze.py
│   ├── airmouse_bringup/       # ALL launch files and ALL parameters
│   │   ├── launch/             # sim, localization, navigation, mission, rviz
│   │   ├── config/             # rtabmap.yaml, ekf.yaml, explorer.yaml, planner.yaml, controller.yaml, bridge.yaml
│   │   └── rviz/airmouse.rviz
│   └── airmouse_nav/
│       ├── airmouse_nav/
│       │   ├── core/           # pure algorithms, NO ROS imports
│       │   │   ├── grid_utils.py    # grid conversion, inflation, wall cost, metres↔cells
│       │   │   ├── astar.py
│       │   │   ├── frontier.py
│       │   │   ├── coverage.py
│       │   │   ├── viewpoint.py
│       │   │   ├── mission.py       # state machine
│       │   │   └── pure_pursuit.py
│       │   ├── nodes/          # thin ROS 2 wrappers
│       │   │   ├── explorer_node.py
│       │   │   ├── controller_node.py
│       │   │   ├── safety_hook_node.py    # pass-through now, CBF later
│       │   │   └── goal_planner_node.py   # test tool: RViz goal → A* path
│       │   └── bridges/        # flight-controller bridge (sim passthrough for now)
│       ├── test/               # pytest for every core/ module
│       └── README.md
├── tools/                      # benchmark runner, metrics, plotting
├── .gitignore                  # build/ install/ log/ __pycache__/ .pytest_cache/ *.db *.db3 *.mcap .vscode/
└── README.md
```

### Coding standards

- `core/` is pure Python + numpy and must run and be tested without ROS or Gazebo.
- Nodes only translate between ROS messages and `core/` functions. No algorithm logic
  lives in nodes.
- Every tunable number lives in a YAML file in `airmouse_bringup/config/`. No magic
  numbers.
- Type hints and a docstring on every function. Small functions with clear names.
- ROS 2 logging, never `print`.
- Grid operations are numpy-vectorized. Each `core/` function states its time cost in its
  docstring when it runs every cycle.
- One commit per finished step, with a clear message. A phase ends with a git tag
  (`phase-0` … `phase-8`).
- Each package has a README: what it does, how to run it, its topics and parameters.

## Phase overview

Phases 2–5 use the simulator's ground-truth pose (the exact position Gazebo knows).
That way planning and control are built and debugged without SLAM errors mixed in.
Phase 6 swaps in real localization.

| Phase | Goal | Main output | Exit gate (summary) |
|---|---|---|---|
| 0 | Environment and repo | Dev container, package skeleton, test setup | Container builds; colcon build and pytest pass |
| 1 | Simulation | Maze generator, drone model, sensors, bridge | Drone hovers and moves on `/cmd_vel`; sensors visible in RViz |
| 2 | Mapping (ground truth) | RTAB-Map mapping with ground-truth pose | Flying by keyboard builds a clean `/map` |
| 3 | A* planner | `grid_utils`, `astar`, goal-planner test node | RViz goal gives a safe, centred path in ≤ 50 ms |
| 4 | Controller | `pure_pursuit`, `controller_node`, safety hook | Drone follows A* paths to RViz goals without wall contact |
| 5 | Exploration + mission | Frontiers, coverage, viewpoints, state machine | Full mission on ground truth in 10/10 random mazes |
| 6 | Real localization | RTAB-Map odometry + EKF, ground truth removed | Full mission on SLAM, with and without LiDAR |
| 7 | Validation | Benchmark runner, fault tests, CPU profile | Benchmark targets met over 20+ mazes |
| 8 | Handoff | Docs, one-command launch, release tag | Controls team runs the test procedure unaided |

## Phase 0: Environment and repository

**Goal:** an identical, reproducible development setup and an empty but buildable
workspace.

**Tasks**

1. Dev container in `.devcontainer/`:
   - base image: Ubuntu 24.04 with ROS 2 Jazzy desktop (e.g. `osrf/ros:jazzy-desktop-full`);
   - Gazebo Harmonic, `ros_gz`, `rtabmap_ros`, `robot_localization`,
     `teleop_twist_keyboard`, numpy, scipy, pytest;
   - NVIDIA GPU access (needs `nvidia-container-toolkit` on the host) and
     X11/Wayland forwarding so Gazebo and RViz windows open;
   - works from VS Code "Reopen in Container" and from a plain `docker compose`
     command.
2. Workspace skeleton exactly as in Repository structure: three packages, empty
   modules with docstrings, empty YAML configs.
3. Test setup: pytest runs `airmouse_nav/test/`; one placeholder test per `core/` module.
4. Git: `.gitignore`, first commit, tag `phase-0`. Default ament linters are skipped for now.
5. Root README: how to open the container, build, test.

**Tests:** `colcon build` succeeds; `pytest` passes; `ros2 pkg list` shows the three
packages.

**Gazebo check:** `gz sim shapes.sdf` opens a window inside the container (proves GPU +
display forwarding).

**Exit criteria**

- [ ] Container builds and opens from VS Code
- [ ] Gazebo and RViz windows open from inside the container
- [ ] `colcon build` and `pytest` pass
- [ ] Tagged `phase-0`

## Phase 1: Simulation world and drone

**Goal:** a realistic test arena and a simulated drone that publishes the same topics the real
one will.

**Tasks**

1. Maze generator (`scripts/generate_maze.py`, input: a random seed):
   - lay out a grid of 1 m cells inside at most 15 m × 15 m;
   - carve corridors with a randomized depth-first search, then remove a few extra walls
     so the maze has loops;
   - place 2 m × 2 m rooms (target 6–8) reachable from corridors;
   - walls 2.44 m tall; one entry/exit opening on the outer wall;
   - place up to 6 simple human-sized survivor models inside rooms;
   - output an SDF world file, plus a ground-truth occupancy image + YAML of the
     layout (used later for scoring maps);
   - guarantee every room is reachable and every corridor is at least 1 m wide (assert in
     code).
2. Drone model (`models/airmouse/`), based on Gazebo Harmonic's multicopter velocity-
   control example (the X3 quadcopter):
   - accepts velocity + yaw-rate commands, so `/cmd_vel` moves it;
   - mass about 1.6 kg; collision radius = guarded radius parameter;
   - check the plugin's frame convention and convert our body-frame TwistStamped to
     what it expects in a small adapter in `bridges/`.
3. Sensors on the model, matching the real hardware:
   - RGB-D camera: 87° horizontal field of view, 424×240, 15 Hz, depth clipped 0.3–3 m,
     with noise;
   - optional 2D LiDAR: 360°, 0.05–12 m, 10 Hz, ±3 cm noise (on/off by `use_lidar`);
   - IMU with noise;
   - downward and upward single-beam rangefinders → `sensor_msgs/Range`;
   - ground-truth odometry publisher.
4. Bridge config (`config/bridge.yaml`) mapping every Gazebo topic to the names in the
   Interface contract.
5. Launch file `sim.launch.py` (arguments: `seed`, `use_lidar`, `use_ground_truth`) and
   an RViz config.

**Tests:** maze generator unit tests (connectivity, corridor width, room count, arena bounds)
over 50 seeds.

**Gazebo check:** launch the sim; take off; drive with `teleop_twist_keyboard`; RViz shows
the depth point cloud, the scan (when on), the TF tree and ground-truth odometry.

**Exit criteria**

- [ ] Maze generator tests pass for 50 seeds
- [ ] Drone hovers at 1.2 m and responds to `/cmd_vel` in the correct directions
- [ ] All contract topics publish at their rates (check with `ros2 topic hz`)
- [ ] Works with `use_lidar` true and false
- [ ] Tagged `phase-1`

## Phase 2: Mapping with ground-truth pose

**Goal:** a clean, live 2D occupancy map, built while the position comes from Gazebo's
ground truth. This isolates mapping from localization errors.

**Tasks**

1. `localization.launch.py` with RTAB-Map in mapping mode, taking external
   odometry from `/ground_truth/odom` (RTAB-Map's own odometry stays off in this
   phase).
2. RTAB-Map grid settings in `config/rtabmap.yaml`:
   - 2D grid built from depth points in a height band of 0.2–2.2 m above the floor
     (catches obstacles above and below a LiDAR plane);
   - also uses `/scan` when `use_lidar` is true;
   - resolution 0.05 m; depth range capped at 3 m; unknown cells stay unknown (-1).
3. Map scoring tool (`tools/score_map.py`): compares a saved `/map` against the maze
   generator's ground-truth layout. Reports wall-cell agreement and false walls.
4. Map snapshot tool: saves `/map` to a file, to build offline test cases for Phases 3 and 5.

**Tests:** the scoring tool's own unit tests (a known map scored against itself = perfect).

**Gazebo check:** fly through a maze by keyboard; RViz shows `/map` growing in real time
with straight walls and open doorways.

**Exit criteria**

- [ ] `/map` updates at 1–2 Hz during flight
- [ ] Wall-cell agreement ≥ 90% (initial target; adjust with the team) on 3 mazes
- [ ] No false walls inside corridors or doorways
- [ ] At least 5 saved maps from different seeds, for later phases
- [ ] Tagged `phase-2`

## Phase 3: A* path planner

**Goal:** a fast, safe, deterministic planner on the 2D grid, usable as a library by the explorer.

**Tasks**

1. `core/grid_utils.py`:
   - convert an occupancy grid (+ its resolution and origin) to a numpy array, and
     convert between metres and cells;
   - inflation: block every cell within guarded radius + safety margin of an obstacle;
   - wall cost: from a distance transform, add a cost that rises near walls, so paths
     prefer corridor centres;
   - unknown cells count as blocked.
2. `core/astar.py`:
   - 8-connected search with an octile-distance heuristic (straight + diagonal moves);
   - cost = step length + wall-cost weight × wall cost;
   - path simplification: drop points that have a clear line of sight to a later point;
   - returns a result object: success flag, path in metres, length, cost, and a failure
     reason (start blocked, goal blocked, no path). Never raises on normal failures.
   - If the start cell is inside inflation (drone close to a wall), plan from the nearest free
     cell.
3. `nodes/goal_planner_node.py` (test tool): a goal clicked in RViz ("2D Goal Pose") → A*
   → `/nav/path`.
4. `tools/plot_plan.py`: plot a saved map with its inflation, wall cost and planned path,
   for debugging.

**Tests** (pytest, on synthetic grids and the Phase 2 saved maps):

- straight corridor, L-turn, dead end, unreachable goal, goal inside a wall, start inside
  inflation;
- the path never crosses a blocked or unknown cell;
- in a 1 m corridor the path stays within ±0.1 m of the centre line;
- the same input always gives the same path;
- runtime under 50 ms on a 300 × 300 grid on the dev machine.

**Gazebo check:** with the Phase 2 map live, click goals in RViz; the drawn path is centred,
avoids walls and is replanned for each new goal (the drone does not move yet).

**Exit criteria**

- [ ] All planner tests pass
- [ ] Runtime target met
- [ ] RViz goal test works on 3 mazes
- [ ] Tagged `phase-3`

## Phase 4: Pure pursuit controller

**Goal:** the drone smoothly follows any A* path, holds altitude and heading, and stops safely
when data goes stale.

**Tasks**

1. `core/pure_pursuit.py` (works in the map frame, 2D):
   - find the look-ahead point on the path; look-ahead distance grows with speed
     between a minimum and maximum;
   - command velocity toward that point (holonomic: the drone can move sideways);
   - speed profile: slow down for sharp turns (limit by path curvature) and when
     approaching the goal; limit acceleration;
   - yaw: P-controller toward the target heading (direction of travel, or the viewpoint
     yaw at the goal), with a yaw-rate limit;
   - altitude: P-controller toward the path's z value. The path carries altitude, so take-off
     and landing are just paths with z = 1.2 m or z = 0;
   - goal reached when within the position and yaw tolerances.
2. `nodes/controller_node.py`: reads `/nav/path` and the pose from TF, runs at 30 Hz,
   rotates the velocity into the body frame, publishes `/nav/cmd_vel_raw`.
   - Stale data: if the pose or path is older than its timeout, command zero velocity
     (hover) and log a warning.
3. `nodes/safety_hook_node.py`: copies `/nav/cmd_vel_raw` to `/cmd_vel`. It clamps
   speeds to the configured maximums and applies a simple geofence: no velocity that
   would leave the arena bounds. This is the slot where the CBF filter goes later.
4. Hook `goal_planner_node` + controller together, so RViz goals are flown.

**Tests:**

- pytest with a small point-mass simulator in the test folder: follow a straight path, an L-
  turn and a U-turn; cross-track error stays under 0.10 m at 0.5 m/s; stops within
  tolerance at the goal;
- stale pose or stale path → zero command;
- geofence blocks outward velocity at the boundary.

**Gazebo check:** click goals in RViz; the drone flies the drawn path through corridors and
doorways, turns smoothly, stops at the goal and hovers.

**Exit criteria**

- [ ] All controller tests pass
- [ ] In Gazebo: 20 RViz goals across 3 mazes reached with no wall contact
- [ ] Minimum wall clearance, measured from ground truth, always above the guarded
  radius
- [ ] Take-off to 1.2 m and landing work through paths
- [ ] Tagged `phase-4`

## Phase 5: Exploration and mission state machine

**Goal:** the drone explores the whole maze on its own, makes sure the camera has seen
every free area, returns to the entry and lands.

**Tasks**

1. `core/frontier.py`:
   - map frontiers: free cells next to unknown cells;
   - group them into clusters with a breadth-first search (8-connected); drop clusters
     smaller than a minimum size;
   - each cluster reports its cells, centroid and size.
2. `core/coverage.py`:
   - a coverage grid with the same size and origin as `/map`;
   - each update, cast rays across the camera's 87° horizontal field of view from the
     current pose, up to a coverage range (default 2.5 m); rays stop at walls; every free
     cell a ray passes is marked "seen";
   - view frontiers: free cells not yet seen, clustered the same way as map frontiers.
3. `core/viewpoint.py`:
   - for each cluster (map or view), sample candidate positions in safe free space
     around it, each with a heading facing the cluster;
   - gain = number of target cells (unknown, or unseen) the camera would see from that
     pose (same ray casting);
   - cost = travel cost from the drone, plus a small cost for turning;
   - score = gain weight × gain − cost; choose the best candidate overall;
   - efficiency: run ONE Dijkstra search from the drone per replan. It gives the travel
     cost to every cell at once, so all candidates are costed together; then use A* only for
     the chosen goal;
   - goal commitment: keep the current goal until it is reached, becomes invalid, or a
     new goal beats it by a margin (avoids flip-flopping);
   - stuck rule: no progress toward a goal for a timeout → blacklist it and pick another.
4. `core/mission.py`, the state machine:
   - IDLE → TAKEOFF (on `/mission/start`) → EXPLORE (altitude reached) →
     RETURN_HOME → LAND (at home) → DONE;
   - ABORT from any state (on `/mission/abort`, stale localization, or geofence breach):
     hover, then land in place;
   - EXPLORE → RETURN_HOME when no map or view frontiers remain, OR when
     remaining time minus (home travel time × safety factor + landing time) drops below
     the reserve. Home travel cost comes from a Dijkstra search from the home point,
     refreshed periodically;
   - `/mission/return_home` forces RETURN_HOME (emergency recall);
   - battery is not wired yet: use the elapsed mission time against the 30-minute limit,
     and leave an optional `/battery` (`sensor_msgs/BatteryState`) input for later.
5. Heading modes:
   - `use_lidar` false: face the direction of travel; turn to the viewpoint heading on
     arrival;
   - `use_lidar` true: may start turning toward the viewpoint heading before arrival.
   - Full "look into rooms while flying past" is a later improvement, not part of this
     baseline.
6. `nodes/explorer_node.py`: wires it all to ROS. It replans at 1 Hz, on goal reached, or on a
   large map change. It publishes `/nav/path`, `/nav/goal`, `/nav/frontiers`,
   `/nav/coverage` and `/mission/state`, and serves the three mission services.
7. `tools/grid_sim.py`: a fast pure-Python simulator. It reveals a ground-truth maze
   through a simulated camera (and LiDAR) and runs the explorer loop with no Gazebo.
   Use it to test exploration logic on hundreds of mazes in minutes.

**Tests:**

- frontier detection and clustering on hand-made grids;
- coverage ray casting stops at walls and marks the right cells;
- the chosen viewpoint actually sees its cluster;
- a state-machine transition table test covering every state and trigger, including abort
  from each state;
- the return-budget trigger fires at the right time;
- `grid_sim` over 50 seeds: every room seen, drone ends at home, no blocked or unknown
  cell entered.

**Gazebo check:** `mission.launch.py` with `use_ground_truth:=true`; call
`/mission/start`. In RViz: frontiers shrink, the coverage layer fills, the drone visits every
room, flies home and lands; `/mission/state` ends at DONE.

**Exit criteria**

- [ ] All tests pass, including `grid_sim` on 50 seeds
- [ ] Gazebo: 10 of 10 random mazes completed on ground truth, with and without LiDAR
- [ ] Every survivor model fell inside the camera coverage in every run
- [ ] Abort and recall work from the services mid-mission
- [ ] Mission times recorded as the baseline for Phase 7
- [ ] Tagged `phase-5`

## Phase 6: Real localization (RTAB-Map + EKF)

**Goal:** remove ground truth. The drone estimates its own position from its sensors, exactly
as the real drone will, and still completes the mission.

**Tasks**

1. Odometry from sensors (RTAB-Map):
   - `use_lidar` true: ICP odometry from `/scan` (aligns each laser scan to the previous
     ones; cheap and robust);
   - `use_lidar` false: RGB-D visual odometry from the camera.
2. EKF (`robot_localization`, `config/ekf.yaml`) fuses:
   - RTAB-Map odometry (x, y, yaw and velocities);
   - IMU (angular velocity, acceleration);
   - downward rangefinder for height (convert Range to a z measurement with a tiny
     adapter node).

   It publishes `/odom` and the `odom → base_link` transform. RTAB-Map keeps publishing
   `map → odom` and corrects drift on loop closure.
3. Ground truth is still recorded, but only for scoring. It never feeds the stack when
   `use_ground_truth` is false.
4. Localization-loss handling: if odometry is lost or its covariance explodes, the mission
   state machine makes the drone hover. If it does not recover within a timeout, ABORT.
5. Wall texture option in the maze generator: plain and textured walls. Visual odometry
   needs texture, and the real arena walls may be plain, so test both.
6. `tools/pose_error.py`: compares the estimated pose with ground truth over a run and
   reports drift (absolute trajectory error).
7. Tune RTAB-Map and the EKF for the Raspberry Pi 5 budget: 424×240 at 15 Hz, limited
   feature count. Record CPU use per node.

**Tests:** adapter unit tests; a launch test that checks every contract topic is alive and the TF
chain is complete with ground truth off.

**Gazebo check:** full mission with `use_ground_truth:=false`, run both with and without
LiDAR. In RViz the map stays straight and does not double walls after loops; the drone still
returns to the entry.

**Exit criteria**

- [ ] 10 of 10 missions complete with LiDAR (textured and plain walls)
- [ ] Missions without LiDAR complete on textured walls; results on plain walls documented
  honestly
- [ ] Position error under 0.10 m RMS (initial target)
- [ ] Map agreement still at the Phase 2 target
- [ ] Localization-loss failsafe demonstrated (cover the camera, or drop sensor topics)
- [ ] Tagged `phase-6`

## Phase 7: Validation and benchmarking

**Goal:** evidence that the stack is reliable across many mazes and faults, and a baseline to
measure every future innovation against.

**Tasks**

1. `tools/benchmark.py`: runs Gazebo headless (no window) over a list of seeds and
   modes (`use_lidar` on/off, plain/textured walls), one full mission each, and writes one
   results row per run.
2. Metrics per run:
   - success (DONE at home, no abort);
   - total mission time;
   - map agreement;
   - camera coverage % of free area;
   - survivors inside coverage (of those placed);
   - minimum wall clearance;
   - path length;
   - position error;
   - average CPU per node.
3. Fault injection runs:
   - camera or LiDAR topic dropped for a few seconds;
   - a sudden odometry jump;
   - `/mission/abort` mid-flight;
   - `/mission/return_home` mid-flight;
   - a short time limit, to force a budget-based return;
   - a start next to the geofence.
4. Compute budget: run the stack with the container limited to 2 CPUs to approximate
   the Raspberry Pi 5. If a Pi is available, run once on it for real numbers.
5. `docs/benchmark_report.md`: results table, plots and notes on every failure with its
   cause.

**Targets** (initial, adjust with the team):

| Metric | Target |
|---|---|
| Mission success | ≥ 95% of runs per mode |
| Survivors inside camera coverage | 100% |
| Minimum wall clearance | Always above the guarded radius |
| Fault runs | Each ends in the designed safe behaviour (hover, return or land) |
| CPU | Navigation nodes fit the Pi 5 budget with room left for RTAB-Map |

**Exit criteria**

- [ ] At least 20 seeds per mode benchmarked
- [ ] Targets met, or every miss explained with a fix planned
- [ ] Benchmark report committed
- [ ] Tagged `phase-7`

## Phase 8: Handoff package for the controls team

**Goal:** the controls team can set up, run and test the navigation stack in Gazebo on their
own, and knows exactly how to plug in their own flight model or controller.

**Tasks**

1. `docs/CONTROLS_HANDOFF.md`, covering:
   - **Quick start:** install Docker + NVIDIA container toolkit, clone, open the container,
     build, launch. Copy-paste commands only.
   - **Launch reference:** `mission.launch.py` and its arguments (`seed`, `use_lidar`,
     `use_ground_truth`, `wall_texture`), what each window shows, and how to start,
     abort and recall a mission.
   - **Integration options:**
     - A: use our simple drone model as is;
     - B: replace the drone model with theirs. It must accept body-frame
       TwistStamped on `/cmd_vel` and publish the sensor topics and TF in the
       Interface contract;
     - C: insert their own controller between `/cmd_vel` and their dynamics model.
   - **Parameter reference:** link to the table in this document.
   - **Troubleshooting:** GPU or display issues, TF errors, the drone not moving, map not
     growing.
   - **Known limitations:** no CBF safety filter yet; no survivor detection or tagging;
     battery not wired (time-based only); flight-controller protocol not decided; visual-
     only mode weak on plain walls (from Phase 7 results).
2. `docs/TEST_PROCEDURE.md`: numbered scenarios, each with steps and expected result:

   | # | Scenario | Expected result |
   |---|---|---|
   | T1 | Launch sim, take off, keyboard flight | Drone hovers at 1.2 m and responds correctly |
   | T2 | RViz goal in a fixed maze | Path drawn, flown without wall contact |
   | T3 | Full mission, LiDAR on | All rooms covered, return home, DONE |
   | T4 | Full mission, LiDAR off | Same as T3 (textured walls) |
   | T5 | Abort mid-flight | Hover, then land in place; state ABORT |
   | T6 | Recall mid-flight | Flies home and lands |
   | T7 | Short time limit | Returns home early, by budget |
   | T8 | Three random seeds | Each completes; results logged by the benchmark tool |

3. One-command launch verified from a fresh clone.
4. Release: changelog, version `v0.1.0` git tag, the benchmark report attached.
5. Dry run: someone outside the automation team follows the handoff doc on a clean
   machine; fix every place they got stuck.

**Exit criteria**

- [ ] Dry run completed with no help from the automation team
- [ ] T1–T8 pass when run by the dry-run person
- [ ] `v0.1.0` tagged and shared with the controls team

## Parameters reference

Starting defaults. All live in `airmouse_bringup/config/`. Tune in simulation, record
changes in the benchmark report.

| Parameter | Default | File | Effect |
|---|---|---|---|
| `use_lidar` | true | launch arg | Use the 2D LiDAR for odometry, mapping and heading mode |
| `use_ground_truth` | false | launch arg | Use Gazebo's exact pose (Phases 2–5 only) |
| `flight_altitude` | 1.2 m | controller.yaml | Cruise height |
| `guarded_radius` | 0.30 m | planner.yaml | Drone radius including prop guards |
| `safety_margin` | 0.10 m | planner.yaml | Extra clearance added to inflation |
| `grid_resolution` | 0.05 m | rtabmap.yaml | Map cell size |
| `grid_height_band` | 0.2–2.2 m | rtabmap.yaml | Depth points used to build the 2D map |
| `wall_cost_weight` | 2.0 | planner.yaml | How strongly paths prefer corridor centres |
| `max_speed` | 0.6 m/s | controller.yaml | Horizontal speed limit (raise after Phase 7) |
| `max_accel` | 0.5 m/s² | controller.yaml | Acceleration limit |
| `max_yaw_rate` | 0.8 rad/s | controller.yaml | Turn-rate limit (protects visual tracking) |
| `lookahead_min` / `lookahead_max` | 0.3 / 0.8 m | controller.yaml | Pure pursuit look-ahead range |
| `goal_tolerance_xy` / `goal_tolerance_yaw` | 0.15 m / 10° | controller.yaml | When a goal counts as reached |
| `pose_timeout` / `path_timeout` | 0.3 s / 2.0 s | controller.yaml | Stale-data hover triggers |
| `control_rate` | 30 Hz | controller.yaml | Controller loop rate |
| `replan_period` | 1.0 s | explorer.yaml | Exploration replanning rate |
| `camera_hfov` | 87° | explorer.yaml | Camera horizontal field of view for coverage |
| `coverage_range` | 2.5 m | explorer.yaml | How far the camera counts as "seeing" |
| `min_cluster_size` | 0.3 m of cells | explorer.yaml | Smaller frontier clusters are ignored |
| `gain_weight` | 1.0 | explorer.yaml | Information gain vs travel cost trade-off |
| `goal_switch_margin` | 20% | explorer.yaml | How much better a new goal must be to switch |
| `stuck_timeout` | 10 s | explorer.yaml | No progress → blacklist goal |
| `mission_time_limit` | 30 min | explorer.yaml | Competition limit |
| `return_reserve` | 2 min | explorer.yaml | Spare time kept when deciding to return |
| `return_safety_factor` | 1.5 | explorer.yaml | Multiplier on estimated home travel time |
| `localization_lost_timeout` | 3 s | explorer.yaml | Hover, then ABORT if not recovered |
| `arena_bounds` | From the maze generator (sim); measured on site (real) | explorer.yaml | Geofence used by the safety hook |

## Open decisions and risks

| Item | Why it matters | Owner | Needed by |
|---|---|---|---|
| LiDAR fitted or not | Decides the main localization mode and heading behaviour | Electronics | Phase 6 |
| Flight-controller link protocol (MAVLink or custom) | Defines the real `/cmd_vel` bridge | Controls + electronics | After Phase 8 (real hardware) |
| Downward rangefinder model | Height input to the EKF | Electronics | Phase 6 |
| Final guarded radius and mass | Inflation margin and corridor clearance | Mechanical | Phase 4 |
| Real arena wall texture | Visual-only odometry is weak on plain walls | Team (ask organizers) | Phase 6 |
| Battery signal | Battery-based return instead of time-based | Electronics | After Phase 8 |
| Ground-station map format | Teammate 3 displays `/map` and survivor tags | Teammate 3 | Phase 5 |

### Main technical risks

- **Localization drift without LiDAR.** Mitigation: test plain walls early (Phase 6); fall back
  to slower speed and limited yaw rate.
- **Raspberry Pi 5 CPU budget.** RTAB-Map is the heaviest part. Mitigation: profile in Phase
  7, lower camera rate or resolution, prefer LiDAR odometry.
- **Sim-to-real gap.** Mitigation: realistic sensor noise and model mass in Phase 1; keep
  every value a parameter.

**Later improvements** (after v0.1.0, not baseline): CBF safety filter; "look into rooms while
flying past"; FALCON-style global visit order to cut revisits; using the known room and
corridor sizes to snap the map to the arena grid; battery-based return.
