# Handoff: Phase 0 → Phase 1

Phase 0 (Environment and Repository) is complete. Phase 1 receives a working dev container,
a buildable empty workspace and passing placeholder tests.

## Exit criteria

- [x] Container builds and opens from VS Code ("Reopen in Container") and from `docker compose`
- [x] Gazebo (`gz sim shapes.sdf`) and RViz (`rviz2`) windows open from inside the container
- [x] `colcon build` and `pytest` pass (29 passed, 0 skipped; `colcon test` also 29/0/0)
- [x] `ros2 pkg list` shows `airmouse_bringup`, `airmouse_description`, `airmouse_nav`
- [x] Handoff note written, committed, tagged `phase-0`

## Commands

Repo root (= git root = ROS 2 workspace root) on the host:
`/home/mgstarr/Desktop/NIDAR/codes/codes`. Inside the container it is mounted at `/ws`.

**Open the container (VS Code):** open the repo root folder → *Ctrl+Shift+P → Dev Containers:
Reopen in Container*. `initializeCommand` runs `xhost` and writes `.devcontainer/.env` for you.

**Open the container (no VS Code):**

```bash
xhost +SI:localuser:$(id -un)
echo RENDER_GID=$(getent group render | cut -d: -f3) > .devcontainer/.env
docker compose -f .devcontainer/docker-compose.yml up -d --build
docker compose -f .devcontainer/docker-compose.yml exec dev bash
```

**Build and test (inside the container):**

```bash
cd /ws
colcon build --symlink-install
source install/setup.bash
pytest src/airmouse_nav/test -v
ros2 pkg list | grep airmouse
```

**Graphics check:** `gz sim shapes.sdf` and
`rviz2 -d /ws/install/airmouse_bringup/share/airmouse_bringup/rviz/airmouse.rviz`.

## Image and versions

| Item | Value |
|---|---|
| Image name | `airmouse-dev:jazzy` (container `airmouse-dev`, compose project `airmouse`) |
| Base image | `osrf/ros:jazzy-desktop-full` (Ubuntu 24.04) |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (gz-sim 8.11.0), from the `ros-jazzy-ros-gz` vendor packages |
| ros_gz | 1.0.24 |
| rtabmap_ros | 0.23.13 |
| robot_localization | 3.8.3 |
| Container user | `ubuntu` (UID 1000, passwordless sudo) |
| GPU in container | NVIDIA GeForce GTX 1650, OpenGL direct rendering via PRIME offload |

## Deviations from the phase document / PLAN.md, and why

1. **Dev machine** is Ubuntu 26.04 with a GTX 1650 Mobile (hybrid Intel + NVIDIA laptop), not
   Ubuntu 22.04 with an RTX 4060. The container hides the OS difference. Hybrid graphics
   needs `__NV_PRIME_RENDER_OFFLOAD=1` and `__GLX_VENDOR_LIBRARY_NAME=nvidia` (set in
   `docker-compose.yml`). The GTX 1650 is slower; Gazebo performance with camera sensors
   is untested until Phase 1.
2. **Compose file location:** `.devcontainer/docker-compose.yml`, and `devcontainer.json`
   uses it, so VS Code and plain compose run the identical setup.
3. **Host networking and IPC** (`network_mode: host`, `ipc: host`): ROS 2 discovery behaves
   as on the host, and Qt/X11 shared memory works.
4. **`RENDER_GID`:** the host's `render` group id (992 here) differs from the container's
   (997), which blocked `/dev/dri`. It is passed in via `.devcontainer/.env` (machine-specific,
   written by `initializeCommand`). Added `.devcontainer/.env` to `.gitignore`, the only
   addition to the plan's list.
5. **`use_sim_time` launch argument** is declared in every launch file (default `true` in
   `sim.launch.py`, `false` elsewhere). It is not in PLAN.md's parameter table but is the
   standard ROS 2 switch to follow the simulator clock; no contract name conflicts.
6. **`planner.yaml`** uses the `/**` node wildcard: the planner is a library used by both
   `goal_planner_node` and `explorer_node`, so the file applies to whichever node loads it.
7. **`bridge.yaml`** is an empty YAML list (`[]`), the `ros_gz_bridge` format, not a
   `ros__parameters` file; `test_scaffold.py` checks it separately.
8. **Old native scaffold** was kept and aligned with PLAN.md (owner's instruction: the plan
   wins): added `core/viewpoint.py`, `nodes/safety_hook_node.py`,
   `nodes/goal_planner_node.py`, `config/bridge.yaml`, `launch/rviz.launch.py`,
   `rviz/airmouse.rviz`, `models/airmouse/`, `tools/`; emptied the configs (old parameter
   names such as `robot_radius_m` removed); `use_lidar` default changed to `true`;
   "Stage" wording replaced with phase numbers. `bridges/` keeps `sim_bridge.py` (Phase 1
   adapter) and `fc_bridge.py` (hook for the real flight controller, out of scope).
9. **Placeholder tests** import each `core/` module and check its docstring (they pass
   rather than skip).

## Open issues

- Harmless `libEGL warning: ... driver (null)` lines when Gazebo starts: Mesa probes the
  NVIDIA device before NVIDIA's own driver is used. Rendering is on the GTX 1650.
- `mesa-utils` (`glxinfo`) is not in the image; install with `sudo apt-get install mesa-utils`
  if a graphics check is needed.
- Phase 1 must check Gazebo sensor rendering (depth camera) performance on the GTX 1650.
- The original PDFs live outside the repo (`/home/mgstarr/Desktop/NIDAR/codes/`);
  `docs/PLAN.md` and `docs/phases/phase-0.md` are Markdown exports of them.
