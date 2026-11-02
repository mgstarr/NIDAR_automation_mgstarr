# airmouse_nav

Navigation for the NIDAR AirMouse drone: frontier exploration with camera coverage,
viewpoint selection, A* planning and pure-pursuit control.

## Layout

- `airmouse_nav/core/`: pure algorithms in Python + numpy. **No ROS imports** (enforced by
  `test/test_scaffold.py`). `grid_utils`, `astar`, `frontier`, `coverage`, `viewpoint`,
  `mission`, `pure_pursuit`.
- `airmouse_nav/nodes/`: thin ROS 2 wrappers: `explorer_node`, `controller_node`,
  `safety_hook_node` (pass-through now, CBF later), `goal_planner_node` (test tool).
- `airmouse_nav/bridges/`: `sim_bridge` (adapter to the Gazebo velocity plugin) and
  `fc_bridge` (hook for the real flight controller; protocol not decided).
- `test/`: pytest unit tests for every `core/` module.

## Run

```bash
# from the workspace root, inside the dev container
colcon build --symlink-install
source install/setup.bash
ros2 run airmouse_nav explorer_node
```

## Test

```bash
pytest src/airmouse_nav/test -v                 # fast, no ROS needed
colcon test --packages-select airmouse_nav && colcon test-result --verbose
```

## Topics

Filled in as the nodes are implemented (Phases 3–5). Planned, from `docs/PLAN.md`:

| Node | Subscribes | Publishes |
|---|---|---|
| `goal_planner_node` | `/map`, `/goal_pose` (RViz), TF | `/nav/path` |
| `explorer_node` | `/map`, TF | `/nav/path`, `/nav/goal`, `/nav/frontiers`, `/nav/coverage`, `/mission/state`; serves `/mission/start`, `/mission/abort`, `/mission/return_home` |
| `controller_node` | `/nav/path`, TF | `/nav/cmd_vel_raw` (30 Hz) |
| `safety_hook_node` | `/nav/cmd_vel_raw` | `/cmd_vel` (30 Hz) |
