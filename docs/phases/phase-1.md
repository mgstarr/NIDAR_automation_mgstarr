# Phase 1 – Simulation World and Drone

Nov 2, 2026 · @Mahima

> Markdown export of the PDF *Phase 1 – Simulation World and Drone*.

**Goal:** a realistic Gazebo test arena and a simulated drone that publishes exactly the topics
the real drone will (see *PLAN.md › Interface contract*).

## Before you start (entry checklist)

- [ ] Tag `phase-0` exists; `docs/handoff/phase-0.md` read
- [ ] Container opens; `colcon build` and `pytest` pass
- [ ] `gz sim shapes.sdf` and `rviz2` open from inside the container

## Working rules

Same as every phase: read `docs/PLAN.md` + this file; explain the approach and wait for
approval; one module at a time with tests; ask before inventing values; never rename
contract topics or parameters without approval.

## Tasks

1. **Maze generator** `airmouse_description/scripts/generate_maze.py` (input: random
   seed; options: `wall_texture` plain/textured, `num_survivors`):
   - lay out a grid of 1 m cells inside at most 15 m × 15 m;
   - carve corridors with a randomized depth-first search, then remove a few extra walls
     so the maze has loops;
   - place 2 m × 2 m rooms (target 6–8), each reachable from a corridor;
   - walls 2.44 m tall; one entry/exit opening on the outer wall;
   - place up to 6 human-sized survivor models inside rooms;
   - output: an SDF world file, plus a ground-truth occupancy image + YAML of the
     layout and a JSON list of room and survivor positions (used for scoring in later
     phases);
   - assert in code: every room reachable, every corridor at least 1 m wide, arena within
     bounds.
2. **Drone model** `airmouse_description/models/airmouse/`, based on Gazebo
   Harmonic's multicopter velocity-control example (X3 quadcopter):
   - moves on velocity + yaw-rate commands;
   - mass about 1.6 kg; collision radius = `guarded_radius` (default 0.30 m);
   - a small adapter in `airmouse_nav/bridges/` converts our body-frame
     `TwistStamped` on `/cmd_vel` to what the plugin expects. Verify the plugin's frame
     convention by test.
3. **Sensors on the model**, matching the real hardware:
   - RGB-D camera: 87° horizontal field of view, 424×240, 15 Hz, depth 0.3–3 m, with
     noise;
   - optional 2D LiDAR: 360°, 0.05–12 m, 10 Hz, ±3 cm noise (on/off by `use_lidar`);
   - IMU with noise;
   - downward and upward single-beam rangefinders publishing `sensor_msgs/Range`;
   - ground-truth odometry publisher (`/ground_truth/odom`).
4. **Bridge config** `airmouse_bringup/config/bridge.yaml`: every Gazebo topic mapped
   to the contract names.
5. **Launch** `airmouse_bringup/launch/sim.launch.py` with arguments `seed`,
   `use_lidar`, `use_ground_truth`, `wall_texture`; plus `rviz/airmouse.rviz`.

## Tests

- Maze generator unit tests over 50 seeds: connectivity, corridor width, room count,
  arena bounds, survivors inside rooms
- Adapter unit test: body-frame command converts correctly for several yaw angles

## Gazebo check

Launch the sim. Command a take-off to 1.2 m, then drive with `teleop_twist_keyboard`.
RViz shows the depth point cloud, the LiDAR scan (when on), the TF tree and the ground-
truth odometry.

## Exit criteria

- [ ] Maze generator tests pass for 50 seeds
- [ ] Drone hovers at 1.2 m and moves in the commanded direction (forward, left, up, yaw
  left)
- [ ] Every contract sensor topic publishes at its rate (`ros2 topic hz`)
- [ ] Works with `use_lidar` true and false, and with plain and textured walls
- [ ] Handoff note written, committed, tagged `phase-1`

## Handoff to Phase 2

Write `docs/handoff/phase-1.md` with:

- how to generate a maze and launch the sim (exact commands and arguments);
- where generated worlds and their ground-truth layout files are stored;
- measured topic rates and the adapter's frame convention;
- deviations from this document, and open issues.

Phase 2 receives: a launchable simulation with all contract sensor topics, ground-truth
odometry, and ground-truth layout files for scoring maps.

**Start prompt for Phase 2** (paste into Claude Code):

```
Read docs/PLAN.md, docs/phases/phase-2.md and docs/handoff/phase-1.md.
Run the Phase 2 entry checklist and report the result.
Then explain your approach for Phase 2 and wait for my approval.
```
