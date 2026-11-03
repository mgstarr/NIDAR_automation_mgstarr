# airmouse_description

Simulation assets for the NIDAR AirMouse drone.

- `models/airmouse/model.sdf`: the drone. X3-style quadcopter (rotor and velocity-controller
  values from Gazebo Harmonic's `multicopter_velocity_control.sdf`), 1.6 kg, collision radius
  0.30 m. Sensors: RGB-D camera + IMU (`camera_link`), optional 2D LiDAR (`lidar_link`),
  downward/upward single-beam rangefinders (`range_down_link`, `range_up_link`), ground-truth
  odometry. Built from simple shapes, so it needs no internet.
- `scripts/generate_maze.py`: random, rule-checked maze from a seed.
- `worlds/generated/seed_<N>_<texture>/`: generated mazes (not in git; regenerated on launch).

## Generate a maze

```bash
ros2 run airmouse_description generate_maze.py --seed 3 --wall-texture textured
```

Writes `world.sdf`, `layout.pgm` + `layout.yaml` (ground-truth occupancy, 0.05 m cells),
`layout.json` (entry, arena bounds, rooms, survivors) and, for textured walls, `wall_*.png`.

Rules (asserted in code, tested over 50 seeds in `test/`): 13 x 13 cells of 1.0 m clear
space with 0.10 m walls (arena 14.4 m), 6-8 rooms of 2 x 2 cells with one door each, walls
2.44 m with a net ceiling, one entry on the west wall, up to 6 survivors in distinct rooms.
World frame = map frame: origin at the entry cell centre (take-off point), +x into the maze.

## Topics

None. The simulation launch lives in `airmouse_bringup` (`sim.launch.py`).
