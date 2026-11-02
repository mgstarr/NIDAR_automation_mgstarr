# airmouse_description

Simulation assets for the NIDAR AirMouse drone:

- `models/airmouse/`: the drone model, based on Gazebo Harmonic's X3 multicopter
  velocity-control example: depth camera, optional 2D LiDAR, IMU, downward and upward
  rangefinders. *(Phase 1)*
- `worlds/`: generated Gazebo Harmonic maze worlds. *(Phase 1)*
- `scripts/generate_maze.py`: random maze generator (input: a seed) that follows the
  competition rules. *(Phase 1)*

## Run

```bash
ros2 run airmouse_description generate_maze.py --help
```

## Topics

None. This package only holds files. The simulation launch lives in `airmouse_bringup`.
