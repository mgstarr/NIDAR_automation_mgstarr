# tools

Offline helper scripts that are not ROS nodes. Added phase by phase (see `docs/PLAN.md`):

| Script | Phase | Purpose |
|---|---|---|
| `takeoff.py` | 1 | Sim test tool: climb to 1.2 m on `/cmd_vel` using ground truth, then hover |
| `score_map.py` | 2 | Compare a saved `/map` with the maze generator's ground truth |
| `plot_plan.py` | 3 | Plot a map with its inflation, wall cost and planned path |
| `grid_sim.py` | 5 | Fast pure-Python exploration simulator (no Gazebo) |
| `pose_error.py` | 6 | Estimated pose vs ground truth (drift) |
| `benchmark.py` | 7 | Headless Gazebo runs over many seeds and modes |
