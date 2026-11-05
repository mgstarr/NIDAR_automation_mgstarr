# tools

Offline helper scripts that are not ROS nodes. Added phase by phase (see `docs/PLAN.md`):

| Script | Phase | Purpose |
|---|---|---|
| `takeoff.py` | 1 | Sim test tool: climb to 1.2 m on `/cmd_vel` using ground truth, then hover |
| `map_io.py` | 2 | Read/write maps as PGM + YAML (map_server format) |
| `save_map.py` | 2 | Save the live `/map` into `src/airmouse_nav/test/data/maps/` |
| `score_map.py` | 2 | Compare a saved map with the maze generator's ground truth |
| `survey_flight.py` | 2 | Sim test tool: fly a tour of every maze cell (true layout + pose) |
| `plot_plan.py` | 3 | Plot a map with its inflation, wall cost and planned path |
| `grid_sim.py` | 5 | Fast pure-Python exploration simulator (no Gazebo) |
| `pose_error.py` | 6 | Estimated pose vs ground truth (drift) |
| `benchmark.py` | 7 | Headless Gazebo runs over many seeds and modes |
