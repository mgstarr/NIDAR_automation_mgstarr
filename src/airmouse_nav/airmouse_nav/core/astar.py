"""A* path planner on the 2D grid.

Planned (Phase 3):
  - 8-connected search with an octile-distance heuristic (straight + diagonal moves);
  - cost = step length + wall-cost weight x wall cost;
  - path simplification: drop points that have a clear line of sight to a later point;
  - returns a result object: success flag, path in metres, length, cost and a failure
    reason (start blocked, goal blocked, no path); never raises on normal failures;
  - if the start cell is inside inflation, plan from the nearest free cell.
"""
