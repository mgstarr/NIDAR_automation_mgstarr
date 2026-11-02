"""Occupancy-grid helpers shared by the planner and the explorer.

Planned (Phase 3):
  - convert an occupancy grid (+ its resolution and origin) to a numpy array,
    and convert between metres and cells;
  - inflation: block every cell within guarded radius + safety margin of an obstacle;
  - wall cost: from a distance transform, a cost that rises near walls, so paths
    prefer corridor centres;
  - unknown cells count as blocked.
"""
