"""Map frontier detection and clustering.

Planned (Phase 5):
  - map frontiers: free cells next to unknown cells;
  - group them into clusters with an 8-connected breadth-first search;
    drop clusters smaller than a minimum size;
  - each cluster reports its cells, centroid and size.
"""
