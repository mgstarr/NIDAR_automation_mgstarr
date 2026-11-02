"""Camera-coverage layer: which free cells the forward camera has actually seen.

Planned (Phase 5):
  - a coverage grid with the same size and origin as /map;
  - each update, cast rays across the camera's horizontal field of view from the current
    pose, up to the coverage range; rays stop at walls; every free cell a ray passes is
    marked "seen";
  - view frontiers: free cells not yet seen, clustered the same way as map frontiers.
"""
