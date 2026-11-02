"""Viewpoint selection: where the drone should fly next, and which way it should face.

Planned (Phase 5):
  - for each cluster (map or view), sample candidate positions in safe free space around
    it, each with a heading facing the cluster;
  - gain = number of target cells (unknown, or unseen) the camera would see from that pose;
  - cost = travel cost from the drone, plus a small cost for turning;
  - score = gain weight x gain - cost; choose the best candidate overall;
  - one Dijkstra search from the drone per replan costs all candidates at once;
    A* is then used only for the chosen goal;
  - goal commitment (switch only when a new goal beats the current one by a margin)
    and a stuck rule (no progress for a timeout -> blacklist the goal).
"""
