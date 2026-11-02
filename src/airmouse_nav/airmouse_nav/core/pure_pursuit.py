"""Holonomic 2D pure-pursuit path follower (map frame).

Planned (Phase 4):
  - look-ahead point on the path; look-ahead distance grows with speed between a
    minimum and maximum;
  - velocity toward that point (the drone can move sideways);
  - speed profile: slow down for sharp turns and near the goal; limit acceleration;
  - yaw: P-controller toward the target heading, with a yaw-rate limit;
  - altitude: P-controller toward the path's z value (take-off and landing are paths);
  - goal reached when within the position and yaw tolerances.
"""
