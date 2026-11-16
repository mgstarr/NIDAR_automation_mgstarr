"""Downward rangefinder -> height of base_link above the floor (pure Python, no ROS).

    z = range_to_z(range_m, min_range, max_range, sensor_offset, roll, pitch)

- A valid reading gives z = range x cos(roll) x cos(pitch) + sensor_offset, where
  sensor_offset is how far the sensor sits below base_link (0.06 m on this drone) and the
  cosines correct for the tilted beam (pass 0, 0 when the tilt is unknown).
- -inf (REP 117: closer than min_range, i.e. standing on the floor) gives z = sensor_offset +
  min_range / 2: the drone is on the ground.
- +inf, NaN or a value outside [min_range, max_range] gives None (no measurement).
"""

from __future__ import annotations

import math


def range_to_z(range_m: float, min_range: float, max_range: float, sensor_offset: float = 0.06,
               roll: float = 0.0, pitch: float = 0.0) -> float | None:
    """Height of base_link above the floor from one rangefinder reading, or None."""
    if range_m == -math.inf:
        return sensor_offset + min_range / 2.0
    if math.isnan(range_m) or math.isinf(range_m) or not min_range <= range_m <= max_range:
        return None
    return range_m * math.cos(roll) * math.cos(pitch) + sensor_offset
