"""Simulation adapter maths: our body-frame /cmd_vel -> the Gazebo velocity plugin's command,
and single-beam LaserScan -> Range value. Pure Python, no ROS (used by sim_bridge_node).

Plugin frame convention (measured in Phase 1, see docs/handoff/phase-1.md): Gazebo's
MulticopterVelocityControl reads linear velocity in the vehicle heading (body) frame, x forward,
y left, z up, and angular.z as yaw rate. So with plugin_frame='body' the command passes through.
'world' is kept for plugins that expect world-frame velocity.
"""

import math
from typing import NamedTuple

PLUGIN_FRAMES = ('body', 'world')


class VelocityCommand(NamedTuple):
    """Velocity + yaw-rate command: m/s and rad/s."""

    vx: float
    vy: float
    vz: float
    yaw_rate: float


def body_to_plugin(cmd: VelocityCommand, yaw: float, plugin_frame: str) -> VelocityCommand:
    """Convert a body-frame (REP 103) command to the frame the plugin expects.

    Args:
        cmd: Command in base_link: x forward, y left, z up, yaw rate about z.
        yaw: Current heading of base_link in the world frame [rad] (used for 'world').
        plugin_frame: 'body' (pass-through) or 'world' (rotate x/y by yaw).

    Returns:
        The command in the plugin's frame. z velocity and yaw rate never change.
    """
    if plugin_frame == 'body':
        return cmd
    if plugin_frame != 'world':
        raise ValueError(f'plugin_frame must be one of {PLUGIN_FRAMES}, got {plugin_frame!r}')
    c, s = math.cos(yaw), math.sin(yaw)
    return VelocityCommand(c * cmd.vx - s * cmd.vy, s * cmd.vx + c * cmd.vy, cmd.vz, cmd.yaw_rate)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Heading (rotation about z) of a quaternion [rad]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def beam_range(ranges: list[float], range_min: float, range_max: float) -> float:
    """Range of a single-beam scan, using sensor_msgs/Range conventions (REP 117).

    Returns:
        The measured distance; +inf when nothing is within range_max; -inf when the target
        is closer than range_min.
    """
    if not ranges or math.isnan(ranges[0]):
        return math.inf
    r = ranges[0]
    if r < range_min:
        return -math.inf
    return r if r <= range_max else math.inf
