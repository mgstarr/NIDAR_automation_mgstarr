"""Safety hook node: /nav/cmd_vel_raw -> speed clamp + geofence -> /cmd_vel.

######################################################################################
#  CBF SLOT: this node is where the future control-barrier-function (CBF) obstacle   #
#  filter goes. Today it only clamps speeds and applies the arena geofence.          #
#  Keep its topics (/nav/cmd_vel_raw in, /cmd_vel out) when replacing the filter.    #
######################################################################################

Input and output: geometry_msgs/TwistStamped in the body frame (base_link).
Pose for the geofence: TF map -> base_link. No pose -> horizontal and vertical velocity are
zeroed (yaw rate kept), with a throttled warning.

Geofence: `arena_bounds` = [xmin, xmax, ymin, ymax] (map frame). Once the drone's centre is
within `guarded_radius` of a bound, the velocity component pointing out of the arena is
removed; motion along or back into the arena is kept.
Clamps: horizontal speed <= max_speed (direction kept), |vz| <= max_vertical_speed,
|yaw rate| <= max_yaw_rate.
"""

from __future__ import annotations

import math
from typing import Sequence

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from airmouse_nav.core.pure_pursuit import body_to_world, world_to_body

NODE_NAME = 'safety_hook_node'


def clamp_command(vx: float, vy: float, vz: float, yaw_rate: float, max_speed: float,
                  max_vertical_speed: float, max_yaw_rate: float) -> tuple[float, float, float, float]:
    """Limit horizontal speed (keeping direction), vertical speed and yaw rate."""
    speed = math.hypot(vx, vy)
    if speed > max_speed > 0:
        vx, vy = vx * max_speed / speed, vy * max_speed / speed
    vz = max(-max_vertical_speed, min(max_vertical_speed, vz))
    yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))
    return vx, vy, vz, yaw_rate


def geofence(x: float, y: float, vx: float, vy: float, bounds: Sequence[float],
             margin: float) -> tuple[float, float]:
    """Remove map-frame velocity components that point out of the arena near a bound.

    Args:
        x, y: Drone position (map frame).
        vx, vy: Map-frame velocity.
        bounds: [xmin, xmax, ymin, ymax].
        margin: Distance from a bound at which outward motion is removed.
    """
    xmin, xmax, ymin, ymax = bounds
    if x >= xmax - margin and vx > 0:
        vx = 0.0
    if x <= xmin + margin and vx < 0:
        vx = 0.0
    if y >= ymax - margin and vy > 0:
        vy = 0.0
    if y <= ymin + margin and vy < 0:
        vy = 0.0
    return vx, vy


def filter_body_command(x: float, y: float, yaw: float, bx: float, by: float,
                        bounds: Sequence[float], margin: float) -> tuple[float, float]:
    """Geofence a body-frame horizontal velocity, given the pose; returns body frame."""
    wx, wy = body_to_world(bx, by, yaw)
    wx, wy = geofence(x, y, wx, wy, bounds, margin)
    return world_to_body(wx, wy, yaw)


class SafetyHookNode(Node):
    """Pass-through with clamps and geofence (CBF slot)."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        d = self.declare_parameter
        self.max_speed = d('max_speed', 0.6).value
        self.max_vertical_speed = d('max_vertical_speed', 0.3).value
        self.max_yaw_rate = d('max_yaw_rate', 0.8).value
        self.margin = d('guarded_radius', 0.30).value
        self.bounds = list(d('arena_bounds', [-0.6, 13.8, -7.2, 7.2]).value)
        self.map_frame = d('map_frame', 'map').value
        self.robot_frame = d('robot_frame', 'base_link').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(TwistStamped, '/nav/cmd_vel_raw', self.on_cmd, 10)
        self.get_logger().info(f'ready: geofence {self.bounds}, margin {self.margin} m')

    def on_cmd(self, msg: TwistStamped) -> None:
        """Filter one command and publish it on /cmd_vel."""
        t = msg.twist
        vx, vy, vz, wz = clamp_command(t.linear.x, t.linear.y, t.linear.z, t.angular.z,
                                       self.max_speed, self.max_vertical_speed,
                                       self.max_yaw_rate)
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
            p, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            vx, vy = filter_body_command(p.x, p.y, yaw, vx, vy, self.bounds, self.margin)
        except TransformException:
            if vx or vy or vz:
                self.get_logger().warn('no pose for the geofence: holding position',
                                       throttle_duration_sec=2.0)
            vx = vy = vz = 0.0
        out = TwistStamped()
        out.header = msg.header
        out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = vx, vy, vz
        out.twist.angular.z = wz
        self.pub.publish(out)


def main(args: list[str] | None = None) -> None:
    """Run the safety hook node until shutdown.

    Args:
        args: Command-line arguments forwarded to rclpy.
    """
    rclpy.init(args=args)
    node = SafetyHookNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
