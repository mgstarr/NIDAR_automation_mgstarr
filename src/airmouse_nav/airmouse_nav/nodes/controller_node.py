"""Controller node: /nav/path + TF (map -> base_link) -> pure pursuit -> /nav/cmd_vel_raw.

Runs at `control_rate`. The command is a body-frame geometry_msgs/TwistStamped (frame
base_link; linear x/y/z in m/s, angular.z = yaw rate). Path z is the target altitude; the yaw
of the path's last pose is the goal yaw (ignored if that orientation is all zeros). An empty path means "stop and hover".

Stale data: pose older than `pose_timeout`, or (while following) path older than
`path_timeout` -> zero command and a throttled warning.
Also publishes /nav/controller_state (std_msgs/String: idle | following | reached | stale:<why>)
for test tools.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from airmouse_nav.core.pure_pursuit import (Pose, PurePursuit, PursuitParams, is_stale,
                                            world_to_body)

NODE_NAME = 'controller_node'


def yaw_from_quaternion(q) -> float:
    """Yaw (rad) of a geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def path_goal_yaw(msg: Path) -> float | None:
    """Goal yaw from the last pose, or None if its orientation is unset (all zeros)."""
    q = msg.poses[-1].pose.orientation
    if q.x == q.y == q.z == q.w == 0.0:
        return None
    return yaw_from_quaternion(q)


class ControllerNode(Node):
    """Pure pursuit at the control rate."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        d = self.declare_parameter
        self.params = PursuitParams(
            flight_altitude=d('flight_altitude', 1.2).value,
            max_speed=d('max_speed', 0.6).value,
            max_accel=d('max_accel', 0.5).value,
            max_yaw_rate=d('max_yaw_rate', 0.8).value,
            lookahead_min=d('lookahead_min', 0.3).value,
            lookahead_max=d('lookahead_max', 0.8).value,
            goal_tolerance_xy=d('goal_tolerance_xy', 0.15).value,
            goal_tolerance_yaw=math.radians(d('goal_tolerance_yaw', 10.0).value),
            yaw_gain=d('yaw_gain', 1.5).value,
            altitude_gain=d('altitude_gain', 1.0).value,
            max_vertical_speed=d('max_vertical_speed', 0.3).value,
            max_yaw_accel=d('max_yaw_accel', 0.5).value)
        self.pose_timeout = d('pose_timeout', 0.3).value
        self.path_timeout = d('path_timeout', 2.0).value
        rate = d('control_rate', 30.0).value
        self.map_frame = d('map_frame', 'map').value
        self.robot_frame = d('robot_frame', 'base_link').value

        self.pp = PurePursuit(self.params)
        self.path_time: float | None = None
        self.last_state = ''
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Path, '/nav/path', self.on_path, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/nav/cmd_vel_raw', 10)
        self.state_pub = self.create_publisher(String, '/nav/controller_state', 10)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.step)
        self.get_logger().info(f'ready: {rate:.0f} Hz, max_speed {self.params.max_speed} m/s')

    def now(self) -> float:
        """Current ROS time in seconds (sim time in simulation)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_path(self, msg: Path) -> None:
        """Start following a new path (empty = stop)."""
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.path_time = stamp if stamp > 0 else self.now()
        if not msg.poses:
            self.pp.clear()
            return
        pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        self.pp.set_path(pts, path_goal_yaw(msg))

    def step(self) -> None:
        """One control cycle."""
        now = self.now()
        pose, pose_time = None, None
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
            pose_time = Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
            t = tf.transform.translation
            pose = Pose(t.x, t.y, t.z, yaw_from_quaternion(tf.transform.rotation))
        except TransformException:
            pass
        stale = is_stale(now, pose_time, self.path_time, self.pose_timeout, self.path_timeout,
                         self.pp.active)
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.robot_frame
        if stale:
            state = f'stale:{stale}'
            if stale == 'path':
                self.pp.clear()
            self.get_logger().warn(f'{stale} data is stale: hovering',
                                   throttle_duration_sec=2.0)
        else:
            c = self.pp.update(pose, self.dt)
            bx, by = world_to_body(c.vx, c.vy, pose.yaw)
            cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z = bx, by, c.vz
            cmd.twist.angular.z = c.yaw_rate
            state = 'following' if c.active else ('reached' if c.reached else 'idle')
        self.cmd_pub.publish(cmd)
        if state != self.last_state:
            if state == 'reached':
                self.get_logger().info('goal reached')
            self.last_state = state
        self.state_pub.publish(String(data=state))


def main(args: list[str] | None = None) -> None:
    """Run the controller node until shutdown.

    Args:
        args: Command-line arguments forwarded to rclpy.
    """
    rclpy.init(args=args)
    node = ControllerNode()
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
