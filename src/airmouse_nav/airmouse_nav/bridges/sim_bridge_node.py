"""Simulation bridge node (sim only; the real drone uses fc_bridge instead).

- /cmd_vel (TwistStamped, body frame) -> /airmouse/cmd_twist (Twist, plugin frame);
- keeps the Gazebo velocity controller enabled (/airmouse/enable);
- /range/down/scan and /range/up/scan (1-beam LaserScan) -> /range/down and /range/up (Range).
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool

from airmouse_nav.bridges.sim_bridge import (VelocityCommand, beam_range, body_to_plugin,
                                             yaw_from_quaternion)

NODE_NAME = 'sim_bridge_node'
RANGERS = ('down', 'up')


class SimBridgeNode(Node):
    """Translates contract topics to and from the Gazebo-specific ones."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.plugin_frame = self.declare_parameter('plugin_frame', 'body').value
        self.range_fov = self.declare_parameter('range_field_of_view', 0.47).value
        enable_period = self.declare_parameter('enable_period', 1.0).value
        self.yaw = 0.0
        self.twist_pub = self.create_publisher(Twist, '/airmouse/cmd_twist', 10)
        self.enable_pub = self.create_publisher(Bool, '/airmouse/enable', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.range_pubs = {}
        for name in RANGERS:
            self.range_pubs[name] = self.create_publisher(Range, f'/range/{name}', 10)
            self.create_subscription(LaserScan, f'/range/{name}/scan',
                                     lambda msg, n=name: self.on_scan(msg, n), 10)
        self.create_timer(enable_period, self.publish_enable)

    def publish_enable(self) -> None:
        """Re-send 'enable' periodically, so a late-starting Gazebo still gets it."""
        self.enable_pub.publish(Bool(data=True))

    def on_odom(self, msg: Odometry) -> None:
        """Track the heading, needed only when the plugin expects world-frame velocity."""
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def on_cmd_vel(self, msg: TwistStamped) -> None:
        """Forward a body-frame command to the plugin."""
        t = msg.twist
        cmd = body_to_plugin(VelocityCommand(t.linear.x, t.linear.y, t.linear.z, t.angular.z),
                             self.yaw, self.plugin_frame)
        out = Twist()
        out.linear.x, out.linear.y, out.linear.z, out.angular.z = cmd
        self.twist_pub.publish(out)

    def on_scan(self, msg: LaserScan, name: str) -> None:
        """Convert a single-beam scan to a Range message."""
        out = Range()
        out.header = msg.header
        out.radiation_type = Range.INFRARED
        out.field_of_view = float(self.range_fov)
        out.min_range, out.max_range = msg.range_min, msg.range_max
        out.range = float(beam_range(list(msg.ranges), msg.range_min, msg.range_max))
        self.range_pubs[name].publish(out)


def main(args: list[str] | None = None) -> None:
    """Run the simulation bridge node.

    Args:
        args: Command-line arguments forwarded to rclpy.
    """
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
