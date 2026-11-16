"""Range-to-z adapter node (Phase 6): /range/down -> a z measurement for the EKF.

Subscribes: /range/down (sensor_msgs/Range).
Publishes:  /range/down/pose (geometry_msgs/PoseWithCovarianceStamped, frame odom): only z is
            meaningful (the EKF config fuses z only); variance = z_stddev^2.
Parameters: sensor_offset (m below base_link, 0.06), z_stddev (0.02 m), odom_frame ('odom').
Tilt is not corrected here (at <= 10 degrees the error is under 2%).
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from airmouse_nav.bridges.range_to_z import range_to_z

NODE_NAME = 'range_to_z_node'


class RangeToZNode(Node):
    """Turns each valid rangefinder reading into a z pose measurement."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.offset = self.declare_parameter('sensor_offset', 0.06).value
        self.var = self.declare_parameter('z_stddev', 0.02).value ** 2
        self.frame = self.declare_parameter('odom_frame', 'odom').value
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/range/down/pose', 10)
        self.create_subscription(Range, '/range/down', self.on_range, qos_profile_sensor_data)

    def on_range(self, msg: Range) -> None:
        """Publish z for a valid reading."""
        z = range_to_z(msg.range, msg.min_range, msg.max_range, self.offset)
        if z is None:
            return
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame
        out.pose.pose.position.z = z
        out.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[21] = cov[28] = cov[35] = 1e6    # x, y, roll, pitch, yaw: unused
        cov[14] = self.var
        out.pose.covariance = cov
        self.pub.publish(out)


def main(args: list[str] | None = None) -> None:
    """Run the adapter until shutdown.

    Args:
        args: Command-line arguments forwarded to rclpy.
    """
    rclpy.init(args=args)
    node = RangeToZNode()
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
