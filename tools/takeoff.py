#!/usr/bin/env python3
"""Test tool (sim): climb to a target altitude on /cmd_vel using ground truth, then hover.

Usage (inside the container, with the sim running):
    python3 tools/takeoff.py              # to 1.2 m (flight_altitude in docs/PLAN.md)
    python3 tools/takeoff.py --altitude 0.3

Phase 4 replaces this with take-off paths through the controller.
"""

import argparse

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Takeoff(Node):
    """P-controller on height until within tolerance for a hold time."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('takeoff_tool')
        self.args = args
        self.z = None
        self.settled_since = None
        self.done = False
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.create_timer(1.0 / args.rate, self.step)

    def on_odom(self, msg: Odometry) -> None:
        """Store the current height."""
        self.z = msg.pose.pose.position.z

    def send(self, vz: float) -> None:
        """Publish a body-frame vertical velocity."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.z = vz
        self.pub.publish(msg)

    def step(self) -> None:
        """One control step."""
        if self.z is None:
            return
        err = self.args.altitude - self.z
        self.send(max(-self.args.max_vz, min(self.args.max_vz, self.args.gain * err)))
        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(err) > self.args.tolerance:
            self.settled_since = None
        elif self.settled_since is None:
            self.settled_since = now
        elif now - self.settled_since > self.args.hold:
            self.send(0.0)
            self.get_logger().info(f'hovering at {self.z:.2f} m')
            self.done = True


def main() -> None:
    """Parse arguments and run until the altitude is reached."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--altitude', type=float, default=1.2)
    parser.add_argument('--gain', type=float, default=1.0)
    parser.add_argument('--max-vz', type=float, default=0.5)
    parser.add_argument('--tolerance', type=float, default=0.03)
    parser.add_argument('--hold', type=float, default=1.0)
    parser.add_argument('--rate', type=float, default=20.0)
    args = parser.parse_args()
    rclpy.init()
    node = Takeoff(args)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
