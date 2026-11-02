"""Controller node: /nav/path + pose from TF -> pure pursuit -> /nav/cmd_vel_raw. Implemented in Phase 4."""

import rclpy
from rclpy.node import Node

NODE_NAME = 'controller_node'


def main(args: list[str] | None = None) -> None:
    """Start the node, report that it is a stub, and shut down.

    Args:
        args: Command-line arguments forwarded to rclpy.
    """
    rclpy.init(args=args)
    node = Node(NODE_NAME)
    node.get_logger().info(f'{NODE_NAME}: not implemented yet')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
