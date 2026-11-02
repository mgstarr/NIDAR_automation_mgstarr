"""Goal planner node (test tool): RViz "2D Goal Pose" -> A* -> /nav/path. Implemented in Phase 3."""

import rclpy
from rclpy.node import Node

NODE_NAME = 'goal_planner_node'


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
