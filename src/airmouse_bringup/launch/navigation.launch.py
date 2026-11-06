"""Start the navigation nodes (Phases 3-5).

Phase 3: goal_planner_node (RViz "2D Goal Pose" -> A* -> /nav/path).
Phase 4 adds the controller and safety hook; Phase 5 the explorer.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    Returns:
        A LaunchDescription with the shared arguments and the navigation nodes.
    """
    config = Path(get_package_share_directory('airmouse_bringup')) / 'config'
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Use the optional RPLIDAR C1 2D scan.'),
        DeclareLaunchArgument(
            'use_ground_truth', default_value='false',
            description='Use the Gazebo ground-truth pose instead of RTAB-Map + EKF (Phases 2-5).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulation clock (/clock).'),
        Node(package='airmouse_nav', executable='goal_planner_node', name='goal_planner_node',
             output='screen',
             parameters=[str(config / 'planner.yaml'), {'use_sim_time': use_sim_time}]),
    ])
