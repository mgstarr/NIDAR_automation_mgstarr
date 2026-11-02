"""Start RViz with the AirMouse display layout (rviz/airmouse.rviz).

Phase 0 stub: declares the shared launch arguments and starts nothing yet.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    Returns:
        A LaunchDescription declaring the shared AirMouse launch arguments.
    """
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
    ])
