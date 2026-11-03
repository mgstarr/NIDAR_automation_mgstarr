"""Start RViz with the AirMouse display layout (rviz/airmouse.rviz)."""

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
        A LaunchDescription that starts RViz.
    """
    config = Path(get_package_share_directory('airmouse_bringup')) / 'rviz' / 'airmouse.rviz'
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulation clock (/clock).'),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', str(config)],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                         value_type=bool)}]),
    ])
