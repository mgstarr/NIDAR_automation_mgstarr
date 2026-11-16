"""Start the survivor detector (airmouse_detection) on the colour camera."""

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
        A LaunchDescription that starts survivor_detector_node.
    """
    config = Path(get_package_share_directory('airmouse_bringup')) / 'config' / 'detection.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock (/clock).'),
        Node(package='airmouse_detection', executable='survivor_detector_node',
             name='survivor_detector_node', output='screen',
             parameters=[str(config), {'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
