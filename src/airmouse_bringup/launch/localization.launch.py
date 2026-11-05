"""Start RTAB-Map (Phase 2: mapping on ground-truth odometry; Phase 6: own odometry + EKF).

Example (with sim.launch.py running):
    ros2 launch airmouse_bringup localization.launch.py use_ground_truth:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context: LaunchContext) -> list:
    """Build the actions once the launch arguments are known."""
    arg = lambda name: context.launch_configurations[name]  # noqa: E731
    use_lidar = arg('use_lidar').lower() == 'true'
    if arg('use_ground_truth').lower() != 'true':
        return [LogInfo(msg='localization: own odometry + EKF arrive in Phase 6; '
                            'run with use_ground_truth:=true for now.')]
    config = Path(get_package_share_directory('airmouse_bringup')) / 'config' / 'rtabmap.yaml'
    remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_rect_raw'),
        ('scan', '/scan'),
        ('odom', '/ground_truth/odom'),
        ('map', '/map'),
    ]
    return [Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        arguments=['--delete_db_on_start'],
        parameters=[str(config), {
            'use_sim_time': arg('use_sim_time').lower() == 'true',
            'subscribe_scan': use_lidar,
            'Grid/Sensor': '2' if use_lidar else '1',   # 2 = LiDAR + depth, 1 = depth only
        }],
        remappings=remappings)]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    Returns:
        A LaunchDescription that starts RTAB-Map.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Use the optional RPLIDAR C1 2D scan.'),
        DeclareLaunchArgument(
            'use_ground_truth', default_value='false',
            description='Use the Gazebo ground-truth pose instead of RTAB-Map + EKF (Phases 2-5).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock (/clock).'),
        OpaqueFunction(function=launch_setup),
    ])
