"""Start RTAB-Map (Phase 2: mapping on ground-truth odometry; Phase 6: own odometry + EKF).

Example (with sim.launch.py running):
    ros2 launch airmouse_bringup localization.launch.py use_ground_truth:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context: LaunchContext) -> list:
    """Build the actions once the launch arguments are known."""
    arg = lambda name: context.launch_configurations[name]  # noqa: E731
    use_lidar = arg('use_lidar').lower() == 'true'
    ground_truth = arg('use_ground_truth').lower() == 'true'
    sim_time = {'use_sim_time': arg('use_sim_time').lower() == 'true'}
    config_dir = Path(get_package_share_directory('airmouse_bringup')) / 'config'
    config = str(config_dir / 'rtabmap.yaml')
    camera = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_rect_raw'),
    ]
    slam_params = {
        **sim_time,
        'subscribe_scan': use_lidar,
        'Grid/Sensor': '2' if use_lidar else '1',   # 2 = LiDAR + depth, 1 = depth only
    }
    nodes = []
    if ground_truth:
        odom_topic = '/ground_truth/odom'           # Phases 2-5: exact pose, no loop closure
    else:
        # Phase 6: odometry from sensors -> EKF -> /odom; RTAB-Map corrects drift (map -> odom).
        odom_topic = '/odom'
        if use_lidar:
            nodes.append(Node(
                package='rtabmap_odom', executable='icp_odometry', name='icp_odometry',
                output='screen', parameters=[config, sim_time],
                remappings=[('scan', '/scan'), ('odom', '/rtabmap/odom')]))
        else:
            nodes.append(Node(
                package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
                output='screen', parameters=[config, sim_time],
                remappings=camera + [('odom', '/rtabmap/odom')]))
        nodes.append(Node(
            package='airmouse_nav', executable='range_to_z_node', name='range_to_z_node',
            output='screen', parameters=[sim_time]))
        nodes.append(Node(
            package='robot_localization', executable='ekf_node', name='ekf_node',
            output='screen', parameters=[str(config_dir / 'ekf.yaml'), sim_time],
            remappings=[('odometry/filtered', '/odom')]))
        slam_params.update({
            'Kp/MaxFeatures': '400',                # visual words for loop closure (Pi budget)
            'Vis/MaxFeatures': '600',
            'Reg/Strategy': '1' if use_lidar else '0',          # refine links with ICP / visual
            'RGBD/NeighborLinkRefining': 'true' if use_lidar else 'false',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/AngularUpdate': '0.05',
            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.15',
            'Icp/CorrespondenceRatio': '0.2',
        })
    nodes.append(Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        arguments=['--delete_db_on_start'],
        parameters=[config, slam_params],
        remappings=camera + [('scan', '/scan'), ('odom', odom_topic), ('map', '/map')]))
    return nodes


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
