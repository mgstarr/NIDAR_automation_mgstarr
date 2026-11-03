"""Start Gazebo Harmonic with a generated maze, the simulated drone and the ros_gz bridge.

Steps: generate the maze for `seed` / `wall_texture`, start Gazebo, spawn the drone (with or
without LiDAR), bridge Gazebo topics to the contract names, publish the drone's static TF
(read from the model file), and optionally the ground-truth odom -> base_link TF and RViz.

Example:
    ros2 launch airmouse_bringup sim.launch.py seed:=3 use_ground_truth:=true
"""

import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Rotation from a camera link (x forward) to its optical frame (z forward, x right, y down).
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)


def generate_world(seed: str, wall_texture: str) -> Path:
    """Run the maze generator and return the world file it wrote."""
    script = Path(get_package_prefix('airmouse_description')) / 'lib' / 'airmouse_description' / 'generate_maze.py'
    out = subprocess.run([sys.executable, str(script), '--seed', seed, '--wall-texture', wall_texture],
                         check=True, capture_output=True, text=True)
    return Path(out.stdout.strip().splitlines()[-1])


def drone_sdf(use_lidar: bool) -> tuple[str, ET.Element]:
    """Load the drone model; drop the LiDAR link and joint when use_lidar is false."""
    path = Path(get_package_share_directory('airmouse_description')) / 'models' / 'airmouse' / 'model.sdf'
    root = ET.parse(path).getroot()
    model = root.find('model')
    if not use_lidar:
        for tag, name in (('link', 'lidar_link'), ('joint', 'lidar_joint')):
            model.remove(model.find(f"{tag}[@name='{name}']"))
    return ET.tostring(root, encoding='unicode'), model


def static_tf_nodes(model: ET.Element) -> list[Node]:
    """base_link -> sensor link transforms from the model's link poses, plus the optical frame."""
    nodes = []
    frames = [(l.get('name'), [float(v) for v in l.findtext('pose', '0 0 0 0 0 0').split()])
              for l in model.findall('link')
              if l.get('name') != 'base_link' and not l.get('name').startswith('rotor_')]
    frames = [('base_link', name, pose) for name, pose in frames]
    frames.append(('camera_link', 'camera_optical_frame', [0.0, 0.0, 0.0, *OPTICAL_RPY]))
    for parent, child, (x, y, z, roll, pitch, yaw) in frames:
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name=f'tf_{child}',
            arguments=['--x', str(x), '--y', str(y), '--z', str(z), '--roll', str(roll),
                       '--pitch', str(pitch), '--yaw', str(yaw),
                       '--frame-id', parent, '--child-frame-id', child],
            parameters=[{'use_sim_time': True}]))
    return nodes


def launch_setup(context: LaunchContext) -> list:
    """Build the actions once the launch arguments are known."""
    arg = lambda name: context.launch_configurations[name]  # noqa: E731
    use_lidar = arg('use_lidar').lower() == 'true'
    use_gt = arg('use_ground_truth').lower() == 'true'
    sim_time = {'use_sim_time': arg('use_sim_time').lower() == 'true'}
    bringup = Path(get_package_share_directory('airmouse_bringup'))
    world = generate_world(arg('seed'), arg('wall_texture'))
    sdf_xml, model = drone_sdf(use_lidar)

    gz_flags = '-r' if arg('gui').lower() == 'true' else '-r -s'
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'{gz_flags} {world}', 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'maze', '-name', 'airmouse', '-string', sdf_xml,
                        '-x', '0', '-y', '0', '-z', '0.1']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': str(bringup / 'config' / 'bridge.yaml')}, sim_time]),
        Node(package='airmouse_nav', executable='sim_bridge_node', name='sim_bridge_node',
             parameters=[str(bringup / 'config' / 'sim_bridge.yaml'), sim_time]),
        Node(package='depth_image_proc', executable='point_cloud_xyz_node', name='depth_points',
             remappings=[('image_rect', '/camera/depth/image_rect_raw'),
                         ('camera_info', '/camera/depth/camera_info'),
                         ('points', '/camera/depth/points')],
             parameters=[sim_time]),
        *static_tf_nodes(model),
    ]
    if use_gt:
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_ground_truth_tf',
            arguments=['/airmouse/odometry_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            remappings=[('/airmouse/odometry_tf', '/tf')], parameters=[sim_time]))
    if arg('rviz').lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch' / 'rviz.launch.py')),
            launch_arguments={'use_sim_time': arg('use_sim_time')}.items()))
    return actions


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    Returns:
        A LaunchDescription for the AirMouse simulation.
    """
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='0', description='Maze random seed.'),
        DeclareLaunchArgument('wall_texture', default_value='plain',
                              description='Maze walls: plain or textured.'),
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Use the optional RPLIDAR C1 2D scan.'),
        DeclareLaunchArgument(
            'use_ground_truth', default_value='false',
            description='Use the Gazebo ground-truth pose instead of RTAB-Map + EKF (Phases 2-5).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock (/clock).'),
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo window.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        OpaqueFunction(function=launch_setup),
    ])
