"""Start the full mission with one command: sim, mapping, controller, safety hook, explorer, RViz.

    ros2 launch airmouse_bringup mission.launch.py use_ground_truth:=true [seed:=3]
        [use_lidar:=true|false] [wall_texture:=plain|textured] [gui:=true] [rviz:=true]
        [real_time_factor:=1.0]
    ros2 service call /mission/start std_srvs/srv/Trigger        # then watch /mission/state

The geofence (`arena_bounds`) for the explorer and the safety hook is read from the generated
maze. Until Phase 6, use_ground_truth must be true.
"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    'seed': ('0', 'Maze seed.'),
    'wall_texture': ('plain', 'Wall texture: plain or textured.'),
    'use_lidar': ('true', 'Use the optional RPLIDAR C1 2D scan.'),
    'use_ground_truth': ('false', 'Use the Gazebo ground-truth pose (required until Phase 6).'),
    'use_sim_time': ('true', 'Use the simulation clock (/clock).'),
    'gui': ('true', 'Show the Gazebo window (false = headless).'),
    'rviz': ('true', 'Start RViz.'),
    'real_time_factor': ('1.0', 'Simulation speed (1.0 = real time).'),
}


def launch_setup(context, *args, **kwargs) -> list:
    """Include sim, localization and navigation; add the explorer."""
    share = Path(get_package_share_directory('airmouse_bringup'))
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}

    def include(name: str, extra: dict) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / name)),
            launch_arguments=extra.items())

    spec = importlib.util.spec_from_file_location('nav_launch',
                                                  share / 'launch' / 'navigation.launch.py')
    nav = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(nav)
    bounds = nav.arena_bounds(a['seed'], a['wall_texture'])
    config = share / 'config'
    explorer = Node(
        package='airmouse_nav', executable='explorer_node', name='explorer_node',
        output='screen',
        parameters=[str(config / 'explorer.yaml'), str(config / 'planner.yaml'),
                    str(config / 'controller.yaml'),
                    {'use_sim_time': a['use_sim_time'] == 'true',
                     'use_lidar': a['use_lidar'] == 'true', 'arena_bounds': bounds}])
    return [
        include('sim.launch.py', {k: a[k] for k in ('seed', 'wall_texture', 'use_lidar',
                                                    'use_ground_truth', 'use_sim_time', 'gui',
                                                    'rviz', 'real_time_factor')}),
        include('localization.launch.py', {k: a[k] for k in ('use_lidar', 'use_ground_truth',
                                                             'use_sim_time')}),
        include('navigation.launch.py', {'use_sim_time': a['use_sim_time'], 'seed': a['seed'],
                                         'wall_texture': a['wall_texture'],
                                         'goal_planner': 'false'}),
        explorer,
    ]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    Returns:
        A LaunchDescription with the mission arguments and nodes.
    """
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()]
        + [OpaqueFunction(function=launch_setup)])
