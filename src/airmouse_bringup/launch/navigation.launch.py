"""Start the navigation nodes (Phases 3-5).

Phase 4: goal_planner_node (RViz "2D Goal Pose" -> A* -> /nav/path, replanned at 1 Hz),
controller_node (/nav/path -> /nav/cmd_vel_raw) and safety_hook_node (-> /cmd_vel).
Phase 5 adds the explorer.

In simulation pass the same `seed` and `wall_texture` as sim.launch.py: the geofence
(`arena_bounds`) is then read from the generated maze's layout.json. Without a seed, the
value in explorer.yaml is used.
"""

import json
import subprocess
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def arena_bounds(seed: str, wall_texture: str) -> list[float]:
    """[xmin, xmax, ymin, ymax] of the generated maze (runs the deterministic generator)."""
    script = (Path(get_package_prefix('airmouse_description')) / 'lib' / 'airmouse_description'
              / 'generate_maze.py')
    out = subprocess.run([sys.executable, str(script), '--seed', seed, '--wall-texture',
                          wall_texture], check=True, capture_output=True, text=True)
    world = Path(out.stdout.strip().splitlines()[-1])
    b = json.loads((world.parent / 'layout.json').read_text())['arena_bounds']
    return [b['xmin'], b['xmax'], b['ymin'], b['ymax']]


def launch_setup(context, *args, **kwargs) -> list:
    """Create the nodes once the launch arguments are known."""
    config = Path(get_package_share_directory('airmouse_bringup')) / 'config'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    seed = LaunchConfiguration('seed').perform(context)
    common = {'use_sim_time': use_sim_time}
    files = [str(config / 'controller.yaml'), str(config / 'planner.yaml')]
    safety = files + [str(config / 'explorer.yaml'), common]
    if seed:
        safety.append({'arena_bounds': arena_bounds(
            seed, LaunchConfiguration('wall_texture').perform(context))})
    return [
        Node(package='airmouse_nav', executable='goal_planner_node', name='goal_planner_node',
             output='screen', parameters=files + [common]),
        Node(package='airmouse_nav', executable='controller_node', name='controller_node',
             output='screen', parameters=files + [common]),
        Node(package='airmouse_nav', executable='safety_hook_node', name='safety_hook_node',
             output='screen', parameters=safety),
    ]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    Returns:
        A LaunchDescription with the shared arguments and the navigation nodes.
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
        DeclareLaunchArgument(
            'seed', default_value='',
            description='Sim only: maze seed, to read the geofence from its layout.json.'),
        DeclareLaunchArgument(
            'wall_texture', default_value='plain',
            description='Sim only: wall texture of the maze (plain or textured).'),
        OpaqueFunction(function=launch_setup),
    ])
