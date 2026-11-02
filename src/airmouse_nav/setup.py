"""Build script for the airmouse_nav ament_python package."""

from setuptools import find_packages, setup

package_name = 'airmouse_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirMouse Team',
    maintainer_email='lily207moon@gmail.com',
    description='Exploration, A* planning and pure-pursuit control for the NIDAR AirMouse drone.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer_node = airmouse_nav.nodes.explorer_node:main',
            'controller_node = airmouse_nav.nodes.controller_node:main',
            'safety_hook_node = airmouse_nav.nodes.safety_hook_node:main',
            'goal_planner_node = airmouse_nav.nodes.goal_planner_node:main',
        ],
    },
)
