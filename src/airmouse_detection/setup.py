"""Setup for airmouse_detection. Installs the detection team's handoff weights into share/."""

from setuptools import find_packages, setup

package_name = 'airmouse_detection'
WEIGHTS = '../../external/NIDAR-Automation/models/survivor_yolo11n_demo.pt'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', [WEIGHTS]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Survivor detection ROS 2 wrapper.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'survivor_detector_node = airmouse_detection.survivor_detector_node:main',
        ],
    },
)
