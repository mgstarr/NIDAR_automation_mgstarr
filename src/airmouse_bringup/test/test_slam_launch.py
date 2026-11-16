"""Launch test (Phase 6): with ground truth off, every contract topic is alive and the TF chain
map -> odom -> base_link is complete.

Needs Gazebo, so it only runs when AIRMOUSE_SIM_TESTS=1:
    AIRMOUSE_SIM_TESTS=1 python3 -m pytest src/airmouse_bringup/test/test_slam_launch.py -s
It starts mission.launch.py headless with use_ground_truth:=false (once with LiDAR, once
without), waits for start-up, runs tools/check_contract.py and stops everything.
"""

import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
pytestmark = pytest.mark.skipif(os.environ.get('AIRMOUSE_SIM_TESTS') != '1',
                                reason='set AIRMOUSE_SIM_TESTS=1 (needs Gazebo)')


@pytest.mark.parametrize('use_lidar', ['true', 'false'])
def test_contract_topics_and_tf_without_ground_truth(use_lidar: str) -> None:
    """All contract topics publish and map -> odom -> base_link resolves."""
    subprocess.run(['bash', str(REPO / 'tools' / 'batch' / 'kill_sim.sh')], capture_output=True)
    launch = subprocess.Popen(
        ['ros2', 'launch', 'airmouse_bringup', 'mission.launch.py', 'seed:=3',
         'wall_texture:=textured', f'use_lidar:={use_lidar}', 'use_ground_truth:=false',
         'gui:=false', 'rviz:=false', 'detection:=false'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        time.sleep(30)
        check = subprocess.run(
            ['python3', str(REPO / 'tools' / 'check_contract.py'), '--use-lidar', use_lidar,
             '--timeout', '30'], capture_output=True, text=True)
        assert check.returncode == 0, check.stdout
    finally:
        os.killpg(launch.pid, signal.SIGINT)
        launch.wait(timeout=30)
        subprocess.run(['bash', str(REPO / 'tools' / 'batch' / 'kill_sim.sh')],
                       capture_output=True)
