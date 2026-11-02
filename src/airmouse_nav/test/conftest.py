"""Shared pytest setup for airmouse_nav.

Makes the package importable straight from the source tree, so `pytest` works without
building or sourcing ROS. Synthetic-maze fixtures are added here from Phase 3 on.
"""

import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parent

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


@pytest.fixture
def config_dir() -> Path:
    """Return the airmouse_bringup config directory in the source tree."""
    return SRC_ROOT / 'airmouse_bringup' / 'config'
