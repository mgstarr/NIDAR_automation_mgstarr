"""Architecture checks: core/ stays ROS-free and the config files are well formed."""

import ast
import importlib
import sys
from pathlib import Path

import pytest
import yaml

CORE_DIR = Path(__file__).resolve().parents[1] / 'airmouse_nav' / 'core'
CORE_MODULES = sorted(p.stem for p in CORE_DIR.glob('*.py') if p.stem != '__init__')

# Top-level packages that core/ must never import.
FORBIDDEN_IMPORTS = {
    'rclpy', 'rcl_interfaces', 'builtin_interfaces', 'tf2_ros', 'tf2_py',
    'launch', 'launch_ros', 'ament_index_python', 'message_filters', 'cv_bridge',
}
# The parts of airmouse_nav that are allowed to depend on ROS.
FORBIDDEN_INTERNAL = ('airmouse_nav.nodes', 'airmouse_nav.bridges')

# ROS parameter files (node sections with ros__parameters). bridge.yaml is a ros_gz_bridge
# topic list instead, so it is checked separately.
PARAM_CONFIGS = {
    'rtabmap.yaml', 'ekf.yaml', 'explorer.yaml', 'planner.yaml', 'controller.yaml',
}
EXPECTED_CONFIGS = PARAM_CONFIGS | {'bridge.yaml'}


def imported_modules(source_path: Path) -> set[str]:
    """Return the full dotted names of every module imported in a source file.

    Args:
        source_path: Python file to scan.

    Returns:
        Set of dotted module names, e.g. {'numpy', 'airmouse_nav.core.grid_utils'}.
    """
    tree = ast.parse(source_path.read_text(), filename=str(source_path))
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            names.add(node.module)
    return names


def is_ros_import(module_name: str) -> bool:
    """Return True if a dotted module name belongs to ROS or to a ROS-facing layer.

    Args:
        module_name: Dotted module name, e.g. 'geometry_msgs.msg'.
    """
    top_level = module_name.split('.')[0]
    return (
        top_level in FORBIDDEN_IMPORTS
        or top_level.endswith('_msgs')
        or top_level.endswith('_srvs')
        or module_name.startswith(FORBIDDEN_INTERNAL)
    )


def test_core_modules_exist() -> None:
    """All planned core modules are present."""
    assert set(CORE_MODULES) == {
        'grid_utils', 'astar', 'frontier', 'coverage', 'viewpoint', 'mission', 'pure_pursuit',
    }


@pytest.mark.parametrize('module', CORE_MODULES)
def test_core_has_no_ros_imports(module: str) -> None:
    """No core/ file imports ROS, message packages, nodes/ or bridges/."""
    offenders = sorted(m for m in imported_modules(CORE_DIR / f'{module}.py') if is_ros_import(m))
    assert not offenders, f'core/{module}.py imports ROS-facing modules: {offenders}'


@pytest.mark.parametrize('module', CORE_MODULES)
def test_core_imports_with_ros_blocked(module: str, monkeypatch: pytest.MonkeyPatch) -> None:
    """Each core module imports even when ROS is present but blocked (catches indirect imports)."""
    for name in FORBIDDEN_IMPORTS:
        monkeypatch.setitem(sys.modules, name, None)
    for loaded in [m for m in sys.modules if m.startswith('airmouse_nav')]:
        monkeypatch.delitem(sys.modules, loaded)
    importlib.import_module(f'airmouse_nav.core.{module}')


def test_all_config_files_present(config_dir: Path) -> None:
    """Every config file named in the design exists in airmouse_bringup/config."""
    present = {p.name for p in config_dir.glob('*.yaml')}
    assert EXPECTED_CONFIGS <= present, f'missing: {sorted(EXPECTED_CONFIGS - present)}'


@pytest.mark.parametrize('filename', sorted(PARAM_CONFIGS))
def test_config_has_ros_parameters(filename: str, config_dir: Path) -> None:
    """Each config parses as YAML and every node section has a ros__parameters mapping."""
    data = yaml.safe_load((config_dir / filename).read_text())
    assert isinstance(data, dict) and data, f'{filename} has no node sections'
    for node_name, section in data.items():
        assert isinstance(section, dict) and 'ros__parameters' in section, (
            f'{filename}: node "{node_name}" has no ros__parameters')
        assert isinstance(section['ros__parameters'], dict), (
            f'{filename}: ros__parameters of "{node_name}" is not a mapping')


def test_bridge_config_is_topic_list(config_dir: Path) -> None:
    """bridge.yaml parses as a list of ros_gz_bridge topic mappings (empty until Phase 1)."""
    data = yaml.safe_load((config_dir / 'bridge.yaml').read_text())
    assert isinstance(data, list), 'bridge.yaml must be a YAML list'
