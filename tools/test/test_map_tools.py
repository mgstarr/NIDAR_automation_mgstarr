"""Tests for tools/map_io.py, save_map.py and score_map.py."""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))
import map_io  # noqa: E402
import save_map  # noqa: E402
import score_map  # noqa: E402

spec = importlib.util.spec_from_file_location(
    'generate_maze', REPO / 'src' / 'airmouse_description' / 'scripts' / 'generate_maze.py')
gm = importlib.util.module_from_spec(spec)
sys.modules['generate_maze'] = gm
spec.loader.exec_module(gm)


def truth_map(seed: int = 0) -> map_io.OccupancyMap:
    """Ground-truth layout of a generated maze as an OccupancyMap."""
    maze = gm.generate(gm.MazeConfig(seed=seed))
    occupied, origin = gm.rasterize(maze)
    grid = np.where(occupied, map_io.OCCUPIED, map_io.FREE).astype(np.int8)
    return map_io.OccupancyMap(grid, maze.config.resolution, origin)


def test_round_trip(tmp_path: Path) -> None:
    """save -> load gives back the same grid, resolution, origin and metadata."""
    rng = np.random.default_rng(0)
    grid = rng.choice([map_io.OCCUPIED, map_io.FREE, map_io.UNKNOWN], size=(37, 52)).astype(np.int8)
    occ = map_io.OccupancyMap(grid, 0.05, (-1.25, 3.4), {'seed': 4, 'use_lidar': 'true'})
    back = map_io.load(map_io.save(tmp_path / 'm', occ))
    assert np.array_equal(back.grid, grid)
    assert back.resolution == 0.05 and back.origin == pytest.approx((-1.25, 3.4))
    assert back.meta == occ.meta


def test_msg_data_conversion() -> None:
    """OccupancyGrid values -> trinary grid, rows from the lowest y."""
    grid = save_map.grid_from_msg_data([0, 100, -1, 50, 10, 90], width=3, height=2)
    assert grid.tolist() == [[0, 100, -1], [-1, 0, 100]]


def test_generator_layout_loads(tmp_path: Path) -> None:
    """The maze generator's layout.yaml/pgm loads as the same grid it rasterized."""
    gm.main(['--seed', '2', '--output-root', str(tmp_path)])
    loaded = map_io.load(tmp_path / 'seed_2_plain' / 'layout.yaml')
    truth = truth_map(2)
    assert np.array_equal(loaded.grid, truth.grid)
    assert loaded.origin == pytest.approx(truth.origin)


def test_layout_against_itself_is_perfect() -> None:
    """The ground truth scored against itself: full agreement, no false walls."""
    truth = truth_map()
    result = score_map.score(truth, truth)
    assert result['wall_agreement'] == 1.0
    assert result['false_wall_cells'] == 0
    assert result['known_free_fraction'] == 1.0


def test_shifted_origin_still_aligns() -> None:
    """A cropped map with its own origin aligns by origin and still scores perfectly."""
    truth = truth_map()
    crop = map_io.OccupancyMap(truth.grid[10:, 7:].copy(), truth.resolution,
                               (truth.origin[0] + 7 * 0.05, truth.origin[1] + 10 * 0.05))
    result = score_map.score(crop, truth)
    assert result['wall_agreement'] == 1.0 and result['false_wall_cells'] == 0


def test_false_wall_detected() -> None:
    """A wall block added in the middle of the entry corridor is reported."""
    truth = truth_map()
    noisy = map_io.OccupancyMap(truth.grid.copy(), truth.resolution, truth.origin)
    r, c = int((0.0 - truth.origin[1]) / 0.05), int((0.0 - truth.origin[0]) / 0.05)
    noisy.grid[r - 2:r + 2, c - 2:c + 2] = map_io.OCCUPIED     # 0.2 m block at the take-off point
    result = score_map.score(noisy, truth)
    assert result['false_wall_cells'] > 0


def test_missing_walls_lower_agreement() -> None:
    """Seen free space with its walls erased lowers wall agreement."""
    truth = truth_map()
    partial = map_io.OccupancyMap(truth.grid.copy(), truth.resolution, truth.origin)
    partial.grid[partial.grid == map_io.OCCUPIED] = map_io.FREE
    assert score_map.score(partial, truth)['wall_agreement'] == 0.0


def test_survivors_are_not_false_walls() -> None:
    """A mapped survivor is a real obstacle; the same block elsewhere is a false wall."""
    truth = truth_map()
    noisy = map_io.OccupancyMap(truth.grid.copy(), truth.resolution, truth.origin)
    r, c = int((0.0 - truth.origin[1]) / 0.05), int((0.0 - truth.origin[0]) / 0.05)
    noisy.grid[r - 2:r + 2, c - 2:c + 2] = map_io.OCCUPIED
    assert score_map.score(noisy, truth, survivors=[(0.0, 0.0)])['false_wall_cells'] == 0
    assert score_map.score(noisy, truth, survivors=[(3.0, 3.0)])['false_wall_cells'] > 0
