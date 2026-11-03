"""Maze generator tests over 50 seeds: rules, outputs and determinism."""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy import ndimage

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'generate_maze.py'
spec = importlib.util.spec_from_file_location('generate_maze', SCRIPT)
gm = importlib.util.module_from_spec(spec)
sys.modules['generate_maze'] = gm
spec.loader.exec_module(gm)

SEEDS = range(50)


@pytest.fixture(scope='module')
def mazes() -> dict:
    """One generated maze per seed (generation is the slow part, so share it)."""
    return {seed: gm.generate(gm.MazeConfig(seed=seed)) for seed in SEEDS}


@pytest.mark.parametrize('seed', SEEDS)
def test_rules_hold(seed: int, mazes: dict) -> None:
    """validate() passes: bounds, corridor width, room count, reachability, survivors."""
    gm.validate(mazes[seed])


@pytest.mark.parametrize('seed', SEEDS)
def test_room_count_and_survivors(seed: int, mazes: dict) -> None:
    """6-8 rooms; up to 6 survivors, each in a distinct room and inside it."""
    maze = mazes[seed]
    assert 6 <= len(maze.rooms) <= 8
    assert len(maze.survivors) == 6
    assert len({s['room'] for s in maze.survivors}) == len(maze.survivors)


@pytest.mark.parametrize('seed', SEEDS)
def test_arena_bounds_and_entry(seed: int, mazes: dict) -> None:
    """Arena fits in 15 x 15 m; the entry gap is in the west wall at the origin's y."""
    maze = mazes[seed]
    xmin, xmax, ymin, ymax = maze.bounds()
    assert xmax - xmin <= 15.0 and ymax - ymin <= 15.0
    assert xmin < 0 < xmax and ymin < 0 < ymax
    assert not maze.wx[0, maze.entry[1]]
    assert maze.wx[0].sum() == maze.config.cells - 1       # exactly one opening on the west wall
    assert maze.wx[-1].all() and maze.wy[:, 0].all() and maze.wy[:, -1].all()


@pytest.mark.parametrize('seed', SEEDS)
def test_rooms_have_one_door(seed: int, mazes: dict) -> None:
    """Each room is a clear 2 x 2 block with exactly one opening to a corridor."""
    maze = mazes[seed]
    n = maze.config.cells
    for room in maze.rooms:
        cells = gm.room_cells(room)
        doors = [(a, b) for a in cells for b in gm.neighbours(a, n)
                 if b not in cells and not gm.wall_present(maze, a, b)]
        assert len(doors) == 1


def test_validate_rejects_narrow_passage() -> None:
    """The width check catches a passage narrower than 1 m."""
    maze = gm.generate(gm.MazeConfig(seed=0))
    narrow = gm.MazeConfig(seed=0, wall_thickness=0.3)
    maze.config = narrow
    with pytest.raises(AssertionError):
        gm.validate(maze)


def test_same_seed_same_maze() -> None:
    """Generation is deterministic."""
    a, b = gm.generate(gm.MazeConfig(seed=7)), gm.generate(gm.MazeConfig(seed=7))
    assert np.array_equal(a.wx, b.wx) and np.array_equal(a.wy, b.wy)
    assert a.rooms == b.rooms and a.survivors == b.survivors


@pytest.mark.parametrize('texture', ['plain', 'textured'])
def test_output_files(texture: str, tmp_path: Path) -> None:
    """main() writes the world, ground-truth image + YAML and the JSON layout."""
    assert gm.main(['--seed', '3', '--wall-texture', texture, '--output-root', str(tmp_path)]) == 0
    out = tmp_path / f'seed_3_{texture}'
    assert (out / 'world.sdf').read_text().count('<model name="survivor_') == 6
    assert ('albedo_map' in (out / 'world.sdf').read_text()) == (texture == 'textured')
    layout = json.loads((out / 'layout.json').read_text())
    assert len(layout['survivors']) == 6 and 6 <= len(layout['rooms']) <= 8
    header, image = (out / 'layout.pgm').read_bytes().split(b'255\n', 1)
    cols, rows = map(int, header.split()[1:3])
    pixels = np.frombuffer(image, dtype=np.uint8).reshape(rows, cols)
    assert set(np.unique(pixels)) == {0, 254}
    assert 'resolution: 0.05' in (out / 'layout.yaml').read_text()
    # The image (flipped back to row 0 = lowest y) matches the rasterized layout.
    occupied, _ = gm.rasterize(gm.generate(gm.MazeConfig(seed=3)))
    assert np.array_equal(pixels[::-1] == 0, occupied)
    assert ndimage.label(pixels > 0)[1] == 1
