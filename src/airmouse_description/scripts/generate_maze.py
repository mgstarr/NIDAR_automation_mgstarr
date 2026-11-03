#!/usr/bin/env python3
"""Generate random, rule-valid AirMouse mazes as Gazebo (SDF) worlds from a random seed.

Layout: a square grid of cells with `cell_clear` m of clear space each, separated by walls
`wall_thickness` m thick (so corridors are exactly `cell_clear` wide). Corridors are carved
with a randomized depth-first search, then a few extra walls are removed to make loops.
Rooms are 2 x 2 cell blocks with one doorway. The entry/exit is on the west outer wall.

World frame = map frame: the origin is the centre of the entry cell (the take-off point)
and +x points into the maze along the entry direction.

Outputs (in one folder per seed and wall texture):
  world.sdf      Gazebo world (walls, net ceiling, survivors);
  layout.pgm     ground-truth occupancy image (black = wall, white = free, grey = outside);
  layout.yaml    map_server-style metadata for layout.pgm;
  layout.json    entry, arena bounds, rooms and survivors (for scoring);
  wall_*.png     wall textures (only when wall_texture is "textured").

Every maze is checked before it is written: arena within bounds, every free space at least
`cell_clear` wide, every room and corridor reachable from the entry.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import zlib
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from scipy import ndimage

Cell = tuple[int, int]

DEFAULT_OUTPUT_ROOT = Path(__file__).resolve().parents[1] / 'worlds' / 'generated'


@dataclass(frozen=True)
class MazeConfig:
    """All generator settings. Defaults follow docs/PLAN.md and the Phase 1 decisions."""

    seed: int
    wall_texture: str = 'plain'        # 'plain' or 'textured'
    num_survivors: int = 6             # at most one per room
    cells: int = 13                    # grid is cells x cells
    cell_clear: float = 1.0            # clear corridor width [m]
    wall_thickness: float = 0.10       # [m]
    wall_height: float = 2.44          # clear height, net ceiling on top [m]
    max_arena: float = 15.0            # arena side limit [m]
    rooms_min: int = 6
    rooms_max: int = 8
    loop_fraction: float = 0.10        # share of remaining inner walls removed for loops
    survivor_margin: float = 0.40      # survivor distance from room walls [m]
    resolution: float = 0.05           # ground-truth image cell size [m]

    @property
    def pitch(self) -> float:
        """Distance between neighbouring wall centre lines [m]."""
        return self.cell_clear + self.wall_thickness


@dataclass
class Maze:
    """A generated maze in grid form.

    wx[i, j] is the wall on vertical line x = i between cells (i-1, j) and (i, j);
    wy[i, j] is the wall on horizontal line y = j between cells (i, j-1) and (i, j).
    """

    config: MazeConfig
    wx: np.ndarray
    wy: np.ndarray
    entry: Cell
    rooms: list[Cell] = field(default_factory=list)        # lower-left cell of each 2x2 block
    survivors: list[dict] = field(default_factory=list)    # {'room': k, 'x': .., 'y': ..}

    @property
    def offset(self) -> tuple[float, float]:
        """Grid coordinates of the world origin (centre of the entry cell)."""
        p = self.config.pitch
        return (self.entry[0] + 0.5) * p, (self.entry[1] + 0.5) * p

    def to_world(self, gx: float, gy: float) -> tuple[float, float]:
        """Convert grid coordinates [m] to world coordinates [m]."""
        ox, oy = self.offset
        return gx - ox, gy - oy

    def bounds(self) -> tuple[float, float, float, float]:
        """Outer wall extent in world coordinates: (xmin, xmax, ymin, ymax)."""
        c, t = self.config, self.config.wall_thickness
        size = c.cells * c.pitch
        x0, y0 = self.to_world(-t / 2, -t / 2)
        return x0, x0 + size + t, y0, y0 + size + t

    def room_box(self, k: int) -> tuple[float, float, float, float]:
        """Clear interior of room k in world coordinates: (xmin, xmax, ymin, ymax)."""
        p, t = self.config.pitch, self.config.wall_thickness
        i, j = self.rooms[k]
        x0, y0 = self.to_world(i * p + t / 2, j * p + t / 2)
        return x0, x0 + 2 * p - t, y0, y0 + 2 * p - t


# ---------------------------------------------------------------- layout generation

def neighbours(cell: Cell, n: int) -> list[Cell]:
    """4-connected grid neighbours of a cell inside an n x n grid."""
    i, j = cell
    return [(a, b) for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1))
            if 0 <= a < n and 0 <= b < n]


def open_wall(maze: Maze, a: Cell, b: Cell) -> None:
    """Remove the wall between two adjacent cells."""
    if a[0] != b[0]:
        maze.wx[max(a[0], b[0]), a[1]] = False
    else:
        maze.wy[a[0], max(a[1], b[1])] = False


def wall_present(maze: Maze, a: Cell, b: Cell) -> bool:
    """True if the wall between two adjacent cells is standing."""
    if a[0] != b[0]:
        return bool(maze.wx[max(a[0], b[0]), a[1]])
    return bool(maze.wy[a[0], max(a[1], b[1])])


def room_cells(room: Cell) -> list[Cell]:
    """The four cells of a 2 x 2 room whose lower-left cell is `room`."""
    i, j = room
    return [(i, j), (i + 1, j), (i, j + 1), (i + 1, j + 1)]


def place_rooms(maze: Maze, rng: np.random.Generator) -> np.ndarray:
    """Pick non-touching 2 x 2 room blocks away from the entry.

    Rooms never touch each other or the entry cell (not even diagonally), so the
    remaining corridor cells stay connected.

    Returns:
        Boolean grid, True for cells that belong to a room.
    """
    c = maze.config
    n = c.cells
    target = int(rng.integers(c.rooms_min, c.rooms_max + 1))
    blocked = np.zeros((n, n), dtype=bool)
    ei, ej = maze.entry
    blocked[max(ei - 1, 0):ei + 2, max(ej - 1, 0):ej + 2] = True
    in_room = np.zeros((n, n), dtype=bool)
    candidates = [(i, j) for i in range(n - 1) for j in range(n - 1)]
    for idx in rng.permutation(len(candidates)):
        if len(maze.rooms) == target:
            break
        i, j = candidates[idx]
        if blocked[i:i + 2, j:j + 2].any():
            continue
        maze.rooms.append((i, j))
        in_room[i:i + 2, j:j + 2] = True
        blocked[max(i - 1, 0):i + 3, max(j - 1, 0):j + 3] = True
    return in_room


def carve_corridors(maze: Maze, in_room: np.ndarray, rng: np.random.Generator) -> None:
    """Randomized depth-first search over all corridor cells, starting at the entry."""
    n = maze.config.cells
    visited = np.zeros((n, n), dtype=bool)
    visited[maze.entry] = True
    stack = [maze.entry]
    while stack:
        cell = stack[-1]
        options = [nb for nb in neighbours(cell, n) if not in_room[nb] and not visited[nb]]
        if not options:
            stack.pop()
            continue
        nxt = options[int(rng.integers(len(options)))]
        open_wall(maze, cell, nxt)
        visited[nxt] = True
        stack.append(nxt)
    assert (visited | in_room).all(), 'corridor cells are not all connected'


def add_loops(maze: Maze, in_room: np.ndarray, rng: np.random.Generator) -> None:
    """Remove a share of the remaining walls between corridor cells, creating loops."""
    n = maze.config.cells
    walls = [((i, j), nb) for i in range(n) for j in range(n) for nb in ((i + 1, j), (i, j + 1))
             if nb[0] < n and nb[1] < n and not in_room[i, j] and not in_room[nb]
             and wall_present(maze, (i, j), nb)]
    count = int(round(maze.config.loop_fraction * len(walls)))
    for idx in rng.choice(len(walls), size=count, replace=False):
        open_wall(maze, *walls[idx])


def open_rooms(maze: Maze, in_room: np.ndarray, rng: np.random.Generator) -> None:
    """Clear each room's inner walls and open exactly one doorway to a corridor."""
    n = maze.config.cells
    for room in maze.rooms:
        cells = room_cells(room)
        for a in cells:
            for b in cells:
                if a < b and abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1:
                    open_wall(maze, a, b)
        doors = [(a, b) for a in cells for b in neighbours(a, n) if not in_room[b]]
        open_wall(maze, *doors[int(rng.integers(len(doors)))])


def place_survivors(maze: Maze, rng: np.random.Generator) -> None:
    """Put up to num_survivors survivors in distinct random rooms, away from the walls."""
    c = maze.config
    count = min(c.num_survivors, len(maze.rooms))
    for k in sorted(rng.choice(len(maze.rooms), size=count, replace=False)):
        x0, x1, y0, y1 = maze.room_box(int(k))
        m = c.survivor_margin
        maze.survivors.append({'room': int(k),
                               'x': float(rng.uniform(x0 + m, x1 - m)),
                               'y': float(rng.uniform(y0 + m, y1 - m))})


def generate(config: MazeConfig) -> Maze:
    """Build a maze from a config. The same config always gives the same maze."""
    rng = np.random.default_rng(config.seed)
    n = config.cells
    maze = Maze(config, wx=np.ones((n + 1, n), dtype=bool), wy=np.ones((n, n + 1), dtype=bool),
                entry=(0, n // 2))
    in_room = place_rooms(maze, rng)
    carve_corridors(maze, in_room, rng)
    add_loops(maze, in_room, rng)
    open_rooms(maze, in_room, rng)
    maze.wx[0, maze.entry[1]] = False          # entry/exit opening in the west outer wall
    place_survivors(maze, rng)
    return maze


# ---------------------------------------------------------------- geometry and checks

def wall_boxes(maze: Maze) -> list[tuple[float, float, float, float]]:
    """Merge standing walls into boxes: (centre_x, centre_y, size_x, size_y), world frame.

    Collinear neighbouring walls become one box; each box covers its end posts.
    """
    p, t = maze.config.pitch, maze.config.wall_thickness
    boxes = []
    for vertical, grid in ((True, maze.wx), (False, maze.wy)):
        lines, cells = grid.shape if vertical else grid.shape[::-1]
        for line in range(lines):
            run_start = None
            for k in range(cells + 1):
                standing = k < cells and (grid[line, k] if vertical else grid[k, line])
                if standing and run_start is None:
                    run_start = k
                elif not standing and run_start is not None:
                    length = (k - run_start) * p + t
                    along = (run_start + k) * p / 2
                    gx, gy = (line * p, along) if vertical else (along, line * p)
                    sx, sy = (t, length) if vertical else (length, t)
                    boxes.append((*maze.to_world(gx, gy), sx, sy))
                    run_start = None
    return boxes


def rasterize(maze: Maze) -> tuple[np.ndarray, tuple[float, float]]:
    """Ground-truth occupancy of the arena.

    Returns:
        (occupied, origin): occupied[row, col] is True for wall pixels, row 0 at the lowest y;
        origin is the world (x, y) of the lower-left corner of pixel (0, 0).
    """
    res = maze.config.resolution
    xmin, xmax, ymin, ymax = maze.bounds()
    cols, rows = int(round((xmax - xmin) / res)), int(round((ymax - ymin) / res))
    occupied = np.zeros((rows, cols), dtype=bool)
    for cx, cy, sx, sy in wall_boxes(maze):
        c0, c1 = (int(round((cx - sx / 2 - xmin) / res)), int(round((cx + sx / 2 - xmin) / res)))
        r0, r1 = (int(round((cy - sy / 2 - ymin) / res)), int(round((cy + sy / 2 - ymin) / res)))
        occupied[max(r0, 0):r1, max(c0, 0):c1] = True
    return occupied, (xmin, ymin)


def world_to_pixel(maze: Maze, origin: tuple[float, float], x: float, y: float) -> tuple[int, int]:
    """World (x, y) -> (row, col) in the rasterized grid."""
    res = maze.config.resolution
    return int((y - origin[1]) / res), int((x - origin[0]) / res)


def validate(maze: Maze) -> None:
    """Assert the competition rules. Raises AssertionError on any violation."""
    c = maze.config
    xmin, xmax, ymin, ymax = maze.bounds()
    assert xmax - xmin <= c.max_arena + 1e-9 and ymax - ymin <= c.max_arena + 1e-9, (
        f'arena {xmax - xmin:.2f} x {ymax - ymin:.2f} m exceeds {c.max_arena} m')
    assert c.rooms_min <= len(maze.rooms) <= c.rooms_max, f'{len(maze.rooms)} rooms placed'

    occupied, origin = rasterize(maze)
    free = ~occupied
    # Width: every free pixel lies inside some fully free cell_clear x cell_clear square.
    k = int(round(c.cell_clear / c.resolution))
    opened = ndimage.binary_opening(free, structure=np.ones((k, k), dtype=bool))
    assert np.array_equal(opened, free), 'a passage is narrower than the minimum corridor width'

    # Reachability: one free region, containing the entry, every room and every survivor.
    labels, count = ndimage.label(free)
    entry_label = labels[world_to_pixel(maze, origin, 0.0, 0.0)]
    assert count == 1 and entry_label == 1, f'free space split into {count} regions'
    for k_room in range(len(maze.rooms)):
        x0, x1, y0, y1 = maze.room_box(k_room)
        assert labels[world_to_pixel(maze, origin, (x0 + x1) / 2, (y0 + y1) / 2)] == entry_label
    for s in maze.survivors:
        x0, x1, y0, y1 = maze.room_box(s['room'])
        assert x0 < s['x'] < x1 and y0 < s['y'] < y1, 'survivor outside its room'


# ---------------------------------------------------------------- file output

def write_png(path: Path, rgb: np.ndarray) -> None:
    """Write an (H, W, 3) uint8 array as a PNG file (no imaging library needed)."""
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[r].tobytes() for r in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack('>I', len(data)) + tag + data + struct.pack(
            '>I', zlib.crc32(tag + data) & 0xFFFFFFFF)

    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                     + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def make_texture(rng: np.random.Generator, size: int = 256) -> np.ndarray:
    """Random high-contrast pattern (blotches, stripes, noise) for visual odometry tests."""
    img = np.full((size, size, 3), rng.integers(90, 170), dtype=np.int16)
    for _ in range(60):
        x, y = rng.integers(0, size, 2)
        w, h = rng.integers(8, 64, 2)
        img[y:y + h, x:x + w] = rng.integers(0, 256, 3)
    img += rng.integers(-25, 26, img.shape, dtype=np.int16)
    return np.clip(img, 0, 255).astype(np.uint8)


def box_sdf(name: str, pose: tuple[float, ...], size: tuple[float, ...], material: str,
            collide: bool = True) -> str:
    """SDF collision + visual pair for one box."""
    pose_s = ' '.join(f'{v:.4f}' for v in pose)
    size_s = ' '.join(f'{v:.4f}' for v in size)
    geom = f'<geometry><box><size>{size_s}</size></box></geometry>'
    col = f'<collision name="{name}_c"><pose>{pose_s}</pose>{geom}</collision>' if collide else ''
    return f'{col}<visual name="{name}_v"><pose>{pose_s}</pose>{geom}{material}</visual>\n'


def survivor_sdf(idx: int, x: float, y: float) -> str:
    """A simple human-sized survivor: 1.4 m body cylinder + head sphere (1.7 m total)."""
    mat = '<material><ambient>1 0.4 0 1</ambient><diffuse>1 0.4 0 1</diffuse></material>'
    body = '<geometry><cylinder><radius>0.2</radius><length>1.4</length></cylinder></geometry>'
    head = '<geometry><sphere><radius>0.15</radius></sphere></geometry>'
    return f"""  <model name="survivor_{idx}"><static>true</static><pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
    <link name="link">
      <collision name="body_c"><pose>0 0 0.7 0 0 0</pose>{body}</collision>
      <visual name="body_v"><pose>0 0 0.7 0 0 0</pose>{body}{mat}</visual>
      <collision name="head_c"><pose>0 0 1.55 0 0 0</pose>{head}</collision>
      <visual name="head_v"><pose>0 0 1.55 0 0 0</pose>{head}{mat}</visual>
    </link>
  </model>
"""


def world_sdf(maze: Maze, textures: list[Path], rng: np.random.Generator) -> str:
    """Full Gazebo world: plugins, light, floor, walls, net ceiling and survivors."""
    c = maze.config
    plain = ('<material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse>'
             '</material>')

    def wall_material() -> str:
        if not textures:
            return plain
        tex = textures[int(rng.integers(len(textures)))]
        return (f'<material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{tex}</albedo_map>'
                '</metal></pbr></material>')

    walls = ''.join(box_sdf(f'wall_{k}', (cx, cy, c.wall_height / 2, 0, 0, 0),
                            (sx, sy, c.wall_height), wall_material())
                    for k, (cx, cy, sx, sy) in enumerate(wall_boxes(maze)))
    xmin, xmax, ymin, ymax = maze.bounds()
    net_mat = ('<material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse>'
               '</material><transparency>0.8</transparency>')
    net = box_sdf('net', ((xmin + xmax) / 2, (ymin + ymax) / 2, c.wall_height + 0.01, 0, 0, 0),
                  (xmax - xmin, ymax - ymin, 0.02), net_mat)
    survivors = ''.join(survivor_sdf(k, s['x'], s['y']) for k, s in enumerate(maze.survivors))
    return f"""<?xml version="1.0"?>
<!-- Generated by generate_maze.py: seed {c.seed}, wall_texture {c.wall_texture}. Do not edit. -->
<sdf version="1.9">
<world name="maze">
  <physics name="4ms" type="ignored">
    <max_step_size>0.004</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.8 0.9 1</background>
    <shadows>false</shadows></scene>
  <light type="directional" name="sun">
    <pose>0 0 10 0 0 0</pose><cast_shadows>false</cast_shadows>
    <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
    <direction>-0.4 0.2 -0.9</direction>
  </light>
  <model name="ground_plane"><static>true</static>
    <link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        <material><ambient>0.5 0.45 0.4 1</ambient><diffuse>0.5 0.45 0.4 1</diffuse></material></visual>
    </link>
  </model>
  <model name="maze"><static>true</static>
    <link name="link">
{walls}{net}    </link>
  </model>
{survivors}</world>
</sdf>
"""


def write_outputs(maze: Maze, out_dir: Path) -> Path:
    """Validate the maze and write all output files. Returns the world file path."""
    validate(maze)
    c = maze.config
    out_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(c.seed + 1_000_003)     # texture stream, independent of layout
    textures = []
    if c.wall_texture == 'textured':
        for k in range(4):
            path = (out_dir / f'wall_{k}.png').resolve()
            write_png(path, make_texture(rng))
            textures.append(path)
    world_path = out_dir / 'world.sdf'
    world_path.write_text(world_sdf(maze, textures, rng))

    occupied, origin = rasterize(maze)
    image = np.where(occupied, 0, 254).astype(np.uint8)[::-1]      # row 0 = top (max y)
    rows, cols = image.shape
    (out_dir / 'layout.pgm').write_bytes(f'P5\n{cols} {rows}\n255\n'.encode() + image.tobytes())
    (out_dir / 'layout.yaml').write_text(
        f'image: layout.pgm\nresolution: {c.resolution}\n'
        f'origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

    xmin, xmax, ymin, ymax = maze.bounds()
    rooms = []
    for k in range(len(maze.rooms)):
        x0, x1, y0, y1 = maze.room_box(k)
        rooms.append({'id': k, 'center': [(x0 + x1) / 2, (y0 + y1) / 2],
                      'min': [x0, y0], 'max': [x1, y1]})
    layout = {
        'seed': c.seed,
        'config': asdict(c),
        'entry': {'position': [0.0, 0.0], 'direction': [1.0, 0.0]},
        'arena_bounds': {'xmin': xmin, 'xmax': xmax, 'ymin': ymin, 'ymax': ymax},
        'rooms': rooms,
        'survivors': [{'id': k, 'room': s['room'], 'position': [s['x'], s['y'], 0.0]}
                      for k, s in enumerate(maze.survivors)],
    }
    (out_dir / 'layout.json').write_text(json.dumps(layout, indent=2))
    return world_path


# ---------------------------------------------------------------- command line

def build_arg_parser() -> argparse.ArgumentParser:
    """Create the command-line parser for the maze generator.

    Returns:
        The configured argument parser.
    """
    d = MazeConfig(seed=0)
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', type=int, required=True, help='random seed (same seed, same maze)')
    parser.add_argument('--wall-texture', choices=('plain', 'textured'), default=d.wall_texture)
    parser.add_argument('--num-survivors', type=int, default=d.num_survivors)
    parser.add_argument('--cells', type=int, default=d.cells, help='grid size (cells per side)')
    parser.add_argument('--loop-fraction', type=float, default=d.loop_fraction)
    parser.add_argument('--output-root', type=Path, default=DEFAULT_OUTPUT_ROOT,
                        help='folder that receives seed_<N>_<texture>/')
    return parser


def main(argv: list[str] | None = None) -> int:
    """Entry point: parse arguments, generate and validate a maze, write its files.

    Args:
        argv: Command-line arguments (defaults to sys.argv[1:]).

    Returns:
        Process exit code. The world file path is printed on stdout.
    """
    args = build_arg_parser().parse_args(argv)
    config = MazeConfig(seed=args.seed, wall_texture=args.wall_texture,
                        num_survivors=args.num_survivors, cells=args.cells,
                        loop_fraction=args.loop_fraction)
    maze = generate(config)
    out_dir = args.output_root / f'seed_{config.seed}_{config.wall_texture}'
    print(write_outputs(maze, out_dir).resolve())
    return 0


if __name__ == '__main__':
    sys.exit(main())
