"""Read and write 2D occupancy maps as PGM image + YAML (map_server format).

Grid convention used everywhere in tools/: an int8 numpy array, row 0 = lowest y,
values 100 = occupied, 0 = free, -1 = unknown (same as nav_msgs/OccupancyGrid data).
`origin` is the map-frame (x, y) of the lower-left corner of cell (0, 0).
Image convention (map_server, negate 0): 0 = occupied, 254 = free, 205 = unknown, row 0 = top.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

OCCUPIED, FREE, UNKNOWN = 100, 0, -1
PIXEL = {OCCUPIED: 0, FREE: 254, UNKNOWN: 205}
OCCUPIED_THRESH, FREE_THRESH = 0.65, 0.196


@dataclass
class OccupancyMap:
    """A 2D occupancy grid with its placement in the map frame."""

    grid: np.ndarray                 # int8 (rows, cols), row 0 = lowest y
    resolution: float                # m per cell
    origin: tuple[float, float]      # map-frame (x, y) of cell (0, 0)'s lower-left corner
    meta: dict = field(default_factory=dict)


def from_occupancy_values(values: np.ndarray) -> np.ndarray:
    """Map OccupancyGrid values (-1, 0..100) to the trinary grid using map_server thresholds."""
    v = np.asarray(values, dtype=np.int16)
    grid = np.full(v.shape, UNKNOWN, dtype=np.int8)
    grid[(v >= 0) & (v / 100.0 > OCCUPIED_THRESH)] = OCCUPIED
    grid[(v >= 0) & (v / 100.0 < FREE_THRESH)] = FREE
    return grid


def save(path: Path, occ: OccupancyMap) -> Path:
    """Write `<path>.pgm` and `<path>.yaml` (path given without suffix). Returns the YAML path."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    image = np.full(occ.grid.shape, PIXEL[UNKNOWN], dtype=np.uint8)
    image[occ.grid == OCCUPIED] = PIXEL[OCCUPIED]
    image[occ.grid == FREE] = PIXEL[FREE]
    rows, cols = image.shape
    pgm = path.with_suffix('.pgm')
    pgm.write_bytes(f'P5\n{cols} {rows}\n255\n'.encode() + image[::-1].tobytes())
    info = {'image': pgm.name, 'mode': 'trinary', 'resolution': float(occ.resolution),
            'origin': [float(occ.origin[0]), float(occ.origin[1]), 0.0], 'negate': 0,
            'occupied_thresh': OCCUPIED_THRESH, 'free_thresh': FREE_THRESH}
    if occ.meta:
        info['airmouse'] = occ.meta
    yml = path.with_suffix('.yaml')
    yml.write_text(yaml.safe_dump(info, sort_keys=False))
    return yml


def load(yaml_path: Path) -> OccupancyMap:
    """Read a map_server YAML + PGM (also the maze generator's ground-truth layout)."""
    yaml_path = Path(yaml_path)
    info = yaml.safe_load(yaml_path.read_text())
    data = (yaml_path.parent / info['image']).read_bytes()
    parts = data.split(maxsplit=4)              # b'P5', cols, rows, maxval, pixels
    cols, rows, maxval = int(parts[1]), int(parts[2]), int(parts[3])
    pixels = np.frombuffer(parts[4][:rows * cols], dtype=np.uint8).reshape(rows, cols)[::-1]
    p = (maxval - pixels.astype(float)) / maxval if not info.get('negate') else pixels / maxval
    grid = np.full((rows, cols), UNKNOWN, dtype=np.int8)
    grid[p > info.get('occupied_thresh', OCCUPIED_THRESH)] = OCCUPIED
    grid[p < info.get('free_thresh', FREE_THRESH)] = FREE
    return OccupancyMap(grid, float(info['resolution']), tuple(info['origin'][:2]),
                        info.get('airmouse', {}))
