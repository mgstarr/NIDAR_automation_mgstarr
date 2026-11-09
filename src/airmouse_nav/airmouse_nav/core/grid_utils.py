"""Occupancy-grid helpers shared by the planner and the explorer (no ROS imports).

Grid convention: a numpy int8 array of shape (rows, cols); row 0 = lowest y, column 0 = lowest
x; values as in nav_msgs/OccupancyGrid (0..100 = occupancy probability, -1 = unknown).
`GridInfo.origin` is the map-frame (x, y) of the lower-left corner of cell (0, 0).

Public API (Phases 3 and 5):

    info = GridInfo(resolution, origin_x, origin_y, rows, cols)
    grid = grid_from_occupancy(data, width, height)        # flat row-major data -> 2D array
    row, col = world_to_cell(info, x, y)                   # metres -> cell (may be off-grid)
    x, y = cell_to_world(info, row, col)                   # cell -> metres (cell centre)
    in_bounds(info, row, col) -> bool
    obstacles = obstacle_mask(grid)                        # occupied OR unknown
    clearance = clearance_map(obstacles, resolution)       # metres from each cell to an obstacle
    pg = build_planning_grid(grid, info, guarded_radius=0.30, safety_margin=0.10,
                             wall_cost_range=0.5)          # -> PlanningGrid

A PlanningGrid holds `blocked` (bool: occupied, unknown, or within guarded_radius +
safety_margin of an occupied cell; unknown cells are not inflated) and `wall_cost` (float 0..1: 1 just outside the inflation, falling
linearly to 0 at `wall_cost_range` metres from the nearest obstacle), plus `clearance`
(metres to the nearest obstacle) and `components` (free-region ids, for instant "no path").
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy import ndimage

OCCUPIED_THRESHOLD = 50   # values >= this count as obstacles (map_server trinary uses 100)


@dataclass(frozen=True)
class GridInfo:
    """Placement of a grid in the map frame."""

    resolution: float   # metres per cell
    origin_x: float     # map-frame x of the lower-left corner of cell (0, 0)
    origin_y: float     # map-frame y of the lower-left corner of cell (0, 0)
    rows: int           # number of rows (cells along y)
    cols: int           # number of columns (cells along x)


@dataclass(frozen=True)
class PlanningGrid:
    """What the planner searches on. Build it once per map, then plan many times."""

    info: GridInfo
    blocked: np.ndarray     # bool (rows, cols): True = never enter
    wall_cost: np.ndarray   # float64 (rows, cols): 0..1, 0 away from walls
    clearance: np.ndarray   # float64 (rows, cols): metres to the nearest obstacle cell edge
    components: np.ndarray  # int32 (rows, cols): free-space region id (0 = blocked)
    inflation_radius: float


def grid_from_occupancy(data: Sequence[int] | np.ndarray, width: int, height: int) -> np.ndarray:
    """Turn flat OccupancyGrid data (row-major, row 0 = lowest y) into a (height, width) array.

    Args:
        data: `nav_msgs/OccupancyGrid.data` values (-1 or 0..100).
        width: Number of columns (`info.width`).
        height: Number of rows (`info.height`).

    Returns:
        int8 array of shape (height, width).

    Raises:
        ValueError: If the data length does not match width x height.
    """
    arr = np.asarray(data, dtype=np.int8)
    if arr.size != width * height:
        raise ValueError(f'grid data has {arr.size} values, expected {width} x {height}')
    return arr.reshape(height, width)


def world_to_cell(info: GridInfo, x: float, y: float) -> tuple[int, int]:
    """Return the (row, col) of the cell containing map-frame point (x, y). May be off-grid."""
    col = int(np.floor((x - info.origin_x) / info.resolution))
    row = int(np.floor((y - info.origin_y) / info.resolution))
    return row, col


def cell_to_world(info: GridInfo, row: int, col: int) -> tuple[float, float]:
    """Return the map-frame (x, y) of the centre of cell (row, col)."""
    return (info.origin_x + (col + 0.5) * info.resolution,
            info.origin_y + (row + 0.5) * info.resolution)


def in_bounds(info: GridInfo, row: int, col: int) -> bool:
    """True if (row, col) lies on the grid."""
    return 0 <= row < info.rows and 0 <= col < info.cols


def obstacle_mask(grid: np.ndarray) -> np.ndarray:
    """Cells that are occupied or unknown (unknown counts as blocked)."""
    return (grid < 0) | (grid >= OCCUPIED_THRESHOLD)


def clearance_map(obstacles: np.ndarray, resolution: float) -> np.ndarray:
    """Distance in metres from each cell centre to the nearest obstacle cell's edge.

    Uses the Euclidean distance transform between cell centres, minus half a cell, so a free
    cell touching a wall has clearance resolution / 2. Obstacle cells get 0. With no obstacles
    at all, every cell gets +inf.
    """
    if not obstacles.any():
        return np.full(obstacles.shape, np.inf)
    dist = ndimage.distance_transform_edt(~obstacles) * resolution
    return np.where(obstacles, 0.0, dist - resolution / 2.0)


def build_planning_grid(grid: np.ndarray, info: GridInfo, guarded_radius: float = 0.30,
                        safety_margin: float = 0.10,
                        wall_cost_range: float = 0.5) -> PlanningGrid:
    """Inflate obstacles and compute the wall cost.

    Args:
        grid: int8 occupancy array (rows, cols), row 0 = lowest y.
        info: Grid placement; rows/cols must match `grid.shape`.
        guarded_radius: Drone radius including prop guards (m).
        safety_margin: Extra clearance added to the inflation (m).
        wall_cost_range: Distance from an obstacle (m) at which the wall cost reaches 0.

    Returns:
        PlanningGrid with `blocked` = occupied, unknown, or clearance from occupied cells <
        guarded_radius + safety_margin (unknown cells are not inflated), and `wall_cost` = 1 at the inflation edge falling linearly to 0 at
        `wall_cost_range` (0 everywhere if the range is not beyond the inflation radius).

    Raises:
        ValueError: If the grid shape does not match `info`.
    """
    if grid.shape != (info.rows, info.cols):
        raise ValueError(f'grid shape {grid.shape} does not match info ({info.rows}, {info.cols})')
    radius = guarded_radius + safety_margin
    occupied = grid >= OCCUPIED_THRESHOLD
    unknown = grid < 0
    # Only real (occupied) cells are inflated; unknown cells are blocked but not inflated
    # (Phase 4, owner approved): inflating unknown space blocked the drone's own surroundings
    # near the entry. Paths are replanned as the map grows, so walls hidden in unknown space are
    # inflated as soon as they are seen.
    clearance = clearance_map(occupied, info.resolution)
    blocked = occupied | unknown | (clearance < radius - 1e-9)
    span = wall_cost_range - radius
    if span > 0:
        wall_cost = np.clip((wall_cost_range - clearance) / span, 0.0, 1.0)
    else:
        wall_cost = np.zeros(grid.shape)
    wall_cost[blocked] = 1.0
    # 4-connected regions: equal to the planner's 8-connected moves, since a diagonal move
    # needs both side cells free.
    components, _ = ndimage.label(~blocked)
    return PlanningGrid(info=info, blocked=blocked, wall_cost=wall_cost, clearance=clearance,
                        components=components.astype(np.int32), inflation_radius=radius)
