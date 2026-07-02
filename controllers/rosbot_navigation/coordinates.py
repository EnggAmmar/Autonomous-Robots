"""Coordinate conversion utilities.

Webots ground-plane navigation normally uses world X and Z.
The grid uses row and column.
"""
import math
from typing import Iterable, Tuple

import config


def world_to_grid(x: float, z: float) -> Tuple[int, int]:
    """Convert Webots world coordinate (x, z) to grid coordinate (row, col)."""
    col = int((x - config.MAP_ORIGIN_X) / config.GRID_RESOLUTION_M)
    row = int((z - config.MAP_ORIGIN_Z) / config.GRID_RESOLUTION_M)
    return row, col


def grid_to_world(row: int, col: int) -> Tuple[float, float]:
    """Convert grid coordinate (row, col) to Webots world coordinate (x, z)."""
    x = col * config.GRID_RESOLUTION_M + config.MAP_ORIGIN_X + config.GRID_RESOLUTION_M / 2.0
    z = row * config.GRID_RESOLUTION_M + config.MAP_ORIGIN_Z + config.GRID_RESOLUTION_M / 2.0
    return x, z


def in_bounds(row: int, col: int, shape: Tuple[int, int]) -> bool:
    return 0 <= row < shape[0] and 0 <= col < shape[1]


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance_2d(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def angle_to_target(robot_x: float, robot_z: float, target_x: float, target_z: float) -> float:
    return math.atan2(target_z - robot_z, target_x - robot_x)


def grid_path_to_world_path(path: Iterable[Tuple[int, int]]):
    return [grid_to_world(row, col) for row, col in path]
