"""Occupancy grid mapping.

This is intentionally simple. It marks robot neighborhood as free and, if lidar is
available, ray-casts ranges into the grid. Tune lidar angle mapping once you know
your sensor's exact Webots device.
"""
import math
from typing import Optional, Tuple

import numpy as np

import config
from coordinates import in_bounds, world_to_grid

GridCell = Tuple[int, int]


class OccupancyGrid:
    def __init__(self):
        rows = int(config.MAP_HEIGHT_M / config.GRID_RESOLUTION_M)
        cols = int(config.MAP_WIDTH_M / config.GRID_RESOLUTION_M)
        self.grid = np.full((rows, cols), config.UNKNOWN, dtype=np.int8)
        self.inflated = self.grid.copy()

    def mark_cell(self, row: int, col: int, value: int):
        if in_bounds(row, col, self.grid.shape):
            # Forbidden green and obstacle should not be overwritten by free.
            if value == config.FREE and self.grid[row, col] in (config.OBSTACLE, config.FORBIDDEN_GREEN):
                return
            self.grid[row, col] = value

    def mark_robot_area_free(self, x: float, z: float, radius_m: float = 0.25):
        center = world_to_grid(x, z)
        radius_cells = max(1, int(radius_m / config.GRID_RESOLUTION_M))
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr * dr + dc * dc <= radius_cells * radius_cells:
                    self.mark_cell(center[0] + dr, center[1] + dc, config.FREE)

    def mark_world_point(self, x: float, z: float, value: int):
        row, col = world_to_grid(x, z)
        self.mark_cell(row, col, value)

    def raycast_update(self, robot_x: float, robot_z: float, robot_heading: float, ranges, fov: float):
        """Update grid with lidar ranges.

        ranges: list of distances in meters.
        fov: field of view in radians.
        """
        if ranges is None or len(ranges) == 0:
            return
        n = len(ranges)
        max_range = 5.0
        for i, r in enumerate(ranges):
            if r is None or math.isinf(r) or math.isnan(r):
                r = max_range
            r = min(float(r), max_range)
            angle = robot_heading - fov / 2.0 + fov * i / max(1, n - 1)
            self._trace_ray(robot_x, robot_z, angle, r, hit=(r < max_range * 0.98))

    def _trace_ray(self, x: float, z: float, angle: float, distance: float, hit: bool):
        step = config.GRID_RESOLUTION_M
        travelled = 0.0
        while travelled < distance:
            px = x + travelled * math.cos(angle)
            pz = z + travelled * math.sin(angle)
            self.mark_world_point(px, pz, config.FREE)
            travelled += step
        if hit:
            ox = x + distance * math.cos(angle)
            oz = z + distance * math.sin(angle)
            self.mark_world_point(ox, oz, config.OBSTACLE)

    def mark_green_ahead_approx(self, robot_x: float, robot_z: float, robot_heading: float, distance_m: float = 0.45):
        """Crude emergency marker: if camera sees green near bottom, mark cells ahead as forbidden.

        This is not a perfect camera projection. It is a safe starter approximation.
        """
        for d in np.linspace(0.15, distance_m, 5):
            gx = robot_x + d * math.cos(robot_heading)
            gz = robot_z + d * math.sin(robot_heading)
            self.mark_world_point(gx, gz, config.FORBIDDEN_GREEN)

    def inflate_obstacles(self):
        inflated = self.grid.copy()
        radius_cells = max(1, int(config.OBSTACLE_INFLATION_M / config.GRID_RESOLUTION_M))
        obstacle_cells = np.argwhere((self.grid == config.OBSTACLE) | (self.grid == config.FORBIDDEN_GREEN))
        for row, col in obstacle_cells:
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    if dr * dr + dc * dc <= radius_cells * radius_cells:
                        nr, nc = row + dr, col + dc
                        if in_bounds(nr, nc, inflated.shape):
                            if self.grid[row, col] == config.FORBIDDEN_GREEN:
                                inflated[nr, nc] = config.FORBIDDEN_GREEN
                            else:
                                inflated[nr, nc] = config.OBSTACLE
        self.inflated = inflated
        return inflated

    def is_safe_world(self, x: float, z: float) -> bool:
        row, col = world_to_grid(x, z)
        if not in_bounds(row, col, self.inflated.shape):
            return False
        return self.inflated[row, col] not in (config.OBSTACLE, config.FORBIDDEN_GREEN)
