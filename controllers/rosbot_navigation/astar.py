"""A* global path planner for an occupancy grid."""
import heapq
import math
from typing import Dict, List, Optional, Tuple

import config
from coordinates import in_bounds

GridCell = Tuple[int, int]

_NEIGHBORS_8 = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
]


def heuristic(a: GridCell, b: GridCell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def is_blocked(grid, cell: GridCell, allow_unknown: bool = False) -> bool:
    row, col = cell
    if not in_bounds(row, col, grid.shape):
        return True
    value = grid[row, col]
    if value in (config.OBSTACLE, config.FORBIDDEN_GREEN):
        return True
    if value == config.UNKNOWN and not allow_unknown:
        return True
    return False


def reconstruct_path(came_from: Dict[GridCell, GridCell], current: GridCell) -> List[GridCell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(grid, start: GridCell, goal: GridCell, allow_unknown: bool = False) -> Optional[List[GridCell]]:
    """Return a grid-cell path from start to goal, or None if no path exists."""
    if is_blocked(grid, start, allow_unknown=True):
        return None
    if is_blocked(grid, goal, allow_unknown=allow_unknown):
        return None

    open_heap = []
    heapq.heappush(open_heap, (0.0, start))

    came_from: Dict[GridCell, GridCell] = {}
    g_score: Dict[GridCell, float] = {start: 0.0}
    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return reconstruct_path(came_from, current)
        closed.add(current)

        for dr, dc, step_cost in _NEIGHBORS_8:
            neighbor = (current[0] + dr, current[1] + dc)
            if is_blocked(grid, neighbor, allow_unknown=allow_unknown):
                continue

            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    return None


def simplify_path(path: List[GridCell], step: int = 3) -> List[GridCell]:
    """Cheap simplification: keep every nth cell plus final cell."""
    if not path or len(path) <= 2:
        return path
    simplified = path[::step]
    if simplified[-1] != path[-1]:
        simplified.append(path[-1])
    return simplified
