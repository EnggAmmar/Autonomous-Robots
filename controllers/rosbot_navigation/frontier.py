"""Frontier exploration utilities."""
from collections import deque
from typing import List, Optional, Tuple

import numpy as np

import config
from coordinates import in_bounds

GridCell = Tuple[int, int]

_NEIGHBORS_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
_NEIGHBORS_8 = [
    (-1, 0), (1, 0), (0, -1), (0, 1),
    (-1, -1), (-1, 1), (1, -1), (1, 1),
]


def is_frontier_cell(grid: np.ndarray, row: int, col: int) -> bool:
    if grid[row, col] != config.FREE:
        return False
    for dr, dc in _NEIGHBORS_4:
        nr, nc = row + dr, col + dc
        if in_bounds(nr, nc, grid.shape) and grid[nr, nc] == config.UNKNOWN:
            return True
    return False


def find_frontier_cells(grid: np.ndarray) -> List[GridCell]:
    cells: List[GridCell] = []
    rows, cols = grid.shape
    for row in range(1, rows - 1):
        for col in range(1, cols - 1):
            if is_frontier_cell(grid, row, col):
                cells.append((row, col))
    return cells


def cluster_frontiers(grid: np.ndarray, frontier_cells: List[GridCell]) -> List[List[GridCell]]:
    frontier_set = set(frontier_cells)
    clusters = []

    while frontier_set:
        start = frontier_set.pop()
        cluster = [start]
        q = deque([start])
        while q:
            row, col = q.popleft()
            for dr, dc in _NEIGHBORS_8:
                nb = (row + dr, col + dc)
                if nb in frontier_set:
                    frontier_set.remove(nb)
                    q.append(nb)
                    cluster.append(nb)
        if len(cluster) >= config.FRONTIER_MIN_CLUSTER_SIZE:
            clusters.append(cluster)
    return clusters


def cluster_centroid(cluster: List[GridCell]) -> GridCell:
    row = int(sum(c[0] for c in cluster) / len(cluster))
    col = int(sum(c[1] for c in cluster) / len(cluster))
    return row, col


def choose_frontier(grid: np.ndarray, robot_cell: GridCell) -> Optional[GridCell]:
    """Choose nearest useful frontier centroid."""
    cells = find_frontier_cells(grid)
    clusters = cluster_frontiers(grid, cells)
    if not clusters:
        return None

    best = None
    best_score = float("inf")
    rr, rc = robot_cell
    for cluster in clusters:
        centroid = cluster_centroid(cluster)
        dist = ((centroid[0] - rr) ** 2 + (centroid[1] - rc) ** 2) ** 0.5
        # Prefer nearer and larger frontier clusters.
        score = dist - 0.25 * len(cluster)
        if score < best_score:
            best_score = score
            best = centroid
    return best
