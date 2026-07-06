"""Frontier exploration utilities."""
import math
from collections import deque
from typing import Iterable, List, Optional, Tuple

import numpy as np

import config
from coordinates import in_bounds, normalize_angle

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


def cluster_frontiers(
    grid: np.ndarray,
    frontier_cells: List[GridCell],
    min_cluster_size: Optional[int] = None,
) -> List[List[GridCell]]:
    """Group frontier cells into connected clusters of at least min_cluster_size
    (defaults to config.FRONTIER_MIN_CLUSTER_SIZE). Pass a lower value (e.g. 1)
    to fall back to small clusters when nothing larger is reachable -- see
    choose_frontier's relaxed_min_cluster_size.
    """
    if min_cluster_size is None:
        min_cluster_size = config.FRONTIER_MIN_CLUSTER_SIZE
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
        if len(cluster) >= min_cluster_size:
            clusters.append(cluster)
    return clusters


def cluster_centroid(cluster: List[GridCell]) -> GridCell:
    row = int(sum(c[0] for c in cluster) / len(cluster))
    col = int(sum(c[1] for c in cluster) / len(cluster))
    return row, col


def cluster_representative(
    cluster: List[GridCell],
    robot_cell: GridCell,
    inflated: Optional[np.ndarray] = None,
) -> GridCell:
    """Pick the cluster member nearest the robot, instead of the arithmetic mean.

    The mean of a non-convex (e.g. L-shaped) cluster can land on a cell that
    isn't even part of the cluster and is not itself a frontier cell. Always
    returning a real member guarantees the target is at least a genuine FREE,
    frontier-adjacent cell in the raw grid.

    The single nearest member is frequently one hugging a wall the robot is
    following -- exactly the kind of cell inflation blocks in the *inflated*
    grid A* actually searches. If given, prefer the nearest member that is
    NOT blocked there, so a cluster with hundreds of reachable members doesn't
    get discarded over one bad pick.
    """
    rr, rc = robot_cell
    if inflated is not None:
        open_members = [
            c for c in cluster
            if inflated[c] not in (config.OBSTACLE, config.FORBIDDEN_GREEN)
        ]
        if open_members:
            return min(open_members, key=lambda c: (c[0] - rr) ** 2 + (c[1] - rc) ** 2)
    return min(cluster, key=lambda c: (c[0] - rr) ** 2 + (c[1] - rc) ** 2)


def choose_frontier(
    grid: np.ndarray,
    robot_cell: GridCell,
    preferred_bearing: Optional[float] = None,
    excluded: Optional[Iterable[GridCell]] = None,
    inflated: Optional[np.ndarray] = None,
    min_cluster_size: Optional[int] = None,
) -> Optional[GridCell]:
    """Choose a useful frontier centroid.

    preferred_bearing: if given (radians, atan2(dz, dx) convention), frontiers
    roughly in that direction are preferred over the globally nearest/largest
    one. Pass the last-seen bearing of the current target (blue/yellow) here
    when it has been glimpsed but then lost, to avoid wasting time exploring
    away from a target already spotted once.

    excluded: centroids to skip. The grid barely changes tick-to-tick while the
    robot is stationary/rotating, so this call is deterministic; without this,
    a frontier goal that A* just failed to reach (e.g. it's inside inflation
    radius of a wall) would be re-selected forever. Pass previously-failed
    goals here to fall through to the next-best candidate instead.

    inflated: the inflated grid A* will actually search. Passing it lets the
    representative pick for each cluster avoid inflation-blocked members
    (see cluster_representative) instead of only ever offering up a point
    A* is guaranteed to reject.

    min_cluster_size: overrides config.FRONTIER_MIN_CLUSTER_SIZE. A small (e.g.
    single-cell) frontier immediately next to the robot can be its only
    reachable one -- everything else it can see may belong to a large cluster
    that's genuinely disconnected from the robot's current pocket (a real
    narrow-passage dead end). The normal minimum exists to avoid chasing noisy
    single-cell frontier artifacts, so callers should try the default first and
    only pass a lower value as an explicit fallback once that's exhausted.
    """
    cells = find_frontier_cells(grid)
    clusters = cluster_frontiers(grid, cells, min_cluster_size=min_cluster_size)
    if not clusters:
        return None

    excluded_set = set(excluded) if excluded else None
    best = None
    best_score = float("inf")
    rr, rc = robot_cell
    for cluster in clusters:
        centroid = cluster_representative(cluster, robot_cell, inflated=inflated)
        if excluded_set is not None and centroid in excluded_set:
            continue
        dist = ((centroid[0] - rr) ** 2 + (centroid[1] - rc) ** 2) ** 0.5
        # Prefer nearer and larger frontier clusters.
        score = dist - 0.25 * len(cluster)
        if preferred_bearing is not None and dist > 0:
            bearing_to_frontier = math.atan2(centroid[0] - rr, centroid[1] - rc)
            bearing_error = abs(normalize_angle(bearing_to_frontier - preferred_bearing))
            score += config.FRONTIER_BEARING_WEIGHT * bearing_error
        if score < best_score:
            best_score = score
            best = centroid
    return best
