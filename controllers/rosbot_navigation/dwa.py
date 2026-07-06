"""Dynamic Window Approach local planner.

This is a compact starter DWA. It samples velocity commands, simulates short arcs,
rejects unsafe commands, and returns the best (v, omega).
"""
import math
from typing import List, Optional, Tuple

import numpy as np

import config
from coordinates import angle_to_target, distance_2d, normalize_angle

Pose = Tuple[float, float, float]  # x, z, heading
Waypoint = Tuple[float, float]


def simulate_trajectory(pose: Pose, v: float, omega: float) -> List[Pose]:
    x, z, theta = pose
    traj = []
    steps = int(config.DWA_PREDICTION_TIME / config.DWA_DT)
    for _ in range(steps):
        x += v * math.cos(theta) * config.DWA_DT
        z += v * math.sin(theta) * config.DWA_DT
        theta = normalize_angle(theta + omega * config.DWA_DT)
        traj.append((x, z, theta))
    return traj


def trajectory_clearance(traj: List[Pose], occupancy_grid) -> float:
    min_clearance = float("inf")
    for x, z, _ in traj:
        if not occupancy_grid.is_safe_world(x, z):
            return -1.0
        min_clearance = min(min_clearance, occupancy_grid.clearance_world(x, z))
    # Cap so one very-open trajectory doesn't dominate the score over goal/heading terms.
    return min(min_clearance, 1.0)


def path_distance_score(traj_end: Pose, local_path: Optional[List[Waypoint]]) -> float:
    if not local_path:
        return 0.0
    end_xy = (traj_end[0], traj_end[1])
    return min(distance_2d(end_xy, p) for p in local_path[:8])


def plan_dwa(pose: Pose, local_goal: Waypoint, occupancy_grid, local_path: Optional[List[Waypoint]] = None) -> Tuple[float, float]:
    best_score = -float("inf")
    best_cmd = (0.0, 0.0)

    v_values = np.linspace(0.0, config.MAX_LINEAR_SPEED, config.DWA_LINEAR_SAMPLES)
    w_values = np.linspace(-config.MAX_ANGULAR_SPEED, config.MAX_ANGULAR_SPEED, config.DWA_ANGULAR_SAMPLES)

    for v in v_values:
        for omega in w_values:
            traj = simulate_trajectory(pose, float(v), float(omega))
            if not traj:
                continue

            clearance = trajectory_clearance(traj, occupancy_grid)
            if clearance < 0.0:
                continue

            end_x, end_z, end_theta = traj[-1]
            desired_heading = angle_to_target(end_x, end_z, local_goal[0], local_goal[1])
            heading_error = abs(normalize_angle(desired_heading - end_theta))
            goal_dist = distance_2d((end_x, end_z), local_goal)
            path_dist = path_distance_score(traj[-1], local_path)

            score = 0.0
            score += config.DWA_GOAL_WEIGHT * (-goal_dist)
            score += config.DWA_GOAL_WEIGHT * (-heading_error)
            score += config.DWA_CLEARANCE_WEIGHT * clearance
            score += config.DWA_SPEED_WEIGHT * v
            score += config.DWA_PATH_WEIGHT * (-path_dist)

            if score > best_score:
                best_score = score
                best_cmd = (float(v), float(omega))

    return best_cmd
