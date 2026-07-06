"""Main Webots controller for the Autonomous Robots Modularbeit.

Development modes:
1. DEVICE_SCAN: prints available device names and does not move.
2. MOTOR_TEST: waits three seconds, then drives forward, turns, and stops.
3. RUN: runs the navigation state machine.

Important: This is starter code. You must test, tune, and understand it before
submission. Add your own comments/references according to the assignment rules.
"""
import math

from controller import Robot

import config
from astar import astar, simplify_path
from coordinates import distance_2d, grid_path_to_world_path, in_bounds, normalize_angle, world_to_grid
from dwa import plan_dwa
from frontier import choose_frontier, find_frontier_cells
from logger import EventLogger
from mapping import OccupancyGrid
from robot_interface import RobotInterface
from vision import (
    detect_blue,
    detect_green,
    detect_yellow,
    print_camera_hsv_stats,
    target_reached,
    webots_camera_to_bgr,
)

SEARCH_BLUE = "SEARCH_BLUE"
GO_TO_BLUE = "GO_TO_BLUE"
SEARCH_YELLOW = "SEARCH_YELLOW"
GO_TO_YELLOW = "GO_TO_YELLOW"
DONE = "DONE"


def visual_servo_omega(horizontal_error: float) -> float:
    """Proportional turning command for GO_TO_BLUE/GO_TO_YELLOW."""
    if abs(horizontal_error) < config.VISUAL_SERVO_DEADBAND:
        return 0.0
    omega = -horizontal_error * config.MAX_ANGULAR_SPEED * config.VISUAL_SERVO_ANGULAR_GAIN
    return max(-config.MAX_ANGULAR_SPEED, min(config.MAX_ANGULAR_SPEED, omega))


def visual_approach_speed(horizontal_error: float) -> float:
    """Drive toward a visible pillar while still allowing hard turns.

    The old value was 0.01 m/s when the target was near the image edge, which
    looked like the robot was frozen while doing a 360-degree scan. This keeps a
    small but real forward speed even while aligning.
    """
    if abs(horizontal_error) < config.VISUAL_APPROACH_ALIGN_ERROR:
        return config.VISUAL_APPROACH_FAST_MPS
    return config.VISUAL_APPROACH_SLOW_MPS


def plan_path_to_frontier(grid_map, robot_cell, inflated, last_seen_bearing, min_cluster_size=None):
    """Try several frontier candidates against this tick's inflated grid."""
    tried_goals = set()
    frontier_goal = None
    path_cells = None
    for _ in range(config.FRONTIER_MAX_RETRIES):
        frontier_goal = choose_frontier(
            grid_map.grid, robot_cell, preferred_bearing=last_seen_bearing,
            excluded=tried_goals, inflated=inflated, min_cluster_size=min_cluster_size,
        )
        if frontier_goal is None:
            break

        candidate = astar(inflated, robot_cell, frontier_goal, allow_unknown=False)
        if candidate is None:
            candidate = astar(inflated, robot_cell, frontier_goal, allow_unknown=True)

        if candidate is not None:
            path_cells = candidate
            break
        tried_goals.add(frontier_goal)
    return path_cells, frontier_goal, tried_goals


def get_next_waypoint(pose, world_path, current_index):
    if not world_path:
        return None, current_index
    x, z, _ = pose
    while current_index < len(world_path) - 1:
        if distance_2d((x, z), world_path[current_index]) < config.WAYPOINT_TOLERANCE_M:
            current_index += 1
        else:
            break
    return world_path[current_index], current_index


def main():
    robot = Robot()
    iface = RobotInterface(robot)
    logger = EventLogger()

    print("========================================")
    print("RosBot navigation controller started")
    print("Mode:", config.MODE)
    print("========================================")
    iface.print_devices()

    if config.MODE == "DEVICE_SCAN":
        while robot.step(config.TIME_STEP) != -1:
            iface.stop()
        return

    if config.MODE == "CAMERA_CALIBRATION":
        print("CAMERA_CALIBRATION: point the camera at each pillar/the green floor and read the printed HSV ranges.")
        while robot.step(config.TIME_STEP) != -1:
            iface.stop()
            bgr = webots_camera_to_bgr(iface.camera)
            if bgr is not None:
                print_camera_hsv_stats(bgr)
        return

    if config.MODE == "MOTOR_TEST":
        print("MOTOR_TEST: wait 3 seconds, drive forward, turn, then stop.")
        motor_test_step = 0
        while robot.step(config.TIME_STEP) != -1:
            t = robot.getTime()
            motor_test_step += 1
            if motor_test_step % 10 == 0:
                pose = iface.get_pose()
                if pose is not None:
                    print(f"t={t:.1f}s pose=({pose[0]:.3f},{pose[1]:.3f}) heading={pose[2]:.2f} rad")
            if t < 3.0:
                iface.stop()
            elif t < 6.0:
                iface.set_velocity(0.18, 0.0)
            elif t < 8.0:
                iface.set_velocity(0.0, 0.70)
            else:
                iface.stop()
        return

    grid_map = OccupancyGrid()
    state = SEARCH_BLUE
    path_cells = None
    path_world = []
    waypoint_index = 0
    last_plan_time = -999.0
    step_count = 0
    last_seen_bearing = None
    last_pose_xz = None
    stuck_reference_xz = None
    stuck_reference_time = 0.0
    stuck_warned = False
    no_path_start_time = None
    visited_waypoints = []

    while robot.step(config.TIME_STEP) != -1:
        step_count += 1
        sim_time = robot.getTime()
        logger.state(sim_time, state)

        pose = iface.get_pose()
        if pose is None:
            iface.stop()
            if step_count % config.PRINT_EVERY_N_STEPS == 0:
                logger.info(sim_time, "No GPS pose available. Check device names.")
            continue

        robot_x, robot_z, robot_heading = pose
        robot_cell = world_to_grid(robot_x, robot_z)

        if last_pose_xz is not None:
            step_dist = distance_2d(last_pose_xz, (robot_x, robot_z))
            max_expected_step = config.MAX_LINEAR_SPEED * (config.TIME_STEP / 1000.0) * config.PHYSICS_JUMP_MULTIPLIER
            if step_dist > max_expected_step:
                logger.info(
                    sim_time,
                    f"SUSPECTED PHYSICS CORRUPTION: pose jumped {step_dist:.3f}m in one step "
                    f"(max plausible ~{max_expected_step:.3f}m) from {last_pose_xz} to ({robot_x:.3f},{robot_z:.3f})",
                )
        last_pose_xz = (robot_x, robot_z)

        if stuck_reference_xz is None:
            stuck_reference_xz = (robot_x, robot_z)
            stuck_reference_time = sim_time
        elif distance_2d(stuck_reference_xz, (robot_x, robot_z)) > config.STUCK_POSITION_EPSILON_M:
            stuck_reference_xz = (robot_x, robot_z)
            stuck_reference_time = sim_time
            stuck_warned = False
        elif state != DONE and not stuck_warned and (sim_time - stuck_reference_time) > config.STUCK_TIME_THRESHOLD_S:
            logger.info(
                sim_time,
                f"STUCK: position has not moved >{config.STUCK_POSITION_EPSILON_M}m in "
                f"{sim_time - stuck_reference_time:.1f}s (state={state}, pose=({robot_x:.3f},{robot_z:.3f}))",
            )
            stuck_warned = True

        grid_map.mark_robot_area_free(robot_x, robot_z)
        lidar_ranges = iface.get_lidar_ranges()
        grid_map.raycast_update(robot_x, robot_z, robot_heading, lidar_ranges, iface.get_lidar_fov())

        bgr = webots_camera_to_bgr(iface.camera)
        blue = yellow = green = None
        if bgr is not None:
            blue = detect_blue(bgr)
            yellow = detect_yellow(bgr)
            green = detect_green(bgr)
            if green.visible and green.center_y is not None and green.center_y > int(0.55 * green.image_height):
                grid_map.mark_green_ahead_approx(robot_x, robot_z, robot_heading)

        sought = blue if state in (SEARCH_BLUE, GO_TO_BLUE) else yellow if state in (SEARCH_YELLOW, GO_TO_YELLOW) else None
        if sought is not None and sought.visible:
            last_seen_bearing = normalize_angle(robot_heading - sought.horizontal_error * (math.pi / 4.0))

        inflated = grid_map.inflate_obstacles()

        if step_count % config.PRINT_EVERY_N_STEPS == 0:
            logger.info(
                sim_time,
                f"pose=({robot_x:.2f},{robot_z:.2f},{robot_heading:.2f}) "
                f"blue={blue.visible if blue else None} yellow={yellow.visible if yellow else None} "
                f"green={green.visible if green else None}",
            )

        if state == SEARCH_BLUE and blue is not None and blue.visible:
            logger.info(sim_time, "Blue detected. Switching to visual approach.")
            state = GO_TO_BLUE
            path_cells = None
            path_world = []
            waypoint_index = 0

        if state == GO_TO_BLUE and blue is not None and target_reached(blue):
            logger.blue_reached(sim_time)
            state = SEARCH_YELLOW
            path_cells = None
            path_world = []
            waypoint_index = 0
            last_seen_bearing = None
            iface.stop()
            continue

        if state == SEARCH_YELLOW and yellow is not None and yellow.visible:
            logger.info(sim_time, "Yellow detected. Switching to visual approach.")
            state = GO_TO_YELLOW
            path_cells = None
            path_world = []
            waypoint_index = 0

        if state == GO_TO_YELLOW and yellow is not None and target_reached(yellow):
            logger.yellow_reached(sim_time)
            state = DONE
            iface.stop()
            continue

        if state == DONE:
            iface.stop()
            continue

        if state == GO_TO_BLUE and blue is not None and blue.visible:
            omega = visual_servo_omega(blue.horizontal_error)
            v = visual_approach_speed(blue.horizontal_error)
            if config.DEBUG_VISUAL_SERVO:
                logger.info(
                    sim_time,
                    f"[DEBUG_VISUAL_SERVO] target=blue error={blue.horizontal_error:.3f} "
                    f"omega={omega:.3f} v={v:.3f} area={blue.area} pose=({robot_x:.3f},{robot_z:.3f})",
                )
            iface.set_velocity(v, omega)
            continue

        if state == GO_TO_YELLOW and yellow is not None and yellow.visible:
            omega = visual_servo_omega(yellow.horizontal_error)
            v = visual_approach_speed(yellow.horizontal_error)
            if config.DEBUG_VISUAL_SERVO:
                logger.info(
                    sim_time,
                    f"[DEBUG_VISUAL_SERVO] target=yellow error={yellow.horizontal_error:.3f} "
                    f"omega={omega:.3f} v={v:.3f} area={yellow.area} pose=({robot_x:.3f},{robot_z:.3f})",
                )
            iface.set_velocity(v, omega)
            continue

        need_replan = path_cells is None or (sim_time - last_plan_time) > config.REPLAN_INTERVAL_S
        if need_replan:
            path_cells, frontier_goal, tried_goals = plan_path_to_frontier(
                grid_map, robot_cell, inflated, last_seen_bearing
            )

            if path_cells is None:
                relaxed_path, relaxed_goal, relaxed_tried = plan_path_to_frontier(
                    grid_map, robot_cell, inflated, last_seen_bearing, min_cluster_size=1
                )
                if relaxed_path is not None:
                    path_cells, frontier_goal = relaxed_path, relaxed_goal
                elif relaxed_goal is not None:
                    frontier_goal = relaxed_goal
                tried_goals |= relaxed_tried

            if frontier_goal is None and not tried_goals:
                if no_path_start_time is None:
                    no_path_start_time = sim_time
                iface.set_velocity(0.0, config.SEARCH_ROTATION_SPEED)
                continue

            backtracking = False
            if path_cells is None and visited_waypoints:
                candidates = list(reversed(visited_waypoints))
                for waypoint in candidates:
                    backtrack_path = astar(inflated, robot_cell, waypoint, allow_unknown=True)
                    if backtrack_path is not None:
                        path_cells = backtrack_path
                        visited_waypoints.remove(waypoint)
                        backtracking = True
                        logger.info(sim_time, f"No frontier reachable. Backtracking to {waypoint}.")
                        break

            if path_cells is not None:
                if no_path_start_time is not None:
                    logger.info(sim_time, f"Recovered from 'No A* path' after {sim_time - no_path_start_time:.2f}s")
                    no_path_start_time = None
                path_cells = simplify_path(path_cells, step=4)
                path_world = grid_path_to_world_path(path_cells)
                waypoint_index = 0
                last_plan_time = sim_time
                if not backtracking:
                    logger.info(sim_time, f"Planned path to frontier. cells={len(path_cells)}")
                    if not visited_waypoints or visited_waypoints[-1] != robot_cell:
                        visited_waypoints.append(robot_cell)
                        if len(visited_waypoints) > config.MAX_VISITED_WAYPOINTS:
                            visited_waypoints.pop(0)
            else:
                if no_path_start_time is None:
                    no_path_start_time = sim_time
                stall_duration = sim_time - no_path_start_time
                if config.DEBUG_PLANNING:
                    rr, rc = robot_cell
                    raw_here = grid_map.grid[rr, rc] if in_bounds(rr, rc, grid_map.grid.shape) else None
                    inf_here = inflated[rr, rc] if in_bounds(rr, rc, inflated.shape) else None
                    if frontier_goal is not None:
                        fr, fc = frontier_goal
                        raw_goal = grid_map.grid[fr, fc] if in_bounds(fr, fc, grid_map.grid.shape) else None
                        inf_goal = inflated[fr, fc] if in_bounds(fr, fc, inflated.shape) else None
                    else:
                        raw_goal = inf_goal = None
                    free_count = int((grid_map.grid == config.FREE).sum())
                    frontier_count = len(find_frontier_cells(grid_map.grid))
                    inflation_cells = max(1, int(config.OBSTACLE_INFLATION_M / config.GRID_RESOLUTION_M))
                    logger.info(
                        sim_time,
                        f"[DEBUG_PLANNING] robot_cell={robot_cell} raw={raw_here} inflated={inf_here} | "
                        f"last_frontier_goal={frontier_goal} raw={raw_goal} inflated={inf_goal} "
                        f"tried={len(tried_goals)} visited_waypoints={len(visited_waypoints)} | "
                        f"free_cells={free_count} frontier_candidates={frontier_count} | "
                        f"OBSTACLE_INFLATION_M={config.OBSTACLE_INFLATION_M:.3f} "
                        f"inflation_cells={inflation_cells} (GRID_RESOLUTION_M={config.GRID_RESOLUTION_M})",
                    )
                logger.info(sim_time, f"No A* path found. Rotating. (stalled {stall_duration:.2f}s)")
                iface.set_velocity(0.0, config.NO_PATH_ROTATION_SPEED)
                continue

        local_goal, waypoint_index = get_next_waypoint(pose, path_world, waypoint_index)
        if local_goal is None:
            iface.set_velocity(0.0, config.SEARCH_ROTATION_SPEED)
            continue

        v, omega = plan_dwa(pose, local_goal, grid_map, local_path=path_world[waypoint_index:])
        iface.set_velocity(v, omega)


if __name__ == "__main__":
    main()
