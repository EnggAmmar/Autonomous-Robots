"""Main Webots controller for the Autonomous Robots Modularbeit.

Development modes:
1. DEVICE_SCAN: prints available device names.
2. MOTOR_TEST: slowly drives forward so you can verify motor names.
3. RUN: runs the starter navigation state machine.

Important: This is starter code. You must test, tune, and understand it before
submission. Add your own comments/references according to the assignment rules.
"""
from controller import Robot

import config
from astar import astar, simplify_path
from coordinates import distance_2d, grid_path_to_world_path, world_to_grid
from dwa import plan_dwa
from frontier import choose_frontier
from logger import EventLogger
from mapping import OccupancyGrid
from robot_interface import RobotInterface
from vision import detect_blue, detect_green, detect_yellow, target_reached, webots_camera_to_bgr

SEARCH_BLUE = "SEARCH_BLUE"
GO_TO_BLUE = "GO_TO_BLUE"
SEARCH_YELLOW = "SEARCH_YELLOW"
GO_TO_YELLOW = "GO_TO_YELLOW"
DONE = "DONE"


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
            pass
        return

    if config.MODE == "MOTOR_TEST":
        step = 0
        while robot.step(config.TIME_STEP) != -1:
            step += 1
            if step < 100:
                iface.set_velocity(0.12, 0.0)
            elif step < 180:
                iface.set_velocity(0.0, 0.6)
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

        # Mapping update.
        grid_map.mark_robot_area_free(robot_x, robot_z)
        lidar_ranges = iface.get_lidar_ranges()
        grid_map.raycast_update(robot_x, robot_z, robot_heading, lidar_ranges, iface.get_lidar_fov())

        # Vision update.
        bgr = webots_camera_to_bgr(iface.camera)
        blue = yellow = green = None
        if bgr is not None:
            blue = detect_blue(bgr)
            yellow = detect_yellow(bgr)
            green = detect_green(bgr)
            if green.visible and green.center_y is not None and green.center_y > int(0.55 * green.image_height):
                grid_map.mark_green_ahead_approx(robot_x, robot_z, robot_heading)

        inflated = grid_map.inflate_obstacles()

        if step_count % config.PRINT_EVERY_N_STEPS == 0:
            logger.info(
                sim_time,
                f"pose=({robot_x:.2f},{robot_z:.2f},{robot_heading:.2f}) "
                f"blue={blue.visible if blue else None} yellow={yellow.visible if yellow else None} "
                f"green={green.visible if green else None}",
            )

        # State transitions based on color target.
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

        # Visual servoing when pillar is visible.
        if state == GO_TO_BLUE and blue is not None and blue.visible:
            omega = -blue.horizontal_error * config.MAX_ANGULAR_SPEED
            v = 0.12 if abs(blue.horizontal_error) < 0.35 else 0.02
            iface.set_velocity(v, omega)
            continue

        if state == GO_TO_YELLOW and yellow is not None and yellow.visible:
            omega = -yellow.horizontal_error * config.MAX_ANGULAR_SPEED
            v = 0.12 if abs(yellow.horizontal_error) < 0.35 else 0.02
            iface.set_velocity(v, omega)
            continue

        # Exploration with frontier + A* + DWA.
        need_replan = path_cells is None or (sim_time - last_plan_time) > config.REPLAN_INTERVAL_S
        if need_replan:
            frontier_goal = choose_frontier(grid_map.grid, robot_cell)
            if frontier_goal is None:
                # If no frontier found, rotate to search with the camera.
                iface.set_velocity(0.0, 0.5)
                continue

            path_cells = astar(inflated, robot_cell, frontier_goal, allow_unknown=False)
            if path_cells is None:
                # Fallback: allow unknown during early development. Safer version should avoid unknown.
                path_cells = astar(inflated, robot_cell, frontier_goal, allow_unknown=True)

            if path_cells is not None:
                path_cells = simplify_path(path_cells, step=4)
                path_world = grid_path_to_world_path(path_cells)
                waypoint_index = 0
                last_plan_time = sim_time
                logger.info(sim_time, f"Planned path to frontier. cells={len(path_cells)}")
            else:
                logger.info(sim_time, "No A* path found. Rotating.")
                iface.set_velocity(0.0, 0.5)
                continue

        local_goal, waypoint_index = get_next_waypoint(pose, path_world, waypoint_index)
        if local_goal is None:
            iface.set_velocity(0.0, 0.4)
            continue

        v, omega = plan_dwa(pose, local_goal, grid_map, local_path=path_world[waypoint_index:])
        iface.set_velocity(v, omega)


if __name__ == "__main__":
    main()
