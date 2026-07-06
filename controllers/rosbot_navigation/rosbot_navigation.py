"""Main Webots controller for the Autonomous Robots Modularbeit.

The controller uses only robot-mounted sensors: camera, GPS/compass, and lidar
when present. It never calls Webots Supervisor APIs.
"""

import math
from typing import Callable, Optional, Set, Tuple

from controller import Robot

from astar import astar, simplify_path
import config
from coordinates import (
    angle_to_target,
    distance_2d,
    grid_path_to_world_path,
    normalize_angle,
    world_to_grid,
)
from dwa import plan_dwa
from frontier import choose_frontier
from logger import EventLogger
from mapping import OccupancyGrid
from robot_interface import RobotInterface
from vision import (
    ColorDetection,
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

GridCell = Tuple[int, int]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def blue_goal_reached(blue) -> bool:
    return target_reached(blue)


def visual_servo_omega(horizontal_error: float) -> float:
    if abs(horizontal_error) < config.VISUAL_SERVO_DEADBAND:
        return 0.0
    omega = -horizontal_error * config.MAX_ANGULAR_SPEED * config.VISUAL_SERVO_ANGULAR_GAIN
    return clamp(omega, -config.MAX_ANGULAR_SPEED, config.MAX_ANGULAR_SPEED)


def visual_approach_speed(horizontal_error: float, area: int) -> float:
    if abs(horizontal_error) > config.VISUAL_APPROACH_ALIGN_ERROR:
        return 0.0

    v = config.VISUAL_APPROACH_FAST_MPS
    if area > 0.75 * config.TARGET_REACHED_AREA:
        v = min(v, 0.12)
    return v


def green_ahead(green: ColorDetection) -> bool:
    if not green.visible:
        return False
    if green.area < config.GREEN_AHEAD_MIN_AREA:
        return False
    y_frac = 0.0
    if green.center_y is not None and green.image_height > 0:
        y_frac = green.center_y / green.image_height
    return (
        y_frac >= config.GREEN_AHEAD_MIN_Y_FRAC
        and abs(green.horizontal_error) <= config.GREEN_AHEAD_MAX_ERROR
    )


def visual_bearing(pose, detection: ColorDetection, camera) -> Optional[float]:
    if pose is None or not detection.visible:
        return None
    try:
        fov = float(camera.getFov())
    except Exception:
        fov = math.radians(70.0)
    # Positive image error means the target is to the robot's right. With this
    # robot's heading convention, subtracting the offset matches visual_servo_omega.
    return normalize_angle(pose[2] - detection.horizontal_error * fov * 0.5)


def turn_away_from_green(iface: RobotInterface, green: ColorDetection):
    if green.visible and green.horizontal_error > 0.0:
        iface.set_velocity(0.0, config.SEARCH_ROTATION_SPEED)
    else:
        iface.set_velocity(0.0, -config.SEARCH_ROTATION_SPEED)


class ExplorationNavigator:
    def __init__(self, logger: EventLogger):
        self.grid = OccupancyGrid()
        self.logger = logger
        self.path_world = []
        self.frontier_goal: Optional[GridCell] = None
        self.failed_frontiers: Set[GridCell] = set()
        self.last_plan_time = -999.0

    def reset_goal_memory(self):
        self.path_world = []
        self.frontier_goal = None
        self.failed_frontiers.clear()
        self.last_plan_time = -999.0

    def update_map(self, iface: RobotInterface, pose, green: ColorDetection):
        if pose is None:
            return
        x, z, heading = pose
        self.grid.mark_robot_area_free(x, z)
        self.grid.raycast_update(x, z, heading, iface.get_lidar_ranges(), iface.get_lidar_fov())
        if green_ahead(green):
            self.grid.mark_green_ahead_approx(x, z, heading, config.GREEN_MARK_DISTANCE_M)
        self.grid.inflate_obstacles()

    def _plan_to_frontier(self, pose, preferred_bearing: Optional[float], sim_time: float) -> bool:
        if pose is None:
            return False
        start = world_to_grid(pose[0], pose[1])

        candidates = [
            choose_frontier(
                self.grid.grid,
                start,
                preferred_bearing=preferred_bearing,
                excluded=self.failed_frontiers,
                inflated=self.grid.inflated,
            ),
            choose_frontier(
                self.grid.grid,
                start,
                preferred_bearing=preferred_bearing,
                excluded=self.failed_frontiers,
                inflated=self.grid.inflated,
                min_cluster_size=1,
            ),
        ]

        for goal in candidates:
            if goal is None:
                continue
            path = astar(self.grid.inflated, start, goal, allow_unknown=False)
            if path:
                self.frontier_goal = goal
                self.path_world = grid_path_to_world_path(simplify_path(path))
                self.last_plan_time = sim_time
                if config.DEBUG_PLANNING:
                    self.logger.info(sim_time, f"Planned frontier path: cells={len(path)} goal={goal}")
                return True
            self.failed_frontiers.add(goal)

        self.path_world = []
        self.frontier_goal = None
        return False

    def _select_waypoint(self, pose) -> Optional[Tuple[float, float]]:
        while self.path_world and distance_2d((pose[0], pose[1]), self.path_world[0]) < config.WAYPOINT_TOLERANCE_M:
            self.path_world.pop(0)
        if not self.path_world:
            return None
        idx = min(config.PLANNING_WAYPOINT_LOOKAHEAD, len(self.path_world) - 1)
        return self.path_world[idx]

    def command(self, iface: RobotInterface, pose, preferred_bearing: Optional[float], sim_time: float):
        if pose is None:
            iface.set_velocity(0.0, config.SEARCH_ROTATION_SPEED)
            return

        need_plan = (
            not self.path_world
            or sim_time - self.last_plan_time >= config.PLAN_STALE_SECONDS
            or (self.frontier_goal is not None and self.grid.inflated[self.frontier_goal] in (config.OBSTACLE, config.FORBIDDEN_GREEN))
        )
        if need_plan and not self._plan_to_frontier(pose, preferred_bearing, sim_time):
            iface.set_velocity(0.0, config.NO_PATH_ROTATION_SPEED)
            return

        waypoint = self._select_waypoint(pose)
        if waypoint is None:
            iface.set_velocity(0.0, config.SEARCH_ROTATION_SPEED)
            return

        v, omega = plan_dwa(pose, waypoint, self.grid, self.path_world)
        if abs(v) < 1e-3 and abs(omega) < 1e-3:
            desired = angle_to_target(pose[0], pose[1], waypoint[0], waypoint[1])
            omega = clamp(normalize_angle(desired - pose[2]), -config.NO_PATH_ROTATION_SPEED, config.NO_PATH_ROTATION_SPEED)
        iface.set_velocity(v, omega)


def run_blue_only(robot: Robot, iface: RobotInterface, logger: EventLogger):
    state = SEARCH_BLUE
    step_count = 0

    while robot.step(config.TIME_STEP) != -1:
        step_count += 1
        sim_time = robot.getTime()
        logger.state(sim_time, state)

        bgr = webots_camera_to_bgr(iface.camera)
        if bgr is None:
            iface.stop()
            if step_count % config.PRINT_EVERY_N_STEPS == 0:
                logger.info(sim_time, "No camera image. Check camera device name.")
            continue

        blue = detect_blue(bgr)
        green = detect_green(bgr)

        if blue_goal_reached(blue):
            logger.blue_reached(sim_time)
            logger.info(sim_time, f"TIMING SUMMARY Start -> Blue: {sim_time:.2f} s")
            logger.info(sim_time, "BLUE_ONLY complete. Stopping robot.")
            state = DONE
            iface.stop()
            continue

        if state == DONE:
            iface.stop()
            continue

        if blue.visible:
            if state != GO_TO_BLUE:
                logger.info(sim_time, "Blue detected. Direct visual approach.")
                state = GO_TO_BLUE
            omega = visual_servo_omega(blue.horizontal_error)
            v = visual_approach_speed(blue.horizontal_error, blue.area)
            if config.DEBUG_VISUAL_SERVO and step_count % config.PRINT_EVERY_N_STEPS == 0:
                logger.info(
                    sim_time,
                    f"[BLUE_ONLY] blue=True area={blue.area} error={blue.horizontal_error:.3f} "
                    f"v={v:.3f} omega={omega:.3f} green={green.visible}",
                )
            iface.set_velocity(v, omega)
        else:
            if state != SEARCH_BLUE:
                logger.info(sim_time, "Blue lost. Scanning in place.")
                state = SEARCH_BLUE
            if config.DEBUG_VISUAL_SERVO and step_count % config.PRINT_EVERY_N_STEPS == 0:
                logger.info(
                    sim_time,
                    f"[BLUE_ONLY] blue=False area={blue.area} error={blue.horizontal_error:.3f} "
                    f"v=0.000 omega={config.SEARCH_ROTATION_SPEED:.3f} green={green.visible}",
                )
            iface.set_velocity(0.0, config.SEARCH_ROTATION_SPEED)


def run_blue_then_yellow(robot: Robot, iface: RobotInterface, logger: EventLogger):
    state = SEARCH_BLUE
    active_name = "blue"
    active_detector: Callable = detect_blue
    last_seen_bearing: Optional[float] = None
    navigator = ExplorationNavigator(logger)
    step_count = 0

    while robot.step(config.TIME_STEP) != -1:
        step_count += 1
        sim_time = robot.getTime()
        logger.state(sim_time, state)

        pose = iface.get_pose()
        bgr = webots_camera_to_bgr(iface.camera)
        if bgr is None:
            iface.stop()
            if step_count % config.PRINT_EVERY_N_STEPS == 0:
                logger.info(sim_time, "No camera image. Check camera device name.")
            continue

        blue = detect_blue(bgr)
        yellow = detect_yellow(bgr)
        green = detect_green(bgr)
        target = active_detector(bgr)

        navigator.update_map(iface, pose, green)

        if active_name == "blue" and target_reached(blue):
            logger.blue_reached(sim_time)
            active_name = "yellow"
            active_detector = detect_yellow
            last_seen_bearing = None
            navigator.reset_goal_memory()
            state = SEARCH_YELLOW
            iface.stop()
            continue

        if active_name == "yellow" and target_reached(yellow):
            logger.yellow_reached(sim_time)
            state = DONE
            iface.stop()
            continue

        if state == DONE:
            iface.stop()
            continue

        hazard = green_ahead(green)
        if hazard and pose is not None:
            navigator.grid.mark_green_ahead_approx(pose[0], pose[1], pose[2], config.GREEN_MARK_DISTANCE_M)
            navigator.grid.inflate_obstacles()

        if target.visible and target.area >= config.DIRECT_TARGET_MIN_AREA and not hazard:
            state = GO_TO_BLUE if active_name == "blue" else GO_TO_YELLOW
            last_seen_bearing = visual_bearing(pose, target, iface.camera)
            omega = visual_servo_omega(target.horizontal_error)
            v = visual_approach_speed(target.horizontal_error, target.area)
            if config.DEBUG_VISUAL_SERVO and step_count % config.PRINT_EVERY_N_STEPS == 0:
                logger.info(
                    sim_time,
                    f"[{active_name.upper()}] visible area={target.area} error={target.horizontal_error:.3f} "
                    f"v={v:.3f} omega={omega:.3f} green_ahead={hazard}",
                )
            iface.set_velocity(v, omega)
            continue

        if hazard:
            if config.DEBUG_VISUAL_SERVO and step_count % config.PRINT_EVERY_N_STEPS == 0:
                logger.info(
                    sim_time,
                    f"Green floor ahead while seeking {active_name}; rotating away "
                    f"area={green.area} error={green.horizontal_error:.3f}",
                )
            navigator.reset_goal_memory()
            turn_away_from_green(iface, green)
            continue

        state = SEARCH_BLUE if active_name == "blue" else SEARCH_YELLOW
        navigator.command(iface, pose, last_seen_bearing, sim_time)


def main():
    robot = Robot()
    iface = RobotInterface(robot)
    logger = EventLogger()

    print("========================================")
    print("RosBot navigation controller started")
    print("Mode:", config.MODE)
    print("Target mode:", getattr(config, "TARGET_MODE", "BLUE_ONLY"))
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

    target_mode = getattr(config, "TARGET_MODE", "BLUE_ONLY")
    if target_mode == "BLUE_ONLY":
        run_blue_only(robot, iface, logger)
    else:
        run_blue_then_yellow(robot, iface, logger)


if __name__ == "__main__":
    main()
