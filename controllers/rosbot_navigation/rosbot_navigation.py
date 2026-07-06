"""Main Webots controller for the Autonomous Robots Modularbeit.

For the current test, TARGET_MODE="BLUE_ONLY" drives directly to the blue pillar
and stops. This avoids frontier/DWA wandering while you tune speed, wheel signs,
and camera thresholds.
"""

from controller import Robot

import config
from logger import EventLogger
from robot_interface import RobotInterface
from vision import (
    detect_blue,
    detect_green,
    detect_yellow,
    print_camera_hsv_stats,
    webots_camera_to_bgr,
)

SEARCH_BLUE = "SEARCH_BLUE"
GO_TO_BLUE = "GO_TO_BLUE"
SEARCH_YELLOW = "SEARCH_YELLOW"
GO_TO_YELLOW = "GO_TO_YELLOW"
DONE = "DONE"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def blue_goal_reached(blue) -> bool:
    return (
        blue.visible
        and blue.area >= config.TARGET_REACHED_AREA
        and abs(blue.horizontal_error) <= config.TARGET_REACHED_MAX_ERROR
    )


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


def run_blue_then_yellow_placeholder(robot: Robot, iface: RobotInterface, logger: EventLogger):
    logger.info(robot.getTime(), "TARGET_MODE is BLUE_THEN_YELLOW, but this tuned file currently runs BLUE_ONLY logic first.")
    run_blue_only(robot, iface, logger)


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
        run_blue_then_yellow_placeholder(robot, iface, logger)


if __name__ == "__main__":
    main()
