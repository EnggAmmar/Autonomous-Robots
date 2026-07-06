"""Configuration values for the Webots RosBot navigation project.

Current test target: reach the BLUE pillar as fast as possible and stop.
Keep numbers here instead of hard-coding them in algorithms.
"""

# Modes: "DEVICE_SCAN", "RUN", "MOTOR_TEST", "CAMERA_CALIBRATION"
MODE = "RUN"

# Target mode: "BLUE_ONLY" stops immediately after reaching the blue pillar.
# Use this while your current target is only blue. Later, change to "BLUE_THEN_YELLOW".
TARGET_MODE = "BLUE_ONLY"

TIME_STEP = 32  # ms

# Robot geometry for the SimpleRosBot training PROTO.
WHEEL_RADIUS_M = 0.055
TRACK_WIDTH_M = 0.32
ROBOT_RADIUS_M = 0.23
SAFETY_MARGIN_M = 0.06

# Your MOTOR_TEST showed that +1.0 is the correct forward sign in this world.
MOTOR_FORWARD_SIGN = 1.0
MOTOR_TURN_SIGN = 1.0

# Speed limits. Do not use values like 5 or 10 here; these are m/s and rad/s.
MAX_LINEAR_SPEED = 0.70       # m/s
MAX_ANGULAR_SPEED = 2.00      # rad/s
MAX_WHEEL_SPEED = 18.0        # rad/s, safety clamp

# Blue-only behaviour.
SEARCH_ROTATION_SPEED = 0.65
NO_PATH_ROTATION_SPEED = 0.65
VISUAL_APPROACH_FAST_MPS = 0.45
VISUAL_APPROACH_SLOW_MPS = 0.16
VISUAL_APPROACH_ALIGN_ERROR = 0.55

# Visual servo. Lower gain = less overshoot, higher gain = faster centering.
VISUAL_SERVO_ANGULAR_GAIN = 0.55
VISUAL_SERVO_DEADBAND = 0.035

# Lidar self-detection zone; kept for later mapping modes.
LIDAR_SELF_OCCLUSION_ANGLE_MIN_RAD = 0.1047
LIDAR_SELF_OCCLUSION_ANGLE_MAX_RAD = 1.658

# Map settings. Kept for later mapping modes.
GRID_RESOLUTION_M = 0.05
MAP_WIDTH_M = 20.0
MAP_HEIGHT_M = 20.0
MAP_ORIGIN_X = -MAP_WIDTH_M / 2.0
MAP_ORIGIN_Z = -MAP_HEIGHT_M / 2.0

UNKNOWN = -1
FREE = 0
OBSTACLE = 1
FORBIDDEN_GREEN = 2
OBSTACLE_INFLATION_M = ROBOT_RADIUS_M + SAFETY_MARGIN_M

# Planning settings. Kept for later mapping/yellow modes.
WAYPOINT_TOLERANCE_M = 0.24
GOAL_TOLERANCE_M = 0.35
REPLAN_INTERVAL_S = 0.7
FRONTIER_MIN_CLUSTER_SIZE = 4
FRONTIER_BEARING_WEIGHT = 40.0
FRONTIER_MAX_RETRIES = 8
MAX_VISITED_WAYPOINTS = 40

# DWA settings. Kept for later mapping/yellow modes.
DWA_DT = 0.10
DWA_PREDICTION_TIME = 1.0
DWA_LINEAR_SAMPLES = 6
DWA_ANGULAR_SAMPLES = 11
DWA_GOAL_WEIGHT = 2.2
DWA_CLEARANCE_WEIGHT = 0.7
DWA_SPEED_WEIGHT = 0.8
DWA_PATH_WEIGHT = 0.7

# HSV thresholds. Your calibration screenshot for blue showed saturated H roughly
# 107..119, S roughly 81..177, V roughly 66..176. These values keep a safe margin.
BLUE_HSV_LOW = (100, 55, 45)
BLUE_HSV_HIGH = (125, 255, 255)

YELLOW_HSV_LOW = (15, 70, 50)
YELLOW_HSV_HIGH = (40, 255, 255)

GREEN_HSV_LOW = (45, 70, 50)
GREEN_HSV_HIGH = (85, 255, 255)

MIN_COLOR_AREA = 100
# Stop when the blue blob is large enough. Increase if it stops too early;
# decrease if it touches the pillar before stopping.
TARGET_REACHED_AREA = 8500

# Debugging
PRINT_EVERY_N_STEPS = 10
DEBUG_SHOW_GREEN_MARKS = False
DEBUG_PLANNING = False
DEBUG_VISUAL_SERVO = True
DEBUG_MOTOR_COMMANDS = True

PHYSICS_JUMP_MULTIPLIER = 4.0
STUCK_POSITION_EPSILON_M = 0.05
STUCK_TIME_THRESHOLD_S = 4.0
