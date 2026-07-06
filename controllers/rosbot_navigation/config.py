"""Configuration values for the Webots RosBot navigation project.

Tune this file first. Keep numbers here instead of hard-coding them in algorithms.
"""

# Start with DEVICE_SCAN. After you see motor/sensor names in the Webots console,
# change MODE to "MOTOR_TEST", then later to "RUN". Use "CAMERA_CALIBRATION" any
# time to print HSV stats of the current camera frame for tuning the thresholds
# below against the real pillars/floor.
MODE = "RUN"  # options: "DEVICE_SCAN", "RUN", "MOTOR_TEST", "CAMERA_CALIBRATION"

TIME_STEP = 32  # ms

# Robot geometry for the SimpleRosBot training PROTO.
# When you switch to the official course RosBot, re-tune these values.
WHEEL_RADIUS_M = 0.055
TRACK_WIDTH_M = 0.32
ROBOT_RADIUS_M = 0.23
SAFETY_MARGIN_M = 0.06

# Motor polarity. In the SimpleRosBot training PROTO the camera/front points along
# robot -Z while the wheel hinge axes are +X. A positive wheel speed therefore
# drives opposite to the controller's forward-heading convention. Keep the turn
# sign separate so MOTOR_TEST can diagnose direction without editing algorithms.
MOTOR_FORWARD_SIGN = -1.0
MOTOR_TURN_SIGN = 1.0

# Speed limits. These are still conservative, but high enough that the robot does
# not spend the whole run rotating in place.
MAX_LINEAR_SPEED = 0.55       # m/s
MAX_ANGULAR_SPEED = 1.80      # rad/s
MAX_WHEEL_SPEED = 16.0        # rad/s, used as safety clamp

# Search/visual approach speeds.
SEARCH_ROTATION_SPEED = 0.70
NO_PATH_ROTATION_SPEED = 0.70
VISUAL_APPROACH_FAST_MPS = 0.22
VISUAL_APPROACH_SLOW_MPS = 0.08
VISUAL_APPROACH_ALIGN_ERROR = 0.45

# GO_TO_BLUE/GO_TO_YELLOW visual-servo proportional controller (rosbot_navigation.py).
# MAX_ANGULAR_SPEED above stays a hard clamp; this is the working gain applied to
# horizontal_error before that clamp. Use CAMERA_CALIBRATION if detection is wrong.
VISUAL_SERVO_ANGULAR_GAIN = 0.75
VISUAL_SERVO_DEADBAND = 0.04

# Lidar self-detection zone (mapping.py's raycast_update). Confirmed empirically
# by placing the robot at two unrelated world positions with the same heading and
# comparing raw lidar ranges: from about +7-8 degrees off dead-ahead through the
# full +90 degree edge of the 180-degree FOV, the reported ranges were IDENTICAL
# (down to mm) at both positions -- proof this side of the FOV is always looking
# at the robot's own body/wheel, not the environment, regardless of where the
# robot actually is. protos/ can't be modified, so these rays are excluded from
# every scan instead of being trusted as real obstacles.
LIDAR_SELF_OCCLUSION_ANGLE_MIN_RAD = 0.1047  # ~6 degrees off dead-ahead
LIDAR_SELF_OCCLUSION_ANGLE_MAX_RAD = 1.658   # ~95 degrees off dead-ahead

# Map settings. Webots navigation plane is usually X-Z. Grid cells are in meters.
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

# Planning
WAYPOINT_TOLERANCE_M = 0.24
GOAL_TOLERANCE_M = 0.35
REPLAN_INTERVAL_S = 0.7
FRONTIER_MIN_CLUSTER_SIZE = 4
FRONTIER_BEARING_WEIGHT = 40.0
FRONTIER_MAX_RETRIES = 8
MAX_VISITED_WAYPOINTS = 40

# DWA local planner
DWA_DT = 0.10
DWA_PREDICTION_TIME = 1.0
DWA_LINEAR_SAMPLES = 6
DWA_ANGULAR_SAMPLES = 11
DWA_GOAL_WEIGHT = 2.2
DWA_CLEARANCE_WEIGHT = 0.7
DWA_SPEED_WEIGHT = 0.8
DWA_PATH_WEIGHT = 0.7

# Vision thresholds in HSV. Tune using Webots camera images.
# OpenCV hue is 0..179, saturation/value are 0..255.
BLUE_HSV_LOW = (95, 70, 50)
BLUE_HSV_HIGH = (130, 255, 255)

YELLOW_HSV_LOW = (15, 70, 50)
YELLOW_HSV_HIGH = (40, 255, 255)

GREEN_HSV_LOW = (45, 70, 50)
GREEN_HSV_HIGH = (85, 255, 255)

MIN_COLOR_AREA = 120
TARGET_REACHED_AREA = 3800

# Debugging
PRINT_EVERY_N_STEPS = 20
DEBUG_SHOW_GREEN_MARKS = False
DEBUG_PLANNING = True
DEBUG_VISUAL_SERVO = False
DEBUG_MOTOR_COMMANDS = True

# Always-on lightweight watchdogs.
PHYSICS_JUMP_MULTIPLIER = 4.0
STUCK_POSITION_EPSILON_M = 0.05
STUCK_TIME_THRESHOLD_S = 4.0
