"""Configuration values for the Webots RosBot navigation project.

Tune this file first. Keep numbers here instead of hard-coding them in algorithms.
"""

# Start with DEVICE_SCAN. After you see motor/sensor names in the Webots console,
# change MODE to "MOTOR_TEST", then later to "RUN".
MODE = "DEVICE_SCAN"  # options: "DEVICE_SCAN", "RUN", "MOTOR_TEST"

TIME_STEP = 32  # ms

# Robot geometry for the SimpleRosBot training PROTO.
# When you switch to the official course RosBot, re-tune these values.
WHEEL_RADIUS_M = 0.055
TRACK_WIDTH_M = 0.32
ROBOT_RADIUS_M = 0.23
SAFETY_MARGIN_M = 0.08

# Speed limits. Start slow, then increase.
MAX_LINEAR_SPEED = 0.30       # m/s
MAX_ANGULAR_SPEED = 1.20      # rad/s
MAX_WHEEL_SPEED = 12.0        # rad/s, used as safety clamp

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
WAYPOINT_TOLERANCE_M = 0.20
GOAL_TOLERANCE_M = 0.35
REPLAN_INTERVAL_S = 1.0
FRONTIER_MIN_CLUSTER_SIZE = 5

# DWA local planner
DWA_DT = 0.10
DWA_PREDICTION_TIME = 1.2
DWA_LINEAR_SAMPLES = 5
DWA_ANGULAR_SAMPLES = 9
DWA_GOAL_WEIGHT = 2.0
DWA_CLEARANCE_WEIGHT = 0.8
DWA_SPEED_WEIGHT = 0.3
DWA_PATH_WEIGHT = 0.8

# Vision thresholds in HSV. Tune using Webots camera images.
# OpenCV hue is 0..179, saturation/value are 0..255.
BLUE_HSV_LOW = (95, 80, 40)
BLUE_HSV_HIGH = (130, 255, 255)
YELLOW_HSV_LOW = (20, 80, 60)
YELLOW_HSV_HIGH = (40, 255, 255)
GREEN_HSV_LOW = (45, 70, 40)
GREEN_HSV_HIGH = (85, 255, 255)

MIN_COLOR_AREA = 150
TARGET_REACHED_AREA = 4500

# Debugging
PRINT_EVERY_N_STEPS = 20
