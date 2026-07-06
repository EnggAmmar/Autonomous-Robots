"""Configuration values for the Webots RosBot navigation project.

Tune this file first. Keep numbers here instead of hard-coding them in algorithms.
"""

# Start with DEVICE_SCAN. After you see motor/sensor names in the Webots console,
# change MODE to "MOTOR_TEST", then later to "RUN". Use "CAMERA_CALIBRATION" any
# time to print HSV stats of the current camera frame for tuning the thresholds
# below against the real pillars/floor.
MODE = "RUN" # options: "DEVICE_SCAN", "RUN", "MOTOR_TEST", "CAMERA_CALIBRATION"

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

# GO_TO_BLUE/GO_TO_YELLOW visual-servo proportional controller (rosbot_navigation.py).
# MAX_ANGULAR_SPEED above stays a hard clamp; this is the working gain applied to
# horizontal_error before that clamp. At gain 1.0 the loop oscillates (heading
# swings between +/-max every frame, no net progress) because bearing-to-target
# grows more sensitive to heading as range closes. Verified via closed-loop
# simulation: gain=1.0 leaves the robot stuck near its start distance after 30s
# simulated; gain=0.4 + the deadband below reaches ~0.2m of a 1m-away target in
# under 10s simulated. VISUAL_SERVO_DEADBAND ignores tiny centering jitter
# (error below this magnitude) that would otherwise seed the oscillation.
VISUAL_SERVO_ANGULAR_GAIN = 0.4
VISUAL_SERVO_DEADBAND = 0.05

# Lidar self-detection zone (mapping.py's raycast_update). Confirmed empirically
# by placing the robot at two unrelated world positions with the same heading and
# comparing raw lidar ranges: from about +7-8 degrees off dead-ahead through the
# full +90 degree edge of the 180-degree FOV, the reported ranges were IDENTICAL
# (down to mm) at both positions -- proof this side of the FOV is always looking
# at the robot's own body/wheel, not the environment, regardless of where the
# robot actually is. protos/ can't be modified, so these rays are excluded from
# every scan instead of being trusted as real obstacles (which is what was
# causing the robot to wall itself into a shrinking pocket over time). The other
# side of the FOV (0 to +6 degrees, and the full -90 to 0 side) tested clean.
LIDAR_SELF_OCCLUSION_ANGLE_MIN_RAD = 0.1047  # ~6 degrees off dead-ahead
LIDAR_SELF_OCCLUSION_ANGLE_MAX_RAD = 1.658  # ~95 degrees off dead-ahead (past the FOV edge, for margin)

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
# Weight (grid-cell-equivalent per radian) biasing frontier choice toward the
# last-seen bearing of a glimpsed-then-lost target. Tune if exploration seems
# to ignore or over-fixate on that direction.
FRONTIER_BEARING_WEIGHT = 40.0
# Max frontier candidates to try (excluding ones A* just failed on) within a
# single replan before giving up and rotating for this tick.
FRONTIER_MAX_RETRIES = 6
# Backtracking: when the robot's current reachable pocket has been fully
# explored (no frontier left anywhere within it, even at relaxed cluster size)
# while frontier cells still exist elsewhere in the map, retrace to an earlier
# waypoint instead of rotating forever -- that waypoint may have led to a
# different, still-unexplored branch that was never taken. Confirmed via grid
# dump: a real too-narrow-passage pocket can be exhaustively mapped with zero
# reachable frontier cells while the rest of the map remains reachable only
# from further back along the path already travelled.
MAX_VISITED_WAYPOINTS = 40

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
BLUE_HSV_LOW = (95, 70, 50)
BLUE_HSV_HIGH = (130, 255, 255)

YELLOW_HSV_LOW = (15, 70, 50)
YELLOW_HSV_HIGH = (40, 255, 255)

GREEN_HSV_LOW = (45, 70, 50)
GREEN_HSV_HIGH = (85, 255, 255)






MIN_COLOR_AREA = 150
TARGET_REACHED_AREA = 4500

# Debugging
PRINT_EVERY_N_STEPS = 20
DEBUG_SHOW_GREEN_MARKS = False  # log world coords marked as forbidden green
DEBUG_PLANNING = True  # log grid/frontier/A* diagnostics on "No A* path found"
DEBUG_VISUAL_SERVO = False  # log horizontal_error/omega/v during GO_TO_BLUE/GO_TO_YELLOW

# Always-on lightweight watchdogs (cheap; only print when something looks
# wrong, so they stay silent during normal operation). These are diagnostics,
# not fixes -- a real corrupted physics state can't be corrected by code, but
# it should be immediately obvious in the log rather than silently producing
# a nonsensical result during a real graded run.
PHYSICS_JUMP_MULTIPLIER = 4.0  # warn if one-step GPS movement exceeds this x the max plausible step
STUCK_POSITION_EPSILON_M = 0.05  # movement below this counts as "not moving"
STUCK_TIME_THRESHOLD_S = 5.0  # warn if position hasn't changed by more than the epsilon for this long