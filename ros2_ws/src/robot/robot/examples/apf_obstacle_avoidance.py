"""
apf_obstacle_avoidance.py — APF obstacle avoidance with lidar
=============================================================
Uses artificial potential fields (APF) to steer the robot toward a sequence
of waypoints while repelling it from lidar-detected obstacles.

LIDAR IS REQUIRED.  GPS fusion is optional (ENABLE_GPS flag).

HOW TO RUN:
    cp examples/apf_obstacle_avoidance.py main.py
    ros2 run robot robot

    Make sure the lidar node is running:
        ros2 run sensors lidar_node

SENSOR TOGGLES:
    ENABLE_GPS = False  →  True   requires camera node + ArUco tag TAG_ID

HOW APF WORKS:
    Attractive force  — pulls the robot toward the next waypoint.
    Repulsive force   — pushes the robot away from any obstacle closer than
                        REPULSION_RANGE_MM.
    The planner sums both force vectors each tick and converts the result
    into a (linear, angular) velocity command.

TUNING TIPS:
    REPULSION_RANGE_MM   — increase if the robot clips obstacles; decrease to
                           allow it to pass closer.
    REPULSION_GAIN       — increase for more aggressive avoidance; if the
                           robot oscillates or stalls, reduce this value.
    VELOCITY_MM_S        — slower speeds give APF more time to react.

WHAT THIS TEACHES:
    1. APF path following with apf_follow_path()
    2. Lidar is enabled once and obstacles are automatically available
    3. How to read live obstacle count during motion
    4. Optional GPS fusion for improved position accuracy
"""

from __future__ import annotations

import time

from robot.hardware_map import LED, Motor
from robot.robot import FirmwareState, Robot, Unit


# ---------------------------------------------------------------------------
# Sensor toggles
# ---------------------------------------------------------------------------

ENABLE_GPS = False   # set True if camera node + ArUco tag TAG_ID is visible

TAG_ID = 11          # ArUco tag ID for GPS fusion (only used when ENABLE_GPS = True)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

POSITION_UNIT    = Unit.MM
WHEEL_DIAMETER   = 74.0    # mm
WHEEL_BASE       = 333.0   # mm
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR          = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED   = False
RIGHT_WHEEL_MOTOR         = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED  = True

# Path — (x_mm, y_mm) waypoints.  Place obstacles somewhere between them.
WAYPOINTS_MM = [
    (1000.0,    0.0),
    (2000.0,    0.0),
]

VELOCITY_MM_S      = 150.0   # forward speed (slower = more reaction time for APF)
LOOKAHEAD_MM       = 250.0   # lookahead for the attractive component
TOLERANCE_MM       =  50.0   # waypoint goal tolerance
MAX_ANGULAR_RAD_S  =   1.2   # angular-rate clamp

# APF repulsion parameters
REPULSION_RANGE_MM = 400.0   # obstacles within this radius cause repulsion (mm)
REPULSION_GAIN     = 800.0   # repulsive force magnitude — tune for your environment

STATUS_PRINT_INTERVAL_S = 0.5


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

    # Lidar is required for APF obstacle avoidance.
    # After enable_lidar(), each /scan update automatically populates the
    # obstacle list that apf_follow_path() reads on every tick.
    robot.enable_lidar()
    print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")


def start_robot(robot: Robot) -> None:
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)


def print_status(robot: Robot) -> None:
    if ENABLE_GPS and robot.is_gps_active():
        x, y, theta = robot.get_fused_pose()
        label = "fused"
    else:
        x, y, theta = robot.get_pose()
        label = "odom "
    obs_count = len(robot.get_obstacles())
    print(f"  {label}=({x:6.0f}, {y:6.0f}) mm  θ={theta:5.1f}°  obstacles={obs_count}")


# ---------------------------------------------------------------------------
# run() — entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    print("Starting APF navigation — place obstacles in the robot's path.")
    print(f"  waypoints      : {WAYPOINTS_MM}")
    print(f"  velocity       : {VELOCITY_MM_S} mm/s")
    print(f"  repulsion range: {REPULSION_RANGE_MM} mm")
    print(f"  repulsion gain : {REPULSION_GAIN}")

    handle = robot.apf_follow_path(
        WAYPOINTS_MM,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        repulsion_range=REPULSION_RANGE_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        repulsion_gain=REPULSION_GAIN,
        blocking=False,
    )

    last_print = time.monotonic()
    while not handle.is_finished():
        now = time.monotonic()
        if now - last_print >= STATUS_PRINT_INTERVAL_S:
            print_status(robot)
            last_print = now
        time.sleep(0.05)

    print("Path complete.")
    print_status(robot)
    robot.shutdown()
