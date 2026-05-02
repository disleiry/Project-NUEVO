"""
pure_pursuit.py — pure-pursuit path following with optional lidar + GPS
========================================================================
Drives the robot along a rectangular waypoint path using the pure-pursuit
steering law.  Lidar and GPS sensor fusion are off by default; flip the
two flags below to enable them.

HOW TO RUN:
    cp examples/pure_pursuit.py main.py
    ros2 run robot robot

SENSOR TOGGLES — change these before running:
    ENABLE_LIDAR = False  →  True   requires the lidar node to be running
    ENABLE_GPS   = False  →  True   requires the camera node + ArUco tag TAG_ID
                                    in view before the robot starts moving

WHAT THIS TEACHES:
    1. Pure-pursuit path following with purepursuit_follow_path()
    2. How to turn on lidar and GPS sensor fusion with a single flag
    3. How to read odometry pose vs fused pose (when GPS is active)
    4. Non-blocking motion handles for printing status while moving
"""

from __future__ import annotations

import time

from robot.hardware_map import DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit


# ---------------------------------------------------------------------------
# Sensor toggles — flip to True to activate the corresponding sensor
# ---------------------------------------------------------------------------

ENABLE_LIDAR = False   # set True if the lidar node is running
ENABLE_GPS   = False   # set True if the camera node + ArUco tag TAG_ID is visible

TAG_ID = 11            # ArUco tag ID tracked for GPS position fusion


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

POSITION_UNIT    = Unit.MM
WHEEL_DIAMETER   = 74.0   # mm
WHEEL_BASE       = 333.0  # mm
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR          = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED   = False
RIGHT_WHEEL_MOTOR         = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED  = True

# Path — list of (x_mm, y_mm) waypoints in the odometry frame.
# The robot starts at (0, 0) after reset_odometry(), facing INITIAL_THETA_DEG.
WAYPOINTS_MM = [
    (   0.0,  500.0),   # straight ahead 500 mm
    ( 500.0,  500.0),   # right 500 mm
    ( 500.0,    0.0),   # back to start row
    (   0.0,    0.0),   # back to start
]

VELOCITY_MM_S       = 200.0   # maximum forward speed
LOOKAHEAD_MM        = 200.0   # pure-pursuit lookahead distance
TOLERANCE_MM        =  30.0   # waypoint goal tolerance (final goal)
MAX_ANGULAR_RAD_S   =   1.0   # angular-rate clamp (rad/s)

STATUS_PRINT_INTERVAL_S = 0.5  # how often to print pose during motion


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

    # Sensor subscriptions are opt-in.
    # Call enable_lidar() / enable_gps() only when the corresponding nodes are running.
    if ENABLE_LIDAR:
        robot.enable_lidar()
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID} on /tag_detections")


def start_robot(robot: Robot) -> None:
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)


def print_status(robot: Robot) -> None:
    x, y, theta = robot.get_pose()
    if ENABLE_GPS and robot.is_gps_active():
        fx, fy, ftheta = robot.get_fused_pose()
        print(
            f"  odom=({x:6.0f}, {y:6.0f}) mm  θ={theta:5.1f}°  |  "
            f"fused=({fx:6.0f}, {fy:6.0f}) mm  θ={ftheta:5.1f}°"
        )
    else:
        print(f"  pose=({x:6.0f}, {y:6.0f}) mm  θ={theta:5.1f}°")


# ---------------------------------------------------------------------------
# run() — entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    print("Starting pure-pursuit path...")
    print(f"  waypoints : {WAYPOINTS_MM}")
    print(f"  velocity  : {VELOCITY_MM_S} mm/s")
    print(f"  lookahead : {LOOKAHEAD_MM} mm")

    # Non-blocking so we can print pose while moving.
    handle = robot.purepursuit_follow_path(
        WAYPOINTS_MM,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
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
