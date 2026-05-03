"""
pure_pursuit.py — FSM-based pure-pursuit path following
=======================================================
Waypoint path following with the same `main.py`-style state loop used by the
older reference example, but updated for the current Robot API.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/pure_pursuit.py main.py
    ros2 run robot robot

BTN_1 starts the path. BTN_2 cancels the active run and returns to IDLE.

SENSOR TOGGLES
--------------
Set these before running if the corresponding nodes are available:

    ENABLE_LIDAR = False  →  True   requires `/scan`
    ENABLE_GPS   = False  →  True   requires `/tag_detections`

WHAT THIS TEACHES
-----------------
1. Pure-pursuit path following inside an FSM loop
2. Optional lidar and GPS enable flow
3. Non-blocking `MotionHandle` polling during motion
4. Periodic status printing while the robot stays responsive to buttons
"""

from __future__ import annotations

import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit
from robot.util import densify_polyline  # noqa: F401 - optional helper for students


# ---------------------------------------------------------------------------
# Sensor toggles
# ---------------------------------------------------------------------------

ENABLE_LIDAR = False
ENABLE_GPS = False
TAG_ID = 11


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

PATH_CONTROL_POINTS = [
    (0.0, 0.0),
    (0.0, 610.0),
    (610.0, 610.0),
    (610.0, 0.0),
    (0.0, 0.0),
]

# Optional: densify long segments for smoother tracking.
# PATH_CONTROL_POINTS = densify_polyline(PATH_CONTROL_POINTS, spacing=50.0)

VELOCITY_MM_S = 150.0
LOOKAHEAD_MM = 120.0
TOLERANCE_MM = 25.0
ADVANCE_RADIUS_MM = 80.0
MAX_ANGULAR_RAD_S = 1.5

STATUS_PRINT_INTERVAL_S = 0.5


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

    if ENABLE_LIDAR:
        robot.enable_lidar()
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


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


def start_path(robot: Robot):
    return robot.purepursuit_follow_path(
        waypoints=PATH_CONTROL_POINTS,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        blocking=False,
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    drive_handle = None
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start path, BTN_2 to cancel")
            print(
                f"[CFG] velocity={VELOCITY_MM_S:.0f} mm/s lookahead={LOOKAHEAD_MM:.0f} mm "
                f"tolerance={TOLERANCE_MM:.0f} mm advance_radius={ADVANCE_RADIUS_MM:.0f} mm"
            )
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_moving_leds(robot)
                print(f"[FSM] MOVING — {len(PATH_CONTROL_POINTS)} waypoints")
                drive_handle = start_path(robot)
                last_status_print_at = now
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — path cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if drive_handle is not None and drive_handle.is_finished():
                    print("[FSM] DONE — path complete")
                    print_status(robot)
                    drive_handle = None
                    robot.stop()
                    show_idle_leds(robot)
                    print("[FSM] IDLE — press BTN_1 to run again")
                    state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
