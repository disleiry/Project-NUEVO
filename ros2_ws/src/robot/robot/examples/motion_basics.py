"""
motion_basics.py — basic motion commands
=========================================
Demonstrates the fundamental Robot API calls for wheel-based driving,
servo control, and direct DC motor commands.

HOW TO RUN:
    cp examples/motion_basics.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES:
    1. Turns left 90°
    2. Turns right 90° (back to original heading)
    3. Moves forward 500 mm
    4. Moves backward 500 mm (back to start)
    5. Sweeps servo channel 1 between 0° and 90°
    6. Runs motor 3 at 200 mm/s for 1 second (if your robot has a third motor)
    7. Stops cleanly

WHAT THIS TEACHES:
    1. High-level differential-drive commands (move_forward, turn_by)
    2. Servo enable and angle control
    3. Direct DC motor velocity for non-drive motors
    4. Odometry parameter configuration
"""

from __future__ import annotations

import time

from robot.hardware_map import DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit


# ---------------------------------------------------------------------------
# Configuration — edit these to match your robot
# ---------------------------------------------------------------------------

POSITION_UNIT    = Unit.MM
WHEEL_DIAMETER   = 74.0   # mm
WHEEL_BASE       = 333.0  # mm
INITIAL_THETA_DEG = 90.0  # degrees; 90 = pointing in the +Y direction

LEFT_WHEEL_MOTOR          = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED   = False
RIGHT_WHEEL_MOTOR         = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED  = True

# Motion parameters
FORWARD_DISTANCE_MM = 500.0   # how far to move forward/backward
DRIVE_VELOCITY_MM_S = 200.0   # linear drive speed
DRIVE_TOLERANCE_MM  =  20.0   # position goal tolerance

# Servo parameters (channel 1–16)
SERVO_CHANNEL = 1

# Extra DC motor (set to None to skip this demo step)
EXTRA_MOTOR_ID = Motor.DC_M3   # e.g. a gripper or conveyor motor
EXTRA_MOTOR_VELOCITY_MM_S = 200.0  # mm/s


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


def start_robot(robot: Robot) -> None:
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)


# ---------------------------------------------------------------------------
# run() — entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    # ── 1. Turn left 90° (CCW) ───────────────────────────────────────────────
    print("Turning left 90°...")
    robot.turn_by(90.0)
    time.sleep(0.5)

    # ── 2. Turn right 90° (CW) — back to original heading ───────────────────
    print("Turning right 90°...")
    robot.turn_by(-90.0)
    time.sleep(0.5)

    # ── 3. Move forward ──────────────────────────────────────────────────────
    print(f"Moving forward {FORWARD_DISTANCE_MM:.0f} mm...")
    robot.move_forward(FORWARD_DISTANCE_MM, DRIVE_VELOCITY_MM_S, DRIVE_TOLERANCE_MM)
    time.sleep(0.5)

    # ── 4. Move backward ─────────────────────────────────────────────────────
    print(f"Moving backward {FORWARD_DISTANCE_MM:.0f} mm...")
    robot.move_backward(FORWARD_DISTANCE_MM, DRIVE_VELOCITY_MM_S, DRIVE_TOLERANCE_MM)
    time.sleep(0.5)

    # ── 5. Servo demo ─────────────────────────────────────────────────────────
    print(f"Sweeping servo channel {SERVO_CHANNEL}...")
    robot.enable_servo(SERVO_CHANNEL)
    for angle_deg in [0.0, 45.0, 90.0, 45.0, 0.0]:
        robot.set_servo(SERVO_CHANNEL, angle_deg)
        print(f"  servo → {angle_deg:.0f}°")
        time.sleep(0.6)
    robot.disable_servo(SERVO_CHANNEL)

    # ── 6. Direct DC motor velocity ──────────────────────────────────────────
    # set_motor_velocity sends a raw velocity command directly to one motor.
    # Use this for non-drive motors (e.g. a conveyor or arm joint).
    # For the drive wheels, prefer move_forward / turn_by / set_velocity instead.
    if EXTRA_MOTOR_ID is not None:
        print(f"Running motor {EXTRA_MOTOR_ID} at {EXTRA_MOTOR_VELOCITY_MM_S:.0f} mm/s for 1 s...")
        robot.set_motor_velocity(int(EXTRA_MOTOR_ID), EXTRA_MOTOR_VELOCITY_MM_S)
        time.sleep(1.0)
        robot.set_motor_velocity(int(EXTRA_MOTOR_ID), 0.0)

    print("Done. Shutting down.")
    robot.shutdown()
