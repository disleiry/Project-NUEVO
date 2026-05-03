"""
motion_basics.py — FSM-based motion and actuator basics
=======================================================
Teaches the core Robot API calls using the same tick-loop pattern as the
student-facing `main.py`.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/motion_basics.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Press BTN_1 to start. The robot:

  1. Turns left 90°
  2. Turns right 90° (back to the starting heading)
  3. Moves forward 500 mm
  4. Moves backward 500 mm (back to the starting point)
  5. Sweeps servo channel 1 through a short angle sequence
  6. Runs motor 3 for 1 second (if configured)

BTN_2 cancels the active step and returns to IDLE.

WHAT THIS TEACHES
-----------------
1. `set_odometry_parameters()` and `reset_odometry()` setup flow
2. Non-blocking `MotionHandle` polling inside the FSM loop
3. Servo control without blocking the whole robot program
4. Direct single-motor velocity control for non-drive actuators
"""

from __future__ import annotations

import time

from robot.hardware_map import Button, DCMotorMode, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit


# ---------------------------------------------------------------------------
# Configuration — edit these to match your robot
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

TURN_DEGREES = 90.0
FORWARD_DISTANCE_MM = 500.0
DRIVE_VELOCITY_MM_S = 200.0
DRIVE_TOLERANCE_MM = 20.0
TURN_TOLERANCE_DEG = 3.0

SERVO_CHANNEL = 1
SERVO_ANGLES_DEG = [0.0, 45.0, 90.0, 45.0, 0.0]
SERVO_SETTLE_SEC = 0.6

EXTRA_MOTOR_ID = Motor.DC_M3
EXTRA_MOTOR_VELOCITY_MM_S = 200.0
EXTRA_MOTOR_RUN_SEC = 1.0


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


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def cancel_motion(handle) -> None:
    if handle is None:
        return
    handle.cancel()
    handle.wait(timeout=1.0)


def stop_extra_motor(robot: Robot) -> None:
    if EXTRA_MOTOR_ID is None:
        return
    robot.set_motor_velocity(int(EXTRA_MOTOR_ID), 0.0)


def disable_extra_motor(robot: Robot) -> None:
    if EXTRA_MOTOR_ID is None:
        return
    robot.disable_motor(int(EXTRA_MOTOR_ID))


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None
    servo_index = 0
    servo_next_time = 0.0
    extra_motor_stop_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state not in ("INIT", "IDLE") and robot.was_button_pressed(Button.BTN_2):
            cancel_motion(motion_handle)
            motion_handle = None
            stop_extra_motor(robot)
            disable_extra_motor(robot)
            robot.disable_servo(SERVO_CHANNEL)
            robot.stop()
            show_idle_leds(robot)
            print("[FSM] IDLE — cancelled")
            state = "IDLE"

        elif state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start, BTN_2 to cancel active steps")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_running_leds(robot)
                print("[FSM] TURN_LEFT")
                motion_handle = robot.turn_by(
                    delta_deg=TURN_DEGREES,
                    blocking=False,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )
                state = "WAIT_TURN_LEFT"

        elif state == "WAIT_TURN_LEFT":
            if motion_handle is not None and motion_handle.is_finished():
                print("[FSM] TURN_RIGHT")
                motion_handle = robot.turn_by(
                    delta_deg=-TURN_DEGREES,
                    blocking=False,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )
                state = "WAIT_TURN_RIGHT"

        elif state == "WAIT_TURN_RIGHT":
            if motion_handle is not None and motion_handle.is_finished():
                print("[FSM] MOVE_FORWARD")
                motion_handle = robot.move_forward(
                    distance=FORWARD_DISTANCE_MM,
                    velocity=DRIVE_VELOCITY_MM_S,
                    tolerance=DRIVE_TOLERANCE_MM,
                    blocking=False,
                )
                state = "WAIT_MOVE_FORWARD"

        elif state == "WAIT_MOVE_FORWARD":
            if motion_handle is not None and motion_handle.is_finished():
                print("[FSM] MOVE_BACKWARD")
                motion_handle = robot.move_backward(
                    distance=FORWARD_DISTANCE_MM,
                    velocity=DRIVE_VELOCITY_MM_S,
                    tolerance=DRIVE_TOLERANCE_MM,
                    blocking=False,
                )
                state = "WAIT_MOVE_BACKWARD"

        elif state == "WAIT_MOVE_BACKWARD":
            if motion_handle is not None and motion_handle.is_finished():
                print(f"[FSM] SERVO_SWEEP channel={SERVO_CHANNEL}")
                motion_handle = None
                servo_index = 0
                robot.enable_servo(SERVO_CHANNEL)
                robot.set_servo(SERVO_CHANNEL, SERVO_ANGLES_DEG[servo_index])
                servo_next_time = now + SERVO_SETTLE_SEC
                state = "SERVO_SWEEP"

        elif state == "SERVO_SWEEP":
            if now >= servo_next_time:
                servo_index += 1
                if servo_index >= len(SERVO_ANGLES_DEG):
                    robot.disable_servo(SERVO_CHANNEL)
                    if EXTRA_MOTOR_ID is None:
                        robot.stop()
                        show_idle_leds(robot)
                        print("[FSM] DONE — press BTN_1 to run again")
                        state = "DONE"
                    else:
                        print(f"[FSM] EXTRA_MOTOR motor={int(EXTRA_MOTOR_ID)}")
                        robot.enable_motor(int(EXTRA_MOTOR_ID), DCMotorMode.VELOCITY)
                        robot.set_motor_velocity(int(EXTRA_MOTOR_ID), EXTRA_MOTOR_VELOCITY_MM_S)
                        extra_motor_stop_at = now + EXTRA_MOTOR_RUN_SEC
                        state = "WAIT_EXTRA_MOTOR"
                else:
                    robot.set_servo(SERVO_CHANNEL, SERVO_ANGLES_DEG[servo_index])
                    servo_next_time = now + SERVO_SETTLE_SEC

        elif state == "WAIT_EXTRA_MOTOR":
            if now >= extra_motor_stop_at:
                stop_extra_motor(robot)
                disable_extra_motor(robot)
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — press BTN_1 to run again")
                state = "DONE"

        elif state == "DONE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                reset_mission_pose(robot)
                print("[FSM] TURN_LEFT")
                motion_handle = robot.turn_by(
                    delta_deg=TURN_DEGREES,
                    blocking=False,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )
                state = "WAIT_TURN_LEFT"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
