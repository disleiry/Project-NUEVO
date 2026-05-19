#!/usr/bin/env python3
"""
manual_test_no_limits.py — Pure Manual Lift Movement Tester (No Limits)
========================================================================
A tool to test raw encoder movement and steps without any boundary constraints.

CONTROLS:
  BTN_1  — Toggle step size (500 -> 100 -> 50 -> 500)
  BTN_2  — Move lift UP by the current step size
  BTN_3  — Move lift DOWN by the current step size
  Ctrl+C — Stop the script and disable the motor safely
"""

import time
from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    Motor,
    POSITION_UNIT,
)
from robot.robot import FirmwareState, Robot

# --- CONFIGURATION ---
LIFT_MOTOR = Motor.DC_M3
JOG_VELOCITY_TICKS = 1000
POSITION_TOLERANCE = 10

def get_lift_ticks(robot: Robot) -> int:
    """Return the current inverted encoder position of the lift motor."""
    dc = robot.get_dc_state()
    if dc is None:
        return 0
    return -int(dc.motors[LIFT_MOTOR - 1].position)

def move_lift_to(robot: Robot, ticks: int) -> bool:
    """Move the lift to an absolute encoder position with NO CLAMPING LIMITS."""
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    return robot.set_motor_position(
        LIFT_MOTOR,
        -int(ticks),  # No max or min limits applied here anymore
        max_vel_ticks=JOG_VELOCITY_TICKS,
        tolerance_ticks=POSITION_TOLERANCE,
        blocking=True,
        timeout=5.0,
    )

def run(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    
    # Initialize firmware state
    current_state = robot.get_state()
    if current_state in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    
    # Zero out the encoder wherever the lift is physically sitting right now
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    robot.reset_motor_position(LIFT_MOTOR)
    time.sleep(0.15)

    # Available step sizes list to cycle through
    STEP_OPTIONS = [500, 100, 50]
    step_index = 0
    current_step = STEP_OPTIONS[step_index]

    print("=" * 60)
    print(" UNLIMITED MANUAL LIFT MOVEMENT TEST RUNNING")
    print("=" * 60)
    print(f" Initial position set to 0 ticks.")
    print(f" Current Step Size: {current_step} ticks")
    print(" Controls:")
    print("   BTN_1 : Toggle step size (500 -> 100 -> 50)")
    print("   BTN_2 : Move UP")
    print("   BTN_3 : Move DOWN")
    print(" WARNING: All software limits are REMOVED. Watch your hard stops!")
    print("=" * 60)

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    try:
        while True:
            current_ticks = get_lift_ticks(robot)

            # --- BTN_1: Toggle step size through the options array ---
            if robot.was_button_pressed(Button.BTN_1):
                step_index = (step_index + 1) % len(STEP_OPTIONS)
                current_step = STEP_OPTIONS[step_index]
                print(f"\n[STEP CHANGED] Mode: {current_step} ticks per press")

            # --- BTN_2: Move UP (No soft limits) ---
            elif robot.was_button_pressed(Button.BTN_2):
                target = current_ticks + current_step
                print(f"[MOVE] UP to target: {target} ticks (Using step: {current_step})")
                move_lift_to(robot, target)
                print(f"[READOUT] Actual position reached: {get_lift_ticks(robot)} ticks")

            # --- BTN_3: Move DOWN (Allows negative values below 0) ---
            elif robot.was_button_pressed(Button.BTN_3):
                target = current_ticks - current_step
                print(f"[MOVE] DOWN to target: {target} ticks (Using step: {current_step})")
                move_lift_to(robot, target)
                print(f"[READOUT] Actual position reached: {get_lift_ticks(robot)} ticks")

            # Timing loop control
            next_tick += period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    except KeyboardInterrupt:
        print("\nStopping script cleanly...")
    finally:
        robot.disable_motor(LIFT_MOTOR)
        print("Motor safely deactivated.")
