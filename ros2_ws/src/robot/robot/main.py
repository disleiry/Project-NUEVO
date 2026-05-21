"""
test_burger_pickup.py — Burger Pickup Isolated Test
====================================================
Runs ONLY Section 1 of the full mission so you can test and tune the
pickup sequence independently, without the ramp / obstacle / drop-off code
getting in the way.
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LEDMode,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    Limit,
    Motor,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    ServoChannel,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot


# ===========================================================================
# LIFT MOTOR — paste values from lift_calibration.py
# ===========================================================================

LIFT_MOTOR         = Motor.DC_M3
LIFT_CARRY_TICKS   = -11500   # TODO: PUT YOUR CALIBRATED VALUE HERE
LIFT_PICKUP_TICKS  = -7500    # UPDATED: Your calibrated stable negative target
LIFT_DROPOFF_TICKS = -7500    # UPDATED: Kept matching pickup height
LIFT_DOWN_TICKS    = 0       
LIFT_MAX_VEL       = 800     
LIFT_TOLERANCE     = 30      
LIFT_JOG_STEP      = 2000     # OPTIMIZED: Bigger steps for responsive jogging
LIFT_TIMEOUT_S     = 10.0    # 10-second timeout to allow full travel


# ===========================================================================
# CLAW SERVO
# ===========================================================================

CLAW_SERVO      = ServoChannel.CH_1
CLAW_OPEN_DEG   = 30.0    # TODO: calibrate
CLAW_CLOSE_DEG  = 80.0    # TODO: calibrate


# ===========================================================================
# ULTRASONIC SENSOR ON CLAW
# ===========================================================================

CLAW_ULTRASONIC_LIM = Limit.LIM_2
CLAW_GRAB_CONFIRMED = True   # polarity: True = object detected


# ===========================================================================
# DRIVE BASE
# ===========================================================================

DRIVE_VELOCITY     = 100.0   
APPROACH_VELOCITY  = 60.0    
TURN_TOLERANCE_DEG = 3.0     
POS_TOLERANCE_MM   = 20.0    


# ===========================================================================
# BURGER PICKUP PARAMETERS — match these to main_full_mission.py
# ===========================================================================

DIST_TO_INGREDIENT_AREA = 610.0    # TODO: measure (mm)
APPROACH_SHELF_DIST = 75.0        # TODO: measure (mm)

INGREDIENT_SLOTS = {
    "bun_bottom": 152.0,    
    "meat":       308.0,  
    "bun_top":    460.0,  
}

INGREDIENT_ORDER = ["bun_bottom", "meat", "bun_top"]
ASSEMBLY_SLOT = "bun_bottom"
MAX_PICK_ATTEMPTS = 3
_ESTOP_IMMUNE = frozenset({"INIT", "INIT_JOG", "WAIT_GREEN"})


# ===========================================================================
# LED HELPERS
# ===========================================================================

def led_idle(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 0)
    robot.set_led(LED.BLUE, 0)

def led_moving(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)
    robot.set_led(LED.RED, 0)
    robot.set_led(LED.BLUE, 0)

def led_hold(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 0)
    robot.set_led(LED.BLUE, 200, mode=LEDMode.BLINK, period_ms=800)

def led_error(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 200)
    robot.set_led(LED.BLUE, 0)

def led_done(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200, mode=LEDMode.BLINK, period_ms=400)
    robot.set_led(LED.RED, 0)
    robot.set_led(LED.BLUE, 0)


# ===========================================================================
# STARTUP
# ===========================================================================

def start_robot(robot: Robot) -> None:
    state = robot.get_state()
    if state in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

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
    robot.enable_vision()   

def reset_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed; continuing")
        robot.wait_for_pose_update(timeout=0.5)


# ===========================================================================
# LIFT HELPERS
# ===========================================================================

def get_lift_ticks(robot: Robot) -> int:
    dc = robot.get_dc_state()
    if dc is None:
        return 0
    return int(dc.motors[LIFT_MOTOR - 1].position)

def lift_return_to_zero(robot: Robot) -> None:
    current = get_lift_ticks(robot)
    print(f"\n[LIFT] Returning to origin — current: {current} ticks")
    if abs(current) <= LIFT_TOLERANCE:
        print("[LIFT] Already at origin.")
    else:
        # Utilize the non-blocking system to cleanly step down to zero during exit
        travel_s = abs(current) / LIFT_MAX_VEL
        timeout  = max(15.0, travel_s * 1.5)
        robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
        
        # We temporarily step directly via standard loop logic for cleanup procedures
        ok = robot.set_motor_position(
            LIFT_MOTOR, 0,
            max_vel_ticks=LIFT_MAX_VEL,
            tolerance_ticks=LIFT_TOLERANCE,
            blocking=True, timeout=timeout,
        )
        if ok:
            print("[LIFT] Reached origin. Safe to power off.")
        else:
            print("[warn] LIFT — did not confirm origin.")
    robot.disable_motor(LIFT_MOTOR)


# ===========================================================================
# CLAW HELPERS
# ===========================================================================

def claw_open(robot: Robot) -> None:
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_OPEN_DEG)
    time.sleep(0.5)

def claw_has_object(robot: Robot) -> bool:
    return robot.get_limit(CLAW_ULTRASONIC_LIM) == CLAW_GRAB_CONFIRMED


# ===========================================================================
# SHELF NAVIGATION HELPERS
# ===========================================================================

def turn_to_face_shelf(robot: Robot) -> None:
    print("[NAV] Turn +90° to face shelf")
    robot.turn_by(delta_deg=90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

def turn_away_from_shelf(robot: Robot) -> None:
    print("[NAV] Turn −90° to resume heading")
    robot.turn_by(delta_deg=-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

def approach_shelf(robot: Robot) -> None:
    print(f"[NAV] Approach shelf {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_forward(distance=APPROACH_SHELF_DIST, velocity=APPROACH_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)

def retreat_from_shelf(robot: Robot) -> None:
    print(f"[NAV] Retreat from shelf {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_forward(distance=-APPROACH_SHELF_DIST, velocity=APPROACH_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)

def drive_to_slot(robot: Robot, from_slot: str | None, to_slot: str) -> None:
    from_dist = INGREDIENT_SLOTS.get(from_slot, 0.0) if from_slot else 0.0
    to_dist   = INGREDIENT_SLOTS[to_slot]
    delta     = to_dist - from_dist
    if abs(delta) < 1.0:
        return
    direction = "fwd" if delta > 0 else "bwd"
    print(f"[NAV] Drive {direction} {abs(delta):.0f} mm ({from_slot} → {to_slot})")
    robot.move_forward(distance=delta, velocity=DRIVE_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)


# ===========================================================================
# VISION HELPER
# ===========================================================================

def detect_green_light(robot: Robot) -> bool:
    if not robot.is_vision_active(timeout_s=3.0):
        return False
    for det in robot.get_detections("traffic light"):
        if float(det.get("confidence", 0)) < 0.50:
            continue
        color = det.get("attributes", {}).get("color", {}).get("value", "")
        if color == "green":
            return True
    return False


# ===========================================================================
# RUN — PICKUP TEST FSM
# ===========================================================================

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)
    reset_pose(robot)

    state         = "INIT"
    current_slot  = None
    pick_attempts = 0
    
    action_sub_state = "INIT"
    action_timer      = 0.0
    next_fsm_state   = ""

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    # Hardware tracking system internals
    internal_target_ticks = 0
    requested_final_ticks = 0
    last_step_time = 0.0
    step_delay_s = 0.200  # Give motor 200ms to safely travel 2000 ticks

    print()
    print("=" * 56)
    print("  BURGER PICKUP TEST  —  Section 1 only")
    print("=" * 56)

    while True:

        # ==================================================================
        # INIT & WAIT_GREEN
        # ==================================================================
        if state == "INIT":
            robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
            claw_open(robot)
            led_idle(robot)
            print("  LIFT ALIGNMENT — align carriage to Sharpie mark")
            print("  BTN_1: UP | BTN_2: DOWN | BTN_10: Confirm")
            state = "INIT_JOG"

        elif state == "INIT_JOG":
            if robot.was_button_pressed(Button.BTN_1):
                requested_final_ticks = requested_final_ticks - LIFT_JOG_STEP
                robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                print(f"[JOG] Stepping target UP to: {requested_final_ticks}")

            elif robot.was_button_pressed(Button.BTN_2):
                requested_final_ticks = requested_final_ticks + LIFT_JOG_STEP
                robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                print(f"[JOG] Stepping target DOWN to: {requested_final_ticks}")

            elif robot.was_button_pressed(Button.BTN_10):
                robot.reset_motor_position(LIFT_MOTOR)
                internal_target_ticks = 0
                requested_final_ticks = 0
                time.sleep(0.15)
                print("[INIT] Encoder zeroed -> WAIT_GREEN")
                led_moving(robot)
                state = "WAIT_GREEN"

        elif state == "WAIT_GREEN":
            if detect_green_light(robot) or robot.was_button_pressed(Button.BTN_5):
                print("[FSM] Proceeding to BURGER_PICKUP")
                state = "BURGER_PICKUP"

        # ==================================================================
        # NAVIGATION TO INGREDIENTS
        # ==================================================================
        elif state == "BURGER_PICKUP":
            robot.move_forward(distance=DIST_TO_INGREDIENT_AREA, velocity=DRIVE_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)
            robot.stop()
            current_slot  = None
            pick_attempts = 0
            state = "MOVE_TO_MEAT"

        elif state == "MOVE_TO_MEAT":
            target = INGREDIENT_ORDER[0]
            drive_to_slot(robot, current_slot, target)
            current_slot = target
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            
            action_sub_state = "OPEN_CLAW"
            next_fsm_state = "MOVE_TO_BURGER_BUN1"
            state = "DO_PICK"

        elif state == "MOVE_TO_BURGER_BUN1":
            drive_to_slot(robot, current_slot, ASSEMBLY_SLOT)
            current_slot = ASSEMBLY_SLOT
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            
            action_sub_state = "LIFT_DOWN"
            next_fsm_state = "MOVE_TO_BURGER_BUN2"
            state = "DO_PLACE"

        elif state == "MOVE_TO_BURGER_BUN2":
            target = INGREDIENT_ORDER[1]
            drive_to_slot(robot, current_slot, target)
            current_slot = target
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            
            action_sub_state = "OPEN_CLAW"
            next_fsm_state = "MOVE_TO_BURGER_BUN3"
            state = "DO_PICK"

        elif state == "MOVE_TO_BURGER_BUN3":
            drive_to_slot(robot, current_slot, ASSEMBLY_SLOT)
            current_slot = ASSEMBLY_SLOT
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            
            action_sub_state = "LIFT_DOWN"
            next_fsm_state = "PICK_UP_BURGER"
            state = "DO_PLACE"

        elif state == "PICK_UP_BURGER":
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            
            action_sub_state = "OPEN_CLAW"
            next_fsm_state = "HOLD"
            state = "DO_PICK"

        # ==================================================================
        # GENERIC NON-BLOCKING PICK SEQUENCE
        # ==================================================================
        elif state == "DO_PICK":
            if action_sub_state == "OPEN_CLAW":
                robot.enable_servo(CLAW_SERVO)
                robot.set_servo(CLAW_SERVO, CLAW_OPEN_DEG)
                action_timer = time.monotonic()
                action_sub_state = "WAIT_OPEN"
                
            elif action_sub_state == "WAIT_OPEN":
                if time.monotonic() - action_timer >= 0.5:
                    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                    # INTERCEPTED: Change absolute hardware call to tracking variable
                    requested_final_ticks = LIFT_PICKUP_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_DOWN"
                    
            elif action_sub_state == "WAIT_LIFT_DOWN":
                current = get_lift_ticks(robot)
                time_elapsed = time.monotonic() - action_timer
                
                if abs(current - LIFT_PICKUP_TICKS) <= LIFT_TOLERANCE or time_elapsed > LIFT_TIMEOUT_S:
                    if time_elapsed > LIFT_TIMEOUT_S:
                        print(f"[WARN] Lift timed out going down. Reached: {current} ticks (Target: {LIFT_PICKUP_TICKS})")
                    else:
                        print(f"[ARM] Reached pickup height at: {current} ticks")
                        
                    robot.set_servo(CLAW_SERVO, CLAW_CLOSE_DEG)
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_CLOSE"
                    
            elif action_sub_state == "WAIT_CLOSE":
                if time.monotonic() - action_timer >= 0.5:
                    # INTERCEPTED: Change absolute hardware call to tracking variable
                    requested_final_ticks = LIFT_CARRY_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_UP"
                    
            elif action_sub_state == "WAIT_LIFT_UP":
                current = get_lift_ticks(robot)
                time_elapsed = time.monotonic() - action_timer
                
                if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or time_elapsed > LIFT_TIMEOUT_S:
                    if time_elapsed > LIFT_TIMEOUT_S:
                        print(f"[WARN] Lift timed out going up. Reached: {current} ticks")
                    else:
                        print(f"[ARM] Lift stopped carrying at: {current} ticks")
                        
                    grabbed = claw_has_object(robot)
                    pick_attempts += 1
                    
                    if grabbed or pick_attempts >= MAX_PICK_ATTEMPTS:
                        retreat_from_shelf(robot)
                        turn_away_from_shelf(robot)
                        pick_attempts = 0
                        state = next_fsm_state
                    else:
                        print(f"[ARM] Grab failed. Retrying.")
                        action_sub_state = "OPEN_CLAW" 

        # ==================================================================
        # GENERIC NON-BLOCKING PLACE SEQUENCE
        # ==================================================================
        elif state == "DO_PLACE":
            if action_sub_state == "LIFT_DOWN":
                # INTERCEPTED: Change absolute hardware call to tracking variable
                requested_final_ticks = LIFT_DROPOFF_TICKS
                action_timer = time.monotonic()
                action_sub_state = "WAIT_LIFT_DOWN"
                
            elif action_sub_state == "WAIT_LIFT_DOWN":
                current = get_lift_ticks(robot)
                time_elapsed = time.monotonic() - action_timer
                
                if abs(current - LIFT_DROPOFF_TICKS) <= LIFT_TOLERANCE or time_elapsed > LIFT_TIMEOUT_S:
                    print(f"[ARM] Lift stopped at dropoff at: {current} ticks")
                    robot.set_servo(CLAW_SERVO, CLAW_OPEN_DEG)
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_OPEN"
                    
            elif action_sub_state == "WAIT_OPEN":
                if time.monotonic() - action_timer >= 0.5:
                    # INTERCEPTED: Change absolute hardware call to tracking variable
                    requested_final_ticks = LIFT_CARRY_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_UP"
                    
            elif action_sub_state == "WAIT_LIFT_UP":
                current = get_lift_ticks(robot)
                time_elapsed = time.monotonic() - action_timer
                
                if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or time_elapsed > LIFT_TIMEOUT_S:
                    print(f"[ARM] Lift stopped carrying at: {current} ticks")
                    retreat_from_shelf(robot)
                    turn_away_from_shelf(robot)
                    pick_attempts = 0
                    state = next_fsm_state

        # ==================================================================
        # END STATES
        # ==================================================================
        elif state == "HOLD":
            led_hold(robot)
            if robot.was_button_pressed(Button.BTN_5):
                state = "RETURN_HOME"

        elif state == "RETURN_HOME":
            robot.stop()
            lift_return_to_zero(robot)
            led_done(robot)
            break 

        # ==================================================================
        # GLOBAL EMERGENCY STOP 
        # ==================================================================
        if state not in _ESTOP_IMMUNE:
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                led_error(robot)
                lift_return_to_zero(robot)
                robot.estop()
                break


        # ==================================================================
        # ASYNCHRONOUS MOTOR SPOON-FEEDER (The Integer Rollover Fix)
        # ==================================================================
        if internal_target_ticks != requested_final_ticks:
            now = time.monotonic()
            if now - last_step_time >= step_delay_s:
                
                # Determine direction: UP (- value) or DOWN (+ value)
                if requested_final_ticks < internal_target_ticks:
                    # Moving up
                    remaining = internal_target_ticks - requested_final_ticks
                    if remaining < 2000:
                        internal_target_ticks = requested_final_ticks
                    else:
                        internal_target_ticks -= 2000
                else:
                    # Moving down
                    remaining = requested_final_ticks - internal_target_ticks
                    if remaining < 2000:
                        internal_target_ticks = requested_final_ticks
                    else:
                        internal_target_ticks += 2000
                
                # Command the intermediate safe step directly to the physical motor API
                robot.set_motor_position(
                    LIFT_MOTOR, 
                    internal_target_ticks, 
                    max_vel_ticks=LIFT_MAX_VEL, 
                    tolerance_ticks=LIFT_TOLERANCE, 
                    blocking=False
                )
                last_step_time = now


        # ── tick-rate control ──────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

