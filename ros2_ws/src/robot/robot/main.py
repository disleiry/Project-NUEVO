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
# LIFT MOTOR
# ===========================================================================

LIFT_MOTOR         = Motor.DC_M3
LIFT_CARRY_TICKS   = -15000    # UPDATED: Drop off / travel height
LIFT_PICKUP_TICKS  = -9500     # UPDATED: Base pickup height
LIFT_DOWN_TICKS    = 0        
LIFT_MAX_VEL       = 1200     
LIFT_TOLERANCE     = 30      
LIFT_JOG_STEP      = 1500     
LIFT_TIMEOUT_S     = 20.0    

# Offset for each stacked burger piece (~1 inch thick)
# UPDATED: Since positive is now UP, this is a positive value.
LIFT_ITEM_THICKNESS_TICKS = -1800  


# ===========================================================================
# CLAW SERVO
# ===========================================================================

CLAW_SERVO      = ServoChannel.CH_13
CLAW_OPEN_DEG   = 10.0   # UPDATED
CLAW_CLOSE_DEG  = 115.0  # UPDATED


# ===========================================================================
# ULTRASONIC SENSOR ON CLAW
# ===========================================================================

CLAW_ULTRASONIC_LIM = Limit.LIM_2
CLAW_GRAB_CONFIRMED = True   


# ===========================================================================
# DRIVE BASE & NAVIGATION
# ===========================================================================

DRIVE_VELOCITY      = 100.0   
APPROACH_VELOCITY   = 60.0    
POS_TOLERANCE_MM    = 20.0    

# Tunable turn angles (adjust if shelf isn't exactly 90 degrees)
TURN_TO_SHELF_DEG   = 80
TURN_FROM_SHELF_DEG = -80
TURN_TOLERANCE_DEG  = 2.0     

TURN_VELOCITY = 30.0
SERVO_DEG_PER_STEP = 0.8


# ===========================================================================
# BURGER PICKUP PARAMETERS
# ===========================================================================

DIST_TO_INGREDIENT_AREA = 640.0    
APPROACH_SHELF_DIST = 55.0        

INGREDIENT_SLOTS = {
    "bun_bottom": 182.0,    
    "meat":       340.0,  
    "bun_top":    480.0,  
}

# The bottom bun is our assembly base. We only fetch meat and top bun.
FETCH_ORDER   = ["meat", "bun_top"]
ASSEMBLY_SLOT = "bun_bottom"

MAX_PICK_ATTEMPTS = 1  
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
        travel_s = abs(current) / LIFT_MAX_VEL
        timeout  = max(15.0, travel_s * 1.5)
        robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
        
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

def claw_close(robot: Robot) -> None:
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_CLOSE_DEG)
    time.sleep(0.5)

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
    print(f"[NAV] Turn {TURN_TO_SHELF_DEG}° to face shelf")
    robot.turn_by(
        delta_deg=TURN_TO_SHELF_DEG, 
        blocking=True, 
        tolerance_deg=TURN_TOLERANCE_DEG
    )

def turn_away_from_shelf(robot: Robot) -> None:
    print(f"[NAV] Turn {TURN_FROM_SHELF_DEG}° to resume heading")
    robot.turn_by(
        delta_deg=TURN_FROM_SHELF_DEG, 
        blocking=True, 
        tolerance_deg=TURN_TOLERANCE_DEG
    )

def approach_shelf(robot: Robot) -> None:
    print(f"[NAV] Approach shelf {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_forward(distance=APPROACH_SHELF_DIST, velocity=APPROACH_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)

def retreat_from_shelf(robot: Robot) -> None:
    print(f"[NAV] Retreat from shelf {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_backward(distance=APPROACH_SHELF_DIST, velocity=APPROACH_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)

def drive_to_slot(robot: Robot, from_slot: str | None, to_slot: str) -> None:
    from_dist = INGREDIENT_SLOTS.get(from_slot, 0.0) if from_slot else 0.0
    to_dist   = INGREDIENT_SLOTS[to_slot]
    delta     = to_dist - from_dist
    
    if abs(delta) < 1.0:
        return
        
    if delta > 0:
        print(f"[NAV] Drive fwd {abs(delta):.0f} mm ({from_slot} → {to_slot})")
        robot.move_forward(distance=abs(delta), velocity=DRIVE_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)
    else:
        print(f"[NAV] Drive bwd {abs(delta):.0f} mm ({from_slot} → {to_slot})")
        robot.move_backward(distance=abs(delta), velocity=DRIVE_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)


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
    active_drop_ticks = 0  
    
    action_sub_state = "INIT"
    action_timer      = 0.0
    next_fsm_state   = ""
    
    manual_skip_triggered = False
    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    internal_target_ticks = 0
    requested_final_ticks = 0
    last_step_time = 0.0
    step_delay_s = 0.150  
    
    requested_claw_deg = CLAW_CLOSE_DEG  
    current_claw_deg = CLAW_CLOSE_DEG

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
            claw_close(robot)  
            led_idle(robot)
            print("  LIFT ALIGNMENT — align carriage to Sharpie mark")
            print("  BTN_1: UP | BTN_2: DOWN | BTN_10: Confirm")
            state = "INIT_JOG"

        elif state == "INIT_JOG":
            if robot.was_button_pressed(Button.BTN_1):
                # Using plus here since positive is UP
                requested_final_ticks = requested_final_ticks + LIFT_JOG_STEP
                robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                print(f"[JOG] Stepping target UP to: {requested_final_ticks}")

            elif robot.was_button_pressed(Button.BTN_2):
                requested_final_ticks = requested_final_ticks - LIFT_JOG_STEP
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
                print("[FSM] Proceeding to PREP_INITIAL_MOVE")
                state = "PREP_INITIAL_MOVE"

        # ==================================================================
        # PREP MOVE
        # ==================================================================
        elif state == "PREP_INITIAL_MOVE":
            print("[ARM] Prepping arm for travel...")
            robot.enable_servo(CLAW_SERVO)
            requested_claw_deg = CLAW_OPEN_DEG  # Smooth transition
            requested_final_ticks = LIFT_CARRY_TICKS
            action_timer = time.monotonic()
            state = "WAIT_PREP_INITIAL"

        elif state == "WAIT_PREP_INITIAL":
            current = get_lift_ticks(robot)
            if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or (time.monotonic() - action_timer > LIFT_TIMEOUT_S):
                print("[ARM] Arm prepped at carry height. Moving out.")
                state = "BURGER_PICKUP"

        # ==================================================================
        # NAVIGATION TO INGREDIENTS
        # ==================================================================
        elif state == "BURGER_PICKUP":
            robot.move_forward(distance=DIST_TO_INGREDIENT_AREA, velocity=DRIVE_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)
            robot.stop()
            current_slot  = None
            pick_attempts = 0
            state = "FETCH_MEAT"

        elif state == "FETCH_MEAT":
            target = FETCH_ORDER[0]  # "meat"
            drive_to_slot(robot, current_slot, target)
            current_slot = target
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            
            action_sub_state = "OPEN_CLAW"
            next_fsm_state = "PLACE_MEAT"
            state = "DO_PICK"

        elif state == "PLACE_MEAT":
            drive_to_slot(robot, current_slot, ASSEMBLY_SLOT)
            current_slot = ASSEMBLY_SLOT
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            
            # Placing meat onto base bun (1 thickness up)
            active_drop_ticks = LIFT_PICKUP_TICKS + (1 * LIFT_ITEM_THICKNESS_TICKS)
            action_sub_state = "LIFT_DOWN"
            next_fsm_state = "FETCH_TOP_BUN"
            state = "DO_PLACE"

        elif state == "FETCH_TOP_BUN":
            target = FETCH_ORDER[1]  # "bun_top"
            drive_to_slot(robot, current_slot, target)
            current_slot = target
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            
            action_sub_state = "OPEN_CLAW"
            next_fsm_state = "PLACE_AND_GRAB_STACK"
            state = "DO_PICK"

        elif state == "PLACE_AND_GRAB_STACK":
            drive_to_slot(robot, current_slot, ASSEMBLY_SLOT)
            current_slot = ASSEMBLY_SLOT
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            
            # Placing top bun onto meat + base bun (2 thicknesses up)
            active_drop_ticks = LIFT_PICKUP_TICKS + (2 * LIFT_ITEM_THICKNESS_TICKS)
            action_sub_state = "LIFT_DOWN_TO_STACK"
            next_fsm_state = "HOLD"
            state = "DO_PLACE_AND_GRAB"

        # ==================================================================
        # GENERIC NON-BLOCKING PICK SEQUENCE
        # ==================================================================
        elif state == "DO_PICK":
            if robot.was_button_pressed(Button.BTN_3):
                manual_skip_triggered = True

            if action_sub_state == "OPEN_CLAW":
                robot.enable_servo(CLAW_SERVO)
                requested_claw_deg = CLAW_OPEN_DEG  # Smooth transition
                action_timer = time.monotonic()
                action_sub_state = "WAIT_OPEN"
                
            elif action_sub_state == "WAIT_OPEN":
                if time.monotonic() - action_timer >= 0.5:
                    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                    requested_final_ticks = LIFT_PICKUP_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_DOWN"
                    
            elif action_sub_state == "WAIT_LIFT_DOWN":
                current = get_lift_ticks(robot)
                time_elapsed = time.monotonic() - action_timer
                
                if abs(current - LIFT_PICKUP_TICKS) <= LIFT_TOLERANCE or time_elapsed > LIFT_TIMEOUT_S:
                    print(f"[ARM] Reached pickup height at: {current} ticks")
                    print(f"[ARM] Closing claw to {CLAW_CLOSE_DEG} degrees...")
                    requested_claw_deg = CLAW_CLOSE_DEG  # Smooth transition
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_CLOSE"
                    
            elif action_sub_state == "WAIT_CLOSE":
                if time.monotonic() - action_timer >= 1.5:
                    print("[ARM] Claw closed. Lifting to carry height...")
                    requested_final_ticks = LIFT_CARRY_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_UP"
                    
            elif action_sub_state == "WAIT_LIFT_UP":
                current = get_lift_ticks(robot)
                time_elapsed = time.monotonic() - action_timer
                
                if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or time_elapsed > LIFT_TIMEOUT_S:
                    grabbed = claw_has_object(robot) or manual_skip_triggered
                    pick_attempts += 1
                    
                    if grabbed:
                        retreat_from_shelf(robot)
                        turn_away_from_shelf(robot)
                        pick_attempts = 0
                        manual_skip_triggered = False  
                        state = next_fsm_state
                    elif pick_attempts >= MAX_PICK_ATTEMPTS:
                        retreat_from_shelf(robot)
                        turn_away_from_shelf(robot)
                        pick_attempts = 0
                        manual_skip_triggered = False
                        state = next_fsm_state
                    else:
                        action_sub_state = "OPEN_CLAW" 

        # ==================================================================
        # GENERIC NON-BLOCKING PLACE SEQUENCE (Leaves Claw Open!)
        # ==================================================================
        elif state == "DO_PLACE":
            if action_sub_state == "LIFT_DOWN":
                requested_final_ticks = active_drop_ticks
                action_timer = time.monotonic()
                action_sub_state = "WAIT_LIFT_DOWN"
                
            elif action_sub_state == "WAIT_LIFT_DOWN":
                current = get_lift_ticks(robot)
                time_elapsed = time.monotonic() - action_timer
                
                if abs(current - active_drop_ticks) <= LIFT_TOLERANCE or time_elapsed > LIFT_TIMEOUT_S:
                    print(f"[ARM] Lift stopped at dynamic dropoff: {current} ticks")
                    print(f"[ARM] Opening claw to {CLAW_OPEN_DEG} degrees...")
                    requested_claw_deg = CLAW_OPEN_DEG  # Smooth transition
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_OPEN"
                    
            elif action_sub_state == "WAIT_OPEN":
                if time.monotonic() - action_timer >= 0.5:
                    print("[ARM] Claw is open. Lifting safely to carry height FIRST...")
                    requested_final_ticks = LIFT_CARRY_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_UP"
                    
            elif action_sub_state == "WAIT_LIFT_UP":
                current = get_lift_ticks(robot)
                time_elapsed = time.monotonic() - action_timer
                
                if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or time_elapsed > LIFT_TIMEOUT_S:
                    print(f"[ARM] Lift clear of burger at: {current} ticks. Leaving Claw OPEN.")
                    retreat_from_shelf(robot)
                    turn_away_from_shelf(robot)
                    pick_attempts = 0
                    state = next_fsm_state

        # ==================================================================
        # DROP 3RD PIECE AND SCOOP ENTIRE BURGER
        # ==================================================================
        elif state == "DO_PLACE_AND_GRAB":
            if action_sub_state == "LIFT_DOWN_TO_STACK":
                requested_final_ticks = active_drop_ticks
                action_timer = time.monotonic()
                action_sub_state = "WAIT_LIFT_DOWN_STACK"

            elif action_sub_state == "WAIT_LIFT_DOWN_STACK":
                current = get_lift_ticks(robot)
                if abs(current - active_drop_ticks) <= LIFT_TOLERANCE or (time.monotonic() - action_timer > LIFT_TIMEOUT_S):
                    print("[ARM] Reached stack height. Dropping top bun...")
                    requested_claw_deg = CLAW_OPEN_DEG  # Smooth transition
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_OPEN"

            elif action_sub_state == "WAIT_OPEN":
                if time.monotonic() - action_timer >= 0.5:
                    print("[ARM] Dropped bun. Lowering to base to grab entire burger...")
                    requested_final_ticks = LIFT_PICKUP_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_DOWN_BASE"

            elif action_sub_state == "WAIT_LIFT_DOWN_BASE":
                current = get_lift_ticks(robot)
                if abs(current - LIFT_PICKUP_TICKS) <= LIFT_TOLERANCE or (time.monotonic() - action_timer > LIFT_TIMEOUT_S):
                    print("[ARM] At base height. Grabbing entire burger...")
                    requested_claw_deg = CLAW_CLOSE_DEG  # Smooth transition
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_CLOSE"

            elif action_sub_state == "WAIT_CLOSE":
                if time.monotonic() - action_timer >= 1.5:
                    print("[ARM] Grabbed entire burger. Lifting to carry height...")
                    requested_final_ticks = LIFT_CARRY_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_UP"

            elif action_sub_state == "WAIT_LIFT_UP":
                current = get_lift_ticks(robot)
                if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or (time.monotonic() - action_timer > LIFT_TIMEOUT_S):
                    print("[ARM] Secured all pieces at carry height.")
                    retreat_from_shelf(robot)
                    turn_away_from_shelf(robot)
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
        # ASYNCHRONOUS MOTOR SPOON-FEEDER (The Physical Leash Fix)
        # ==================================================================
        current_phys = get_lift_ticks(robot)
        
        if abs(current_phys - requested_final_ticks) > LIFT_TOLERANCE:
            now = time.monotonic()
            
            if now - last_step_time >= step_delay_s:
                if requested_final_ticks < current_phys:
                    internal_target_ticks = max(requested_final_ticks, current_phys - 1500)
                else:
                    internal_target_ticks = min(requested_final_ticks, current_phys + 1500)
                
                robot.set_motor_position(
                    LIFT_MOTOR, 
                    internal_target_ticks, 
                    max_vel_ticks=LIFT_MAX_VEL, 
                    tolerance_ticks=LIFT_TOLERANCE, 
                    blocking=False
                )
                last_step_time = now

        # ==================================================================
        # ASYNCHRONOUS SERVO SPEED GOVERNOR
        # ==================================================================
        if abs(current_claw_deg - requested_claw_deg) > 0.5:
            if requested_claw_deg > current_claw_deg:
                current_claw_deg = min(requested_claw_deg, current_claw_deg + SERVO_DEG_PER_STEP)
            else:
                current_claw_deg = max(requested_claw_deg, current_claw_deg - SERVO_DEG_PER_STEP)
            
            robot.set_servo(CLAW_SERVO, current_claw_deg)

        # ── tick-rate control ──────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
