import time
from robot.robot import FirmwareState, Robot
from robot.hardware_map import (
    Button, DCMotorMode, Motor, ServoChannel, LED,
    POSITION_UNIT, WHEEL_DIAMETER, WHEEL_BASE, INITIAL_THETA_DEG,
    LEFT_WHEEL_MOTOR, LEFT_WHEEL_DIR_INVERTED, 
    RIGHT_WHEEL_MOTOR, RIGHT_WHEEL_DIR_INVERTED
)

# --- CONSTANTS ---
LIFT_MOTOR = Motor.DC_M3
LIFT_CARRY_TICKS = -14000
LIFT_PICKUP_TICKS = -9000
LIFT_ITEM_THICKNESS_TICKS = -1800
LIFT_MAX_VEL = 1800
LIFT_TOLERANCE = 30
LIFT_JOG_STEP = 100

CLAW_SERVO = ServoChannel.CH_13
CLAW_OPEN_DEG = 60.0
CLAW_CLOSE_MEAT_DEG = 150.0
CLAW_CLOSE_BUN_DEG = 140.0

DRIVE_VELOCITY = 100.0
APPROACH_VELOCITY = 60.0
POS_TOLERANCE_MM = 20.0

TURN_TO_SHELF_DEG = 79
TURN_FROM_SHELF_DEG = -79
TURN_TOLERANCE_DEG = 2.0

DIST_TO_INGREDIENT_AREA = 980.0
APPROACH_SHELF_DIST = 30.0

INGREDIENT_SLOTS = {
    "bun_bottom": 207.0,
    "meat": 350.0,
    "bun_top": 485.0,
}

# --- MINIMAL SETUP TO PREVENT FREEZING ---
def configure_and_start_robot(robot: Robot):
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER, wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR, left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR, right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.enable_vision()
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)

# --- HELPER FUNCTIONS ---
def claw_close(robot: Robot, angle: float):
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, angle)
    time.sleep(0.5)

def claw_open(robot: Robot):
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_OPEN_DEG)
    time.sleep(0.5)

def get_lift_ticks(robot: Robot) -> int:
    dc = robot.get_dc_state()
    if dc is None:
        return 0
    return int(dc.motors[LIFT_MOTOR - 1].position)

def move_lift(robot: Robot, target_ticks: int):
    """Safely ramps the motor target to prevent firmware leash errors."""
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    current_ticks = float(get_lift_ticks(robot))
    
    step_rate_hz = 50.0
    delay_s = 1.0 / step_rate_hz
    ticks_per_step = LIFT_MAX_VEL / step_rate_hz
    
    internal_target = current_ticks
    
    # Spoon-feed intermediate positions to avoid math overflows in firmware
    while abs(internal_target - target_ticks) > LIFT_TOLERANCE:
        if target_ticks > internal_target:
            internal_target = min(float(target_ticks), internal_target + ticks_per_step)
        else:
            internal_target = max(float(target_ticks), internal_target - ticks_per_step)
            
        robot.set_motor_position(
            LIFT_MOTOR, int(internal_target), 
            max_vel_ticks=LIFT_MAX_VEL, 
            tolerance_ticks=LIFT_TOLERANCE, 
            blocking=False
        )
        time.sleep(delay_s)
        
    # Final blocking wait to ensure it completely settles at the final destination
    robot.set_motor_position(
        LIFT_MOTOR, int(target_ticks), 
        max_vel_ticks=LIFT_MAX_VEL, 
        tolerance_ticks=LIFT_TOLERANCE, 
        blocking=True, timeout=2.0
    )

def drive_to_slot(robot: Robot, current_pos: float, target_slot: str) -> float:
    target_pos = INGREDIENT_SLOTS[target_slot]
    delta = target_pos - current_pos
    if delta > 0:
        robot.move_forward(delta, DRIVE_VELOCITY, POS_TOLERANCE_MM, blocking=True)
    elif delta < 0:
        robot.move_backward(abs(delta), DRIVE_VELOCITY, POS_TOLERANCE_MM, blocking=True)
    return target_pos

def led_moving(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)

# --- MAIN LOOP ---
def run(robot: Robot) -> None:
    print("[INIT] Configuring robot to clear E-Stop...")
    configure_and_start_robot(robot)
    
    state = "INIT"
    current_x = 0.0
    jog_ticks = 0

    while True:
        if state == "INIT":
            robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
            claw_close(robot, CLAW_CLOSE_BUN_DEG)
            print("=== LIFT ALIGNMENT ===")
            print(" BTN_1: UP | BTN_2: DOWN | BTN_10: Confirm")
            state = "INIT_JOG"

        elif state == "INIT_JOG":
            if robot.was_button_pressed(Button.BTN_1): 
                jog_ticks += LIFT_JOG_STEP
                move_lift(robot, jog_ticks)
            elif robot.was_button_pressed(Button.BTN_2):
                jog_ticks -= LIFT_JOG_STEP
                move_lift(robot, jog_ticks)
            elif robot.was_button_pressed(Button.BTN_10):
                robot.reset_motor_position(LIFT_MOTOR)
                time.sleep(0.15)
                print("[INIT] Encoder zeroed -> WAIT_GREEN")
                led_moving(robot)
                state = "WAIT_GREEN"
            time.sleep(0.05)

        elif state == "WAIT_GREEN":
            # Start sequence on button press
            if robot.was_button_pressed(Button.BTN_5): 
                print("[FSM] Proceeding to pickup.")
                state = "BURGER_PICKUP"
            time.sleep(0.1)

        elif state == "BURGER_PICKUP":
            # 1. PREP INITIAL MOVE
            claw_open(robot)
            move_lift(robot, LIFT_CARRY_TICKS)
            
            # 2. NAVIGATE TO INGREDIENT AREA
            robot.move_forward(DIST_TO_INGREDIENT_AREA, DRIVE_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            
            # 3. FETCH MEAT
            current_x = drive_to_slot(robot, current_x, "meat")
            robot.turn_by(TURN_TO_SHELF_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            robot.move_forward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            
            move_lift(robot, LIFT_PICKUP_TICKS)
            claw_close(robot, CLAW_CLOSE_MEAT_DEG)
            move_lift(robot, LIFT_CARRY_TICKS)
            
            robot.move_backward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            robot.turn_by(TURN_FROM_SHELF_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            
            # 4. PLACE MEAT
            current_x = drive_to_slot(robot, current_x, "bun_bottom")
            robot.turn_by(TURN_TO_SHELF_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            robot.move_forward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            
            move_lift(robot, LIFT_PICKUP_TICKS + LIFT_ITEM_THICKNESS_TICKS)
            claw_open(robot)
            move_lift(robot, LIFT_CARRY_TICKS)
            
            robot.move_backward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            robot.turn_by(TURN_FROM_SHELF_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

            # 5. FETCH TOP BUN
            current_x = drive_to_slot(robot, current_x, "bun_top")
            robot.turn_by(TURN_TO_SHELF_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            robot.move_forward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            
            move_lift(robot, LIFT_PICKUP_TICKS)
            claw_close(robot, CLAW_CLOSE_BUN_DEG)
            move_lift(robot, LIFT_CARRY_TICKS)
            
            robot.move_backward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            robot.turn_by(TURN_FROM_SHELF_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

            # 6. PLACE TOP BUN & GRAB FULL STACK
            current_x = drive_to_slot(robot, current_x, "bun_bottom")
            robot.turn_by(TURN_TO_SHELF_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            robot.move_forward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            
            move_lift(robot, LIFT_PICKUP_TICKS + (2 * LIFT_ITEM_THICKNESS_TICKS)) 
            claw_open(robot)
            
            move_lift(robot, LIFT_PICKUP_TICKS)
            claw_close(robot, CLAW_CLOSE_BUN_DEG)
            move_lift(robot, LIFT_CARRY_TICKS)
            
            robot.move_backward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            robot.turn_by(TURN_FROM_SHELF_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

            print("[FSM] Burger pickup complete.")
            state = "DONE"

        elif state == "DONE":
            time.sleep(1.0)
