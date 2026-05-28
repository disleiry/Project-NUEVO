"""
main_combined.py - Burger Pickup + Traffic Light Bypass + Pure Pursuit
"""

from __future__ import annotations
import time
from typing import Any

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
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline

# ==============================================================================
# SENSOR / GPS / PURE PURSUIT CONFIG
# ==============================================================================
ENABLE_VISION = True
ENABLE_LIDAR = True
ENABLE_GPS = True

TAG_ID = 25
GPS_POSITION_ALPHA = 0.20
ENABLE_GPS_TANGENT_HEADING = True
GPS_TANGENT_ALPHA = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0

LED_BRIGHTNESS = 255
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_CONFIDENCE = 0.50

PURE_PURSUIT_CONTROL_POINTS = [
    (300.0, 3700.0),
    (620.0, 3700.0),
    (690.0, 3540.0),
    (730.0, 3300.0),
    (750.0, 3000.0),
    (950.0, 600.0),
    (1600.0, 600.0),
]
PURE_PURSUIT_CONTROL_POINTS = densify_polyline(PURE_PURSUIT_CONTROL_POINTS, spacing=100.0)

LAPF_CONTROL_POINTS = [
    (1800.0, 3250.0),
]
LAPF_CONTROL_POINTS = densify_polyline(LAPF_CONTROL_POINTS, spacing=50.0)

PURE_PURSUIT_VELOCITY_MM_S = 150.0
LOOKAHEAD_MM = 225.0
PURE_PURSUIT_TOLERANCE_MM = 25.0
ADVANCE_RADIUS_MM = 75.0
PURE_PURSUIT_MAX_ANGULAR_RAD_S = 1.5

LAPF_VELOCITY_MM_S = 150.0
LAPF_TOLERANCE_MM = 50.0
LAPF_MAX_ANGULAR_RAD_S = 0.6
LEASH_LENGTH_MM = 150.0
REPULSION_RANGE_MM = 300.0
TARGET_SPEED_MM_S = 200.0
REPULSION_GAIN = 350.0
ATTRACTION_GAIN = 1.0
FORCE_EMA_ALPHA = 0.35
INFLATION_MARGIN_MM = 150.0
LEASH_HALF_ANGLE_DEG = 25.0

MISSION_STAGES: list[dict[str, Any]] = [
    {
        "name": "Straight/ramp pure pursuit",
        "type": "pure_pursuit",
        "waypoints": PURE_PURSUIT_CONTROL_POINTS,
    },
]
for i, waypoint in enumerate(LAPF_CONTROL_POINTS, start=1):
    MISSION_STAGES.append(
        {
            "name": f"Obstacle course LAPF waypoint {i}",
            "type": "lapf",
            "waypoint": waypoint,
        }
    )

# ==============================================================================
# BURGER PICKUP / LIFT / CLAW CONFIG
# ==============================================================================
LIFT_MOTOR = Motor.DC_M3
LIFT_CARRY_TICKS = -14000
LIFT_PICKUP_TICKS = -9000
LIFT_DOWN_TICKS = 0
LIFT_MAX_VEL = 1800
LIFT_TOLERANCE = 30
LIFT_JOG_STEP = 100
LIFT_TIMEOUT_S = 20.0
LIFT_ITEM_THICKNESS_TICKS = -1800

CLAW_SERVO = ServoChannel.CH_13
CLAW_OPEN_DEG = 60.0
CLAW_CLOSE_MEAT_DEG = 150.0
CLAW_CLOSE_BUN_DEG = 140.0
CLAW_ULTRASONIC_LIM = Limit.LIM_2
CLAW_GRAB_CONFIRMED = True

DRIVE_VELOCITY = 100.0
APPROACH_VELOCITY = 60.0
POS_TOLERANCE_MM = 20.0
TURN_TO_SHELF_DEG = 79
TURN_FROM_SHELF_DEG = -79
TURN_TOLERANCE_DEG = 2.0
TURN_VELOCITY = 30.0
SERVO_DEG_PER_STEP = 0.8

# --- ADJUSTED DISTANCES & ANGLES ---
INITIAL_MOVE_DIST = 200.0
DIST_TO_INGREDIENT_AREA = 780.0  # 980.0 - 200.0
TRAFFIC_LIGHT_TURN_DEG = 45.0
# -----------------------------------

APPROACH_SHELF_DIST = 30.0
INGREDIENT_SLOTS = {
    "bun_bottom": 207.0,
    "meat": 350.0,
    "bun_top": 485.0,
}
FETCH_ORDER = ["meat", "bun_top"]
ASSEMBLY_SLOT = "bun_bottom"

# ==============================================================================
# HELPER FUNCTIONS
# ==============================================================================
def resolve_lapf_config() -> dict[str, float]:
    return {
        "leash_length_mm": float(LEASH_LENGTH_MM),
        "repulsion_range_mm": float(REPULSION_RANGE_MM),
        "target_speed_mm_s": float(TARGET_SPEED_MM_S),
        "repulsion_gain": float(REPULSION_GAIN),
        "attraction_gain": float(ATTRACTION_GAIN),
        "force_ema_alpha": float(FORCE_EMA_ALPHA),
        "inflation_margin_mm": float(INFLATION_MARGIN_MM),
        "leash_half_angle_deg": float(LEASH_HALF_ANGLE_DEG),
    }

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
    if ENABLE_VISION:
        robot.enable_vision()
    if ENABLE_LIDAR:
        robot.enable_lidar()
        robot.set_lidar_mount(x_mm=LIDAR_MOUNT_X_MM, y_mm=LIDAR_MOUNT_Y_MM, theta_deg=LIDAR_MOUNT_THETA_DEG)
        robot.set_lidar_filter(range_min_mm=LIDAR_RANGE_MIN_MM, range_max_mm=LIDAR_RANGE_MAX_MM, fov_deg=LIDAR_FOV_DEG)
        robot.start_lidar_world_publisher()
    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
    if ENABLE_GPS_TANGENT_HEADING:
        robot.enable_gps_tangent_heading(alpha=GPS_TANGENT_ALPHA, min_displacement_mm=GPS_TANGENT_MIN_DISPLACEMENT_MM)

def start_robot(robot: Robot) -> None:
    state = robot.get_state()
    if state in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

def reset_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed; continuing")
    robot.wait_for_pose_update(timeout=0.5)

# --- LED Helpers ---
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

# --- Lift & Claw Helpers ---
def get_lift_ticks(robot: Robot) -> int:
    dc = robot.get_dc_state()
    if dc is None:
        return 0
    return int(dc.motors[LIFT_MOTOR - 1].position)

def lift_return_to_zero(robot: Robot) -> None:
    current = get_lift_ticks(robot)
    if abs(current) <= LIFT_TOLERANCE:
        return
    travel_s = abs(current) / LIFT_MAX_VEL
    timeout = max(15.0, travel_s * 1.5)
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    ok = robot.set_motor_position(LIFT_MOTOR, 0, max_vel_ticks=LIFT_MAX_VEL, tolerance_ticks=LIFT_TOLERANCE, blocking=True, timeout=timeout)
    if not ok:
        robot.disable_motor(LIFT_MOTOR)

def claw_close(robot: Robot, angle: float = CLAW_CLOSE_BUN_DEG) -> None:
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, angle)
    time.sleep(0.5)

def claw_open(robot: Robot) -> None:
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_OPEN_DEG)
    time.sleep(0.5)

def claw_has_object(robot: Robot) -> bool:
    return robot.get_limit(CLAW_ULTRASONIC_LIM) == CLAW_GRAB_CONFIRMED

# --- Vision Helper ---
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

# --- Navigation & Course Helpers ---
def turn_to_face_shelf(robot: Robot) -> None:
    robot.turn_by(delta_deg=TURN_TO_SHELF_DEG, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

def approach_shelf(robot: Robot) -> None:
    robot.move_forward(distance=APPROACH_SHELF_DIST, velocity=APPROACH_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)

def drive_to_slot(robot: Robot, from_slot: str | None, to_slot: str) -> None:
    from_dist = INGREDIENT_SLOTS.get(from_slot, 0.0) if from_slot else 0.0
    to_dist = INGREDIENT_SLOTS[to_slot]
    delta = to_dist - from_dist
    if abs(delta) < 1.0:
        return
    if delta > 0:
        robot.move_forward(distance=abs(delta), velocity=DRIVE_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)
    else:
        robot.move_backward(distance=abs(delta), velocity=DRIVE_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)

def start_pure_pursuit_stage(robot: Robot, stage: dict[str, Any]):
    waypoints = stage["waypoints"]
    return robot.purepursuit_follow_path(
        waypoints=waypoints,
        velocity=PURE_PURSUIT_VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=PURE_PURSUIT_TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=PURE_PURSUIT_MAX_ANGULAR_RAD_S,
        blocking=False,
    )

def start_lapf_stage(robot: Robot, stage: dict[str, Any]):
    cfg = resolve_lapf_config()
    goal_x, goal_y = stage["waypoint"]
    return robot.lapf_to_goal(
        goal_x, goal_y,
        velocity=LAPF_VELOCITY_MM_S,
        tolerance=LAPF_TOLERANCE_MM,
        leash_length_mm=cfg["leash_length_mm"],
        repulsion_range_mm=cfg["repulsion_range_mm"],
        target_speed_mm_s=cfg["target_speed_mm_s"],
        max_angular_rad_s=LAPF_MAX_ANGULAR_RAD_S,
        repulsion_gain=cfg["repulsion_gain"],
        attraction_gain=cfg["attraction_gain"],
        force_ema_alpha=cfg["force_ema_alpha"],
        inflation_margin_mm=cfg["inflation_margin_mm"],
        leash_half_angle_deg=cfg["leash_half_angle_deg"],
        blocking=False,
    )

def start_course_stage(robot: Robot, stage_index: int):
    stage = MISSION_STAGES[stage_index]
    if stage["type"] == "pure_pursuit":
        return start_pure_pursuit_stage(robot, stage)
    if stage["type"] == "lapf":
        return start_lapf_stage(robot, stage)
    raise ValueError(f"Unknown stage type: {stage['type']}")


# ==============================================================================
# MAIN STATE MACHINE
# ==============================================================================
def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)
    reset_pose(robot)
    
    state = "INIT"
    current_slot = None
    pick_attempts = 0
    active_drop_ticks = 0
    
    action_sub_state = "INIT"
    action_timer = 0.0
    next_fsm_state = ""
    manual_skip_triggered = False
    
    # --- RESTORED TIMING VARIABLES ---
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    
    internal_target_ticks = 0
    requested_final_ticks = 0
    last_step_time = 0.0
    step_delay_s = 0.150
    # ---------------------------------

    requested_claw_deg = CLAW_CLOSE_BUN_DEG
    current_claw_deg = CLAW_CLOSE_BUN_DEG
    active_close_deg = CLAW_CLOSE_BUN_DEG
    
    stage_index = 0
    motion_handle = None

    print("========================================================")
    print(" COMBINED MISSION: Burger Pickup -> Pure Pursuit Course ")
    print("========================================================")

    while True:
        # ---------------------------------------------------------
        # 1. INIT & TRAFFIC LIGHT BYPASS
        # ---------------------------------------------------------
        if state == "INIT":
            robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
            claw_close(robot, CLAW_CLOSE_BUN_DEG) 
            led_idle(robot)
            print("[INIT] Align carriage to mark. BTN_1: UP | BTN_2: DOWN | BTN_10: Confirm")
            state = "INIT_JOG"
            
        elif state == "INIT_JOG":
            if robot.was_button_pressed(Button.BTN_1):
                requested_final_ticks += LIFT_JOG_STEP
                robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
            elif robot.was_button_pressed(Button.BTN_2):
                requested_final_ticks -= LIFT_JOG_STEP
                robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
            elif robot.was_button_pressed(Button.BTN_10):
                robot.reset_motor_position(LIFT_MOTOR)
                internal_target_ticks = 0
                requested_final_ticks = 0
                time.sleep(0.15)
                print("[INIT] Encoder zeroed -> INITIAL_MOVE_FORWARD")
                led_moving(robot)
                state = "INITIAL_MOVE_FORWARD"

        elif state == "INITIAL_MOVE_FORWARD":
            print(f"[NAV] Moving forward {INITIAL_MOVE_DIST} mm before turn")
            robot.move_forward(distance=INITIAL_MOVE_DIST, velocity=DRIVE_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)
            robot.stop()
            state = "WAIT_BEFORE_TURN"

        elif state == "WAIT_BEFORE_TURN":
            time.sleep(0.5)
            state = "TURN_TO_LIGHT"

        elif state == "TURN_TO_LIGHT":
            print(f"[NAV] Turning {TRAFFIC_LIGHT_TURN_DEG} degrees to face traffic light")
            robot.turn_by(delta_deg=TRAFFIC_LIGHT_TURN_DEG, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
            robot.stop()
            state = "WAIT_AFTER_TURN"
            
        elif state == "WAIT_AFTER_TURN":
            time.sleep(0.5) 
            if ENABLE_VISION:
                robot.get_detections("traffic light") 
            state = "WAIT_GREEN_LIGHT"

        elif state == "WAIT_GREEN_LIGHT":
            if detect_green_light(robot) or robot.was_button_pressed(Button.BTN_5):
                print("[FSM] Green light detected! Returning to heading.")
                robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                state = "RETURN_TO_HEADING"

        elif state == "RETURN_TO_HEADING":
            print(f"[NAV] Turning back {-TRAFFIC_LIGHT_TURN_DEG} degrees")
            robot.turn_by(delta_deg=-TRAFFIC_LIGHT_TURN_DEG, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
            robot.stop()
            state = "WAIT_AFTER_RETURN"
            
        elif state == "WAIT_AFTER_RETURN":
            time.sleep(0.5)
            state = "PREP_INITIAL_MOVE"

        # ---------------------------------------------------------
        # 2. BURGER PICKUP SEQUENCE
        # ---------------------------------------------------------
        elif state == "PREP_INITIAL_MOVE":
            print("[ARM] OPENING claw for travel heading...")
            robot.enable_servo(CLAW_SERVO)
            requested_claw_deg = CLAW_OPEN_DEG
            state = "WAIT_PREP_INITIAL_CLAW"
            
        elif state == "WAIT_PREP_INITIAL_CLAW":
            if abs(current_claw_deg - requested_claw_deg) <= 0.5:
                print("[ARM] Claw opened. Raising lift to carry height...")
                requested_final_ticks = LIFT_CARRY_TICKS
                action_timer = time.monotonic()
                state = "WAIT_PREP_INITIAL_LIFT"
                
        elif state == "WAIT_PREP_INITIAL_LIFT":
            current = get_lift_ticks(robot)
            if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or (time.monotonic() - action_timer > LIFT_TIMEOUT_S):
                print("[ARM] Arm prepped at carry height. Moving out.")
                state = "BURGER_PICKUP"

        elif state == "BURGER_PICKUP":
            print(f"[NAV] Driving to ingredient area (Adjusted Dist: {DIST_TO_INGREDIENT_AREA}mm)")
            robot.move_forward(distance=DIST_TO_INGREDIENT_AREA, velocity=DRIVE_VELOCITY, tolerance=POS_TOLERANCE_MM, blocking=True)
            robot.stop()
            current_slot = None
            pick_attempts = 0
            state = "FETCH_MEAT"
            
        elif state == "FETCH_MEAT":
            target = FETCH_ORDER[0] # "meat"
            drive_to_slot(robot, current_slot, target)
            current_slot = target
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            active_close_deg = CLAW_CLOSE_MEAT_DEG
            action_sub_state = "OPEN_CLAW"
            next_fsm_state = "PLACE_MEAT"
            state = "DO_PICK"

        elif state == "PLACE_MEAT":
            drive_to_slot(robot, current_slot, ASSEMBLY_SLOT)
            current_slot = ASSEMBLY_SLOT
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            active_drop_ticks = LIFT_PICKUP_TICKS + (1 * LIFT_ITEM_THICKNESS_TICKS)
            action_sub_state = "LIFT_DOWN_TO_STACK"
            next_fsm_state = "FETCH_BUN_TOP"
            state = "DO_PLACE"

        elif state == "FETCH_BUN_TOP":
            target = FETCH_ORDER[1] # "bun_top"
            drive_to_slot(robot, current_slot, target)
            current_slot = target
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            active_close_deg = CLAW_CLOSE_BUN_DEG
            action_sub_state = "OPEN_CLAW"
            next_fsm_state = "PLACE_AND_GRAB_STACK"
            state = "DO_PICK"

        elif state == "PLACE_AND_GRAB_STACK":
            drive_to_slot(robot, current_slot, ASSEMBLY_SLOT)
            current_slot = ASSEMBLY_SLOT
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            active_drop_ticks = LIFT_PICKUP_TICKS + (2 * LIFT_ITEM_THICKNESS_TICKS)
            action_sub_state = "WAIT_OPEN"
            requested_claw_deg = CLAW_OPEN_DEG
            next_fsm_state = "BURGER_PICKUP_DONE"
            state = "DO_PLACE_AND_GRAB"

        # --- Reusable Sub-Routines for Arm Actions ---
        elif state == "DO_PICK":
            if robot.was_button_pressed(Button.BTN_3):
                manual_skip_triggered = True
                
            if action_sub_state == "OPEN_CLAW":
                robot.enable_servo(CLAW_SERVO)
                requested_claw_deg = CLAW_OPEN_DEG
                action_sub_state = "WAIT_LIFT_DOWN"
                
            elif action_sub_state == "WAIT_LIFT_DOWN":
                requested_final_ticks = LIFT_PICKUP_TICKS
                action_timer = time.monotonic()
                action_sub_state = "WAIT_CLOSE"

            elif action_sub_state == "WAIT_CLOSE":
                requested_claw_deg = active_close_deg
                if abs(current_claw_deg - requested_claw_deg) <= 0.5:
                    requested_final_ticks = LIFT_CARRY_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_UP"
                    
            elif action_sub_state == "WAIT_LIFT_UP":
                current = get_lift_ticks(robot)
                if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or (time.monotonic() - action_timer > LIFT_TIMEOUT_S):
                    state = next_fsm_state

        elif state == "DO_PLACE":
            if action_sub_state == "LIFT_DOWN_TO_STACK":
                requested_final_ticks = active_drop_ticks
                action_timer = time.monotonic()
                action_sub_state = "WAIT_OPEN"
            
            elif action_sub_state == "WAIT_OPEN":
                current = get_lift_ticks(robot)
                if abs(current - active_drop_ticks) <= LIFT_TOLERANCE or (time.monotonic() - action_timer > LIFT_TIMEOUT_S):
                    requested_claw_deg = CLAW_OPEN_DEG
                    if abs(current_claw_deg - requested_claw_deg) <= 0.5:
                        requested_final_ticks = LIFT_CARRY_TICKS
                        action_timer = time.monotonic()
                        action_sub_state = "WAIT_LIFT_UP"
                        
            elif action_sub_state == "WAIT_LIFT_UP":
                current = get_lift_ticks(robot)
                if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or (time.monotonic() - action_timer > LIFT_TIMEOUT_S):
                    state = next_fsm_state

        elif state == "DO_PLACE_AND_GRAB":
            if action_sub_state == "WAIT_OPEN":
                if abs(current_claw_deg - requested_claw_deg) <= 0.5:
                    requested_final_ticks = LIFT_PICKUP_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_DOWN_BASE"
                    
            elif action_sub_state == "WAIT_LIFT_DOWN_BASE":
                current = get_lift_ticks(robot)
                if abs(current - LIFT_PICKUP_TICKS) <= LIFT_TOLERANCE or (time.monotonic() - action_timer > LIFT_TIMEOUT_S):
                    requested_claw_deg = CLAW_CLOSE_BUN_DEG
                    action_sub_state = "WAIT_LIFT_UP_FINAL"

            elif action_sub_state == "WAIT_LIFT_UP_FINAL":
                if abs(current_claw_deg - CLAW_CLOSE_BUN_DEG) <= 0.5:
                    requested_final_ticks = LIFT_CARRY_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_UP"

            elif action_sub_state == "WAIT_LIFT_UP":
                current = get_lift_ticks(robot)
                if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or (time.monotonic() - action_timer > LIFT_TIMEOUT_S):
                    state = next_fsm_state

        elif state == "BURGER_PICKUP_DONE":
            print("[FSM] Burger Pickup complete! Starting Pure Pursuit Mission!")
            reset_pose(robot)  # Reset odometry before course
            state = "START_COURSE"

        # ---------------------------------------------------------
        # 3. PURE PURSUIT & OBSTACLE COURSE
        # ---------------------------------------------------------
        elif state == "START_COURSE":
            if stage_index >= len(MISSION_STAGES):
                print("[FSM] All mission stages complete!")
                state = "DONE"
                continue
            
            motion_handle = start_course_stage(robot, stage_index)
            state = "MOVING_COURSE"
            
        elif state == "MOVING_COURSE":
            if motion_handle is not None and motion_handle.is_done():
                print(f"[FSM] Stage {stage_index + 1} complete.")
                stage_index += 1
                state = "START_COURSE"

        elif state == "DONE":
            robot.stop()
            led_done(robot)
            break

        # ==================================================================
        # ASYNCHRONOUS MOTOR SPOON-FEEDER (The Physical Leash Fix)
        # ==================================================================
        current_phys = get_lift_ticks(robot)
        if abs(current_phys - requested_final_ticks) > LIFT_TOLERANCE:
            now = time.monotonic()
            if now - last_step_time > step_delay_s:
                last_step_time = now
                robot.set_motor_position(
                    LIFT_MOTOR, 
                    requested_final_ticks, 
                    max_vel_ticks=LIFT_MAX_VEL, 
                    tolerance_ticks=LIFT_TOLERANCE, 
                    blocking=False
                )
            
        if requested_claw_deg > current_claw_deg:
            current_claw_deg = min(requested_claw_deg, current_claw_deg + SERVO_DEG_PER_STEP)
        else:
            current_claw_deg = max(requested_claw_deg, current_claw_deg - SERVO_DEG_PER_STEP)
            
        robot.set_servo(CLAW_SERVO, current_claw_deg)
        
        # Estop Break Check
        if state not in {"INIT", "INIT_JOG"}:
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                led_error(robot)
                lift_return_to_zero(robot)
                robot.estop()
                break

        # ── tick-rate control ──────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
