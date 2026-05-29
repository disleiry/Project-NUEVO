import time
from typing import Any
from robot.robot import FirmwareState, Robot
from robot.hardware_map import (
    Button, DCMotorMode, Motor, ServoChannel, LED, LEDMode,
    POSITION_UNIT, WHEEL_DIAMETER, WHEEL_BASE, INITIAL_THETA_DEG,
    LEFT_WHEEL_MOTOR, LEFT_WHEEL_DIR_INVERTED, 
    RIGHT_WHEEL_MOTOR, RIGHT_WHEEL_DIR_INVERTED,
    LIDAR_FOV_DEG, LIDAR_MOUNT_THETA_DEG, LIDAR_MOUNT_X_MM, LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM, LIDAR_RANGE_MIN_MM, TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM,
    DEFAULT_FSM_HZ
)
from robot.util import densify_polyline

# ==========================================
# --- BURGER PICKUP & ARM CONSTANTS ---
# ==========================================
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
CLAW_CLOSE_BUN_DEG = 142.0

DRIVE_VELOCITY = 150.0
APPROACH_VELOCITY = 60.0
POS_TOLERANCE_MM = 20.0

TURN_VELOCITY_DEG = 45.0
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

# ==========================================
# --- SENSOR & SENSOR FUSION SETUP ---
# ==========================================
ENABLE_VISION = True
TAG_ID = 25
GPS_POSITION_ALPHA = 0.20
ENABLE_GPS_TANGENT_HEADING = True
GPS_TANGENT_ALPHA = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0

# ==========================================
# --- NAVIGATION & PATH GENERATION ---
# ==========================================
PURE_PURSUIT_VELOCITY_MM_S = 150.0
LOOKAHEAD_MM = 225.0
PURE_PURSUIT_TOLERANCE_MM = 25.0
ADVANCE_RADIUS_MM = 75.0
PURE_PURSUIT_MAX_ANGULAR_RAD_S = 1.5

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

STATUS_PRINT_INTERVAL_S = 0.5
STAGE_PAUSE_S = 0.00

LAPF_CONTROL_POINTS = [
    (1800.0, 3250.0), 
]
LAPF_CONTROL_POINTS = densify_polyline(LAPF_CONTROL_POINTS, spacing=50.0)

MISSION_STAGES: list[dict[str, Any]] = [
    {
        "name": "Straight/ramp pure pursuit",
        "type": "pure_pursuit",
        "waypoints": PURE_PURSUIT_CONTROL_POINTS,
    },
]
for i, waypoint in enumerate(LAPF_CONTROL_POINTS, start=1):
    MISSION_STAGES.append({
        "name": f"Obstacle course LAPF waypoint {i}",
        "type": "lapf",
        "waypoint": waypoint,
    })

# ==========================================
# --- GENERAL HELPERS & CONFIGURATIONS ---
# ==========================================
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

# STEP 1: Baseline Odometry setup only (for burger maneuvers)
def configure_and_start_robot_for_pickup(robot: Robot) -> None:
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
        
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)

# STEP 2: Activated immediately after burger pickup is completed
def enable_gps_after_pickup(robot: Robot) -> None:
    print("[INIT] Burger pickup done -> Connecting GPS sensor fusion streams...")
    robot.enable_gps()
    robot.set_tracked_tag_id(TAG_ID)
    robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
    robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
    if ENABLE_GPS_TANGENT_HEADING:
        robot.enable_gps_tangent_heading(
            alpha=GPS_TANGENT_ALPHA,
            min_displacement_mm=GPS_TANGENT_MIN_DISPLACEMENT_MM,
        )
    # Give the localization filter a split second to lock before moving onto the ramp
    time.sleep(0.3)
    robot.wait_for_pose_update(timeout=0.5)

# STEP 3: Activated ONLY when transitioning into the LAPF stage at (1600, 600)
def enable_lidar_for_obstacles(robot: Robot) -> None:
    print("[INIT] Reached (1600, 600) -> Activating LIDAR tracking layers...")
    robot.enable_lidar()
    robot.set_lidar_mount(
        x_mm=LIDAR_MOUNT_X_MM,
        y_mm=LIDAR_MOUNT_Y_MM,
        theta_deg=LIDAR_MOUNT_THETA_DEG,
    )
    robot.set_lidar_filter(
        range_min_mm=LIDAR_RANGE_MIN_MM,
        range_max_mm=LIDAR_RANGE_MAX_MM,
        fov_deg=LIDAR_FOV_DEG,
    )
    robot.start_lidar_world_publisher()
    time.sleep(0.1)

def get_best_pose(robot: Robot) -> tuple[str, float, float, float]:
    if robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        return "fused", x, y, theta
    x, y, theta = robot.get_odometry_pose()
    return "odom ", x, y, theta

# ==========================================
# --- HARDWARE CONTROLS (BURGER TASK) ---
# ==========================================
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
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    current_ticks = float(get_lift_ticks(robot))
    
    step_rate_hz = 50.0
    delay_s = 1.0 / step_rate_hz
    ticks_per_step = LIFT_MAX_VEL / step_rate_hz
    internal_target = current_ticks
    
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

# ==========================================
# --- STAGE INITIALIZATION FOR THE MISSION ---
# ==========================================
def start_pure_pursuit_stage(robot: Robot, stage: dict[str, Any]):
    waypoints = stage["waypoints"]
    print(
        f"[FSM] MOVING - pure pursuit stage with {len(waypoints)} waypoints "
        f"ending at ({waypoints[-1][0]:.0f}, {waypoints[-1][1]:.0f}) mm"
    )
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
    # Turn on LIDAR mapping layers ONLY right here as we start Stage 2/2
    enable_lidar_for_obstacles(robot)
    
    cfg = resolve_lapf_config()
    goal_x, goal_y = stage["waypoint"]
    print(f"[FSM] MOVING - LAPF obstacle waypoint goal=({goal_x:.0f}, {goal_y:.0f}) mm")
    return robot.lapf_to_goal(
        goal_x,
        goal_y,
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

def print_course_status(robot: Robot, stage_index: int) -> None:
    stage = MISSION_STAGES[stage_index]
    label, x, y, theta = get_best_pose(robot)
    if stage["type"] == "lapf":
        goal_x, goal_y = stage["waypoint"]
    else:
        goal_x, goal_y = stage["waypoints"][-1]
        
    virtual_target = robot.get_virtual_target()
    obstacle_tracks = robot.get_obstacle_tracks()
    
    if virtual_target is None:
        vt_summary = " vt=(none)"
    else:
        vt_summary = f" vt=({virtual_target[0]:.0f}, {virtual_target[1]:.0f}) mm"
        
    if obstacle_tracks:
        nearest_boundary_mm = min(
            max(
                0.0,
                ((float(track["x"]) - x)**2 + (float(track["y"]) - y)**2)**0.5
                - float(track["radius"]),
            )
            for track in obstacle_tracks
        )
        track_summary = f" tracked={len(obstacle_tracks)} nearest_track={nearest_boundary_mm:.0f} mm"
    else:
        track_summary = " tracked=0"
        
    print(
        f" stage {stage_index + 1}/{len(MISSION_STAGES)} {stage['name']} | "
        f"goal=({goal_x:.0f}, {goal_y:.0f}) mm | "
        f"{label}=({x:.0f}, {y:.0f}) mm θ={theta:.1f}°"
        f"{vt_summary}{track_summary}"
    )

# ==========================================
# --- STATE MACHINE CONTROL LOOP ---
# ==========================================
def run(robot: Robot) -> None:
    print("[INIT] Configuring isolated odometry mappings...")
    configure_and_start_robot_for_pickup(robot)
    
    state = "INIT"
    current_x = 0.0
    jog_ticks = 0
    motion_handle = None
    course_stage_index = 0
    last_status_print_at = 0.0

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
            if robot.was_button_pressed(Button.BTN_5): 
                print("[FSM] Starting pickup sequence on odometry baseline.")
                state = "BURGER_PICKUP"
            time.sleep(0.1)

        elif state == "BURGER_PICKUP":
            # 1. PREP INITIAL ARM MOVE
            claw_open(robot)
            move_lift(robot, LIFT_CARRY_TICKS)
            
            # 2. NAVIGATE TO INGREDIENT AREA
            robot.move_forward(DIST_TO_INGREDIENT_AREA, DRIVE_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            
            # 3. ACCUMULATE MEAT
            current_x = drive_to_slot(robot, current_x, "meat")
            robot.turn_by(TURN_TO_SHELF_DEG, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            robot.move_forward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            
            move_lift(robot, LIFT_PICKUP_TICKS)
            claw_close(robot, CLAW_CLOSE_MEAT_DEG)
            move_lift(robot, LIFT_CARRY_TICKS)
            
            robot.move_backward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            robot.turn_by(TURN_FROM_SHELF_DEG, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            
            # 4. PLACE MEAT
            current_x = drive_to_slot(robot, current_x, "bun_bottom")
            robot.turn_by(TURN_TO_SHELF_DEG, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            robot.move_forward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            
            move_lift(robot, LIFT_PICKUP_TICKS + LIFT_ITEM_THICKNESS_TICKS)
            claw_open(robot)
            move_lift(robot, LIFT_CARRY_TICKS)
            
            robot.move_backward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            robot.turn_by(TURN_FROM_SHELF_DEG, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

            # 5. FETCH TOP BUN
            current_x = drive_to_slot(robot, current_x, "bun_top")
            robot.turn_by(TURN_TO_SHELF_DEG, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            robot.move_forward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            
            move_lift(robot, LIFT_PICKUP_TICKS)
            claw_close(robot, CLAW_CLOSE_BUN_DEG)
            move_lift(robot, LIFT_CARRY_TICKS)
            
            robot.move_backward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            robot.turn_by(TURN_FROM_SHELF_DEG, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

            # 6. PLACE TOP BUN & GRAB FULL STACK
            current_x = drive_to_slot(robot, current_x, "bun_bottom")
            robot.turn_by(TURN_TO_SHELF_DEG, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            robot.move_forward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            
            move_lift(robot, LIFT_PICKUP_TICKS + (2 * LIFT_ITEM_THICKNESS_TICKS)) 
            claw_open(robot)
            
            move_lift(robot, LIFT_PICKUP_TICKS)
            claw_close(robot, CLAW_CLOSE_BUN_DEG)
            move_lift(robot, LIFT_CARRY_TICKS)
            
            robot.move_backward(APPROACH_SHELF_DIST, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            robot.turn_by(TURN_FROM_SHELF_DEG, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

            print("[FSM] Burger pickup complete! Turning on GPS streams...")
            
            # Fire up GPS stream and snap to absolute map frames right here
            enable_gps_after_pickup(robot)
            
            course_stage_index = 0
            last_status_print_at = time.monotonic()
            motion_handle = start_course_stage(robot, course_stage_index)
            state = "COURSE_MOVING"

        elif state == "COURSE_MOVING":
            now = time.monotonic()
            
            if robot.was_button_pressed(Button.BTN_2):
                if motion_handle is not None:
                    motion_handle.cancel()
                    motion_handle.wait(timeout=1.0)
                robot.stop()
                print("[FSM] IDLE — course trajectory aborted.")
                state = "DONE"
                
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_course_status(robot, course_stage_index)
                    last_status_print_at = now
                
                if motion_handle is not None and motion_handle.is_finished():
                    robot.stop()
                    print(f"[FSM] Finished stage {course_stage_index + 1}: {MISSION_STAGES[course_stage_index]['name']}")
                    
                    if course_stage_index + 1 < len(MISSION_STAGES):
                        course_stage_index += 1
                        if STAGE_PAUSE_S > 0:
                            time.sleep(STAGE_PAUSE_S)
                        # When entering stage index 1 (the LAPF stage), it will activate the LIDAR
                        motion_handle = start_course_stage(robot, course_stage_index)
                    else:
                        print("[FSM] Course tracking complete! Target reached.")
                        state = "DONE"
                        
            time.sleep(0.05)

        elif state == "DONE":
            time.sleep(1.0)
