"""
main.py — traffic light + burger build + ramp/LAPF course navigation
=====================================================================

Mission order:
1. Align/zero the lift using BTN_1/BTN_2 and confirm with BTN_10.
2. Move forward 400 mm, turn to face the traffic light, wait until it is green, then turn forward.
3. Build the burger using the lift/claw pickup sequence.
4. Reset odometry and run the ramp/course navigation:
   - Pure pursuit through the ramp path.
   - LAPF + LiDAR for the obstacle-course section.

Controls:
    BTN_1 / BTN_2 during INIT_JOG: jog lift up/down.
    BTN_10 during INIT_JOG: zero lift encoder and continue.
    BTN_5 during WAIT_GREEN: manual green override.
    BTN_3 during pickup: manual grab-skip override.
    BTN_2 during most active states: cancel / estop.
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
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    Limit,
    Motor,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    ServoChannel,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline


# ===========================================================================
# SENSOR SETUP
# ===========================================================================

ENABLE_VISION = True
ENABLE_LIDAR = True
ENABLE_GPS = True

# IMPORTANT: update this to match the ArUco marker ID on your robot.
TAG_ID = 25

GPS_POSITION_ALPHA = 0.01
ENABLE_GPS_TANGENT_HEADING = True
GPS_TANGENT_ALPHA = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0


# ===========================================================================
# TRAFFIC LIGHT START
# ===========================================================================

LED_BRIGHTNESS = 255
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_CONFIDENCE = 0.50

# Before looking for the traffic light, move forward a short distance.
# This gives the camera a better view before the robot turns in place.
TRAFFIC_LIGHT_PRE_MOVE_MM = 400.0
TRAFFIC_LIGHT_PRE_MOVE_VELOCITY_MM_S = 100.0
TRAFFIC_LIGHT_PRE_MOVE_TOLERANCE_MM = 20.0

# Absolute headings for traffic light viewing and returning.
# Assuming INITIAL_THETA_DEG is 90, 120/125 looks left of forward.
TRAFFIC_LIGHT_LOOK_ANGLE_DEG = 125.0
TRAFFIC_LIGHT_RETURN_ANGLE_DEG = 90.0
TRAFFIC_TURN_TOLERANCE_DEG = 2.0

# Safety override. Usually false for competition so a random sign doesn't stop the run.
ENABLE_STOP_SIGN_OVERRIDE = False


# ===========================================================================
# BURGER LIFT / CLAW CONFIGURATION
# ===========================================================================

LIFT_MOTOR = Motor.DC_M3
LIFT_CARRY_TICKS = -14000
LIFT_PICKUP_TICKS = -9000
LIFT_DOWN_TICKS = 0
LIFT_MAX_VEL = 1800
LIFT_TOLERANCE = 30
LIFT_JOG_STEP = 100
LIFT_TIMEOUT_S = 20.0

# Offset for each stacked burger piece.
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
SHELF_TURN_TOLERANCE_DEG = 2.0

SERVO_DEG_PER_STEP = 0.8

DIST_TO_INGREDIENT_AREA = 980.0
APPROACH_SHELF_DIST = 30.0

INGREDIENT_SLOTS = {
    "bun_bottom": 207.0,
    "meat": 350.0,
    "bun_top": 485.0,
}

FETCH_ORDER = ["meat", "bun_top"]
ASSEMBLY_SLOT = "bun_bottom"
MAX_PICK_ATTEMPTS = 1


# ===========================================================================
# RAMP / COURSE NAVIGATION
# ===========================================================================

PURE_PURSUIT_CONTROL_POINTS = [
    (300.0, 3500.0),    # Home straight
    (920.0, 3500.0),    # Transition / turn
    (920.0, 700.0),     # Ramp / return direction
    (1800.0, 700.0),    # Entrance toward obstacle course
]
PURE_PURSUIT_CONTROL_POINTS = densify_polyline(PURE_PURSUIT_CONTROL_POINTS, spacing=100.0)

LAPF_CONTROL_POINTS = [
    (1800.0, 3250.0),   # Obstacle waypoint / finish
]
LAPF_CONTROL_POINTS = densify_polyline(LAPF_CONTROL_POINTS, spacing=50.0)

PURE_PURSUIT_VELOCITY_MM_S = 150.0
LOOKAHEAD_MM = 200.0
PURE_PURSUIT_TOLERANCE_MM = 25.0
ADVANCE_RADIUS_MM = 150.0
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

STATUS_PRINT_INTERVAL_S = 0.5
STAGE_PAUSE_S = 0.00

COURSE_STAGES: list[dict[str, Any]] = [
    {
        "name": "Straight/ramp pure pursuit",
        "type": "pure_pursuit",
        "waypoints": PURE_PURSUIT_CONTROL_POINTS,
    },
]

for i, waypoint in enumerate(LAPF_CONTROL_POINTS, start=1):
    COURSE_STAGES.append(
        {
            "name": f"Obstacle course LAPF waypoint {i}",
            "type": "lapf",
            "waypoint": waypoint,
        }
    )


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


def dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)


def show_traffic_light_color(robot: Robot, color: str) -> None:
    if color == "red":
        robot.set_led(LED.RED, LED_BRIGHTNESS)
        robot.set_led(LED.GREEN, 0)
    elif color == "green":
        robot.set_led(LED.RED, 0)
        robot.set_led(LED.GREEN, LED_BRIGHTNESS)


# ===========================================================================
# ROBOT SETUP
# ===========================================================================

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
        print("[sensor] vision enabled — traffic-light detection active")

    if ENABLE_LIDAR:
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
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")

        if ENABLE_GPS_TANGENT_HEADING:
            robot.enable_gps_tangent_heading(
                alpha=GPS_TANGENT_ALPHA,
                min_displacement_mm=GPS_TANGENT_MIN_DISPLACEMENT_MM,
            )
            print("[sensor] GPS tangent heading enabled")


def start_robot(robot: Robot) -> None:
    state = robot.get_state()
    if state in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)


# ===========================================================================
# LIFT / CLAW HELPERS
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
        timeout = max(15.0, travel_s * 1.5)
        robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)

        ok = robot.set_motor_position(
            LIFT_MOTOR,
            0,
            max_vel_ticks=LIFT_MAX_VEL,
            tolerance_ticks=LIFT_TOLERANCE,
            blocking=True,
            timeout=timeout,
        )
        if ok:
            print("[LIFT] Reached origin. Safe to power off.")
        else:
            print("[warn] LIFT — did not confirm origin.")
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


def update_arm_targets(
    robot: Robot,
    requested_final_ticks: int,
    internal_target_ticks: int,
    last_step_time: float,
    requested_claw_deg: float,
    current_claw_deg: float,
    step_delay_s: float = 0.150,
) -> tuple[int, float, float]:
    """Spoon-feed lift target and slowly move claw servo.

    Returns:
        internal_target_ticks, last_step_time, current_claw_deg
    """
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
                blocking=False,
            )
            last_step_time = now

    if abs(current_claw_deg - requested_claw_deg) > 0.5:
        if requested_claw_deg > current_claw_deg:
            current_claw_deg = min(requested_claw_deg, current_claw_deg + SERVO_DEG_PER_STEP)
        else:
            current_claw_deg = max(requested_claw_deg, current_claw_deg - SERVO_DEG_PER_STEP)

        robot.set_servo(CLAW_SERVO, current_claw_deg)

    return internal_target_ticks, last_step_time, current_claw_deg


# ===========================================================================
# BURGER NAVIGATION HELPERS
# ===========================================================================

def turn_to_face_shelf(robot: Robot) -> None:
    print(f"[NAV] Turn {TURN_TO_SHELF_DEG}° to face shelf")
    robot.turn_by(
        delta_deg=TURN_TO_SHELF_DEG,
        blocking=True,
        tolerance_deg=SHELF_TURN_TOLERANCE_DEG,
    )


def turn_away_from_shelf(robot: Robot) -> None:
    print(f"[NAV] Turn {TURN_FROM_SHELF_DEG}° to resume heading")
    robot.turn_by(
        delta_deg=TURN_FROM_SHELF_DEG,
        blocking=True,
        tolerance_deg=SHELF_TURN_TOLERANCE_DEG,
    )


def approach_shelf(robot: Robot) -> None:
    print(f"[NAV] Approach shelf {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_forward(
        distance=APPROACH_SHELF_DIST,
        velocity=APPROACH_VELOCITY,
        tolerance=POS_TOLERANCE_MM,
        blocking=True,
    )


def retreat_from_shelf(robot: Robot) -> None:
    print(f"[NAV] Retreat from shelf {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_backward(
        distance=APPROACH_SHELF_DIST,
        velocity=APPROACH_VELOCITY,
        tolerance=POS_TOLERANCE_MM,
        blocking=True,
    )


def drive_to_slot(robot: Robot, from_slot: str | None, to_slot: str) -> None:
    from_dist = INGREDIENT_SLOTS.get(from_slot, 0.0) if from_slot else 0.0
    to_dist = INGREDIENT_SLOTS[to_slot]
    delta = to_dist - from_dist

    if abs(delta) < 1.0:
        return

    if delta > 0:
        print(f"[NAV] Drive fwd {abs(delta):.0f} mm ({from_slot} → {to_slot})")
        robot.move_forward(
            distance=abs(delta),
            velocity=DRIVE_VELOCITY,
            tolerance=POS_TOLERANCE_MM,
            blocking=True,
        )
    else:
        print(f"[NAV] Drive bwd {abs(delta):.0f} mm ({from_slot} → {to_slot})")
        robot.move_backward(
            distance=abs(delta),
            velocity=DRIVE_VELOCITY,
            tolerance=POS_TOLERANCE_MM,
            blocking=True,
        )


# ===========================================================================
# VISION HELPERS
# ===========================================================================

def find_traffic_light_color(robot: Robot):
    """Return highest-confidence red/green traffic-light color, or None."""
    if not ENABLE_VISION:
        return None

    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best_color = None
    best_confidence = -1.0

    for detection in robot.get_detections("traffic light"):
        confidence = float(detection.get("confidence", 0.0))
        if confidence < MIN_TRAFFIC_CONFIDENCE:
            continue

        attributes = detection.get("attributes", {})
        color_attribute = attributes.get("color", {})
        color = color_attribute.get("value")

        if color not in ("red", "green"):
            continue

        if confidence > best_confidence:
            best_confidence = confidence
            best_color = str(color)

    return best_color


def stop_sign_detected(robot: Robot) -> bool:
    if not ENABLE_STOP_SIGN_OVERRIDE or not ENABLE_VISION:
        return False
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False
    return bool(robot.get_detections("stop sign"))


# ===========================================================================
# COURSE HELPERS
# ===========================================================================

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


def get_best_pose(robot: Robot) -> tuple[str, float, float, float]:
    if ENABLE_GPS and robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        return "fused", x, y, theta

    x, y, theta = robot.get_odometry_pose()
    return "odom ", x, y, theta


def cancel_motion(robot: Robot, handle) -> None:
    if handle is not None:
        handle.cancel()
        handle.wait(timeout=1.0)
    robot.stop()


def start_pure_pursuit_stage(robot: Robot, stage: dict[str, Any]):
    waypoints = stage["waypoints"]
    print(
        f"[FSM] MOVING — pure pursuit stage with {len(waypoints)} waypoints "
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
    cfg = resolve_lapf_config()
    goal_x, goal_y = stage["waypoint"]

    print(f"[FSM] MOVING — LAPF obstacle waypoint goal=({goal_x:.0f}, {goal_y:.0f}) mm")

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
    stage = COURSE_STAGES[stage_index]

    if stage["type"] == "pure_pursuit":
        return start_pure_pursuit_stage(robot, stage)

    if stage["type"] == "lapf":
        return start_lapf_stage(robot, stage)

    raise ValueError(f"Unknown course stage type: {stage['type']}")


def print_course_status(robot: Robot, stage_index: int) -> None:
    stage = COURSE_STAGES[stage_index]
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
        vt_summary = f" vt=({virtual_target[0]:6.0f}, {virtual_target[1]:6.0f}) mm"

    if obstacle_tracks:
        nearest_boundary_mm = min(
            max(
                0.0,
                ((float(track["x"]) - x) ** 2 + (float(track["y"]) - y) ** 2) ** 0.5
                - float(track["radius"]),
            )
            for track in obstacle_tracks
        )
        track_summary = f" tracked={len(obstacle_tracks)} nearest_track={nearest_boundary_mm:.0f} mm"
    else:
        track_summary = " tracked=0"

    gps_summary = ""
    if ENABLE_GPS:
        gps_summary = f" gps={'fresh' if robot.is_gps_active() else 'stale'}"

    print(
        f"  course stage {stage_index + 1}/{len(COURSE_STAGES)} {stage['name']} "
        f"goal=({goal_x:.0f}, {goal_y:.0f}) mm | "
        f"{label}=({x:6.0f}, {y:6.0f}) mm θ={theta:5.1f}°"
        f"{gps_summary}{vt_summary}{track_summary}"
    )


def print_config(robot: Robot) -> None:
    lapf_cfg = resolve_lapf_config()

    print("[CFG] Traffic-light start:")
    print(
        f"      pre_move={TRAFFIC_LIGHT_PRE_MOVE_MM:.0f} mm, "
        f"look_angle={TRAFFIC_LIGHT_LOOK_ANGLE_DEG:.1f}°, "
        f"return_angle={TRAFFIC_LIGHT_RETURN_ANGLE_DEG:.1f}°, "
        f"tolerance={TRAFFIC_TURN_TOLERANCE_DEG:.1f}°"
    )

    print("[CFG] Burger pickup:")
    print(
        f"      dist_to_ingredients={DIST_TO_INGREDIENT_AREA:.0f} mm, "
        f"slots={INGREDIENT_SLOTS}, fetch_order={FETCH_ORDER}"
    )

    print("[CFG] Pure pursuit control points:")
    for i, waypoint in enumerate(PURE_PURSUIT_CONTROL_POINTS, start=1):
        print(f"      {i:02d}: ({waypoint[0]:.0f}, {waypoint[1]:.0f}) mm")

    print("[CFG] LAPF obstacle-course control points:")
    for i, waypoint in enumerate(LAPF_CONTROL_POINTS, start=1):
        print(f"      {i:02d}: ({waypoint[0]:.0f}, {waypoint[1]:.0f}) mm")

    print(
        f"[CFG] pure_pursuit velocity={PURE_PURSUIT_VELOCITY_MM_S:.0f} mm/s "
        f"lookahead={LOOKAHEAD_MM:.0f} mm tolerance={PURE_PURSUIT_TOLERANCE_MM:.0f} mm"
    )
    print(
        f"[CFG] LAPF velocity={LAPF_VELOCITY_MM_S:.0f} mm/s "
        f"tolerance={LAPF_TOLERANCE_MM:.0f} mm max_angular={LAPF_MAX_ANGULAR_RAD_S:.2f} rad/s"
    )
    print(
        f"[CFG] LAPF leash={lapf_cfg['leash_length_mm']:.0f} mm "
        f"half_angle={lapf_cfg['leash_half_angle_deg']:.0f}° "
        f"target_speed={lapf_cfg['target_speed_mm_s']:.0f} mm/s "
        f"repulsion_range={lapf_cfg['repulsion_range_mm']:.0f} mm "
        f"repulsion_gain={lapf_cfg['repulsion_gain']:.0f} "
        f"attraction_gain={lapf_cfg['attraction_gain']:.2f} "
        f"force_ema_alpha={lapf_cfg['force_ema_alpha']:.2f} "
        f"inflation={lapf_cfg['inflation_margin_mm']:.0f} mm"
    )

    if ENABLE_GPS:
        print(
            f"[CFG] gps tag_id={TAG_ID} tag_body=({TAG_BODY_OFFSET_X_MM:.0f}, "
            f"{TAG_BODY_OFFSET_Y_MM:.0f}) mm position_alpha={GPS_POSITION_ALPHA:.2f}"
        )


# ===========================================================================
# BURGER BUILD SEQUENCE
# ===========================================================================

def run_burger_build_sequence(robot: Robot) -> bool:
    """Build and grab the burger, then return control to the main mission."""

    print()
    print("=" * 64)
    print("  BURGER BUILD — building burger before ramp/course navigation")
    print("=" * 64)

    state = "PREP_INITIAL_MOVE"
    current_slot = None
    pick_attempts = 0
    active_drop_ticks = 0

    action_sub_state = "INIT"
    action_timer = 0.0
    next_fsm_state = ""

    manual_skip_triggered = False

    internal_target_ticks = 0
    requested_final_ticks = 0
    last_step_time = 0.0
    step_delay_s = 0.150

    requested_claw_deg = CLAW_CLOSE_BUN_DEG
    current_claw_deg = CLAW_CLOSE_BUN_DEG
    active_close_deg = CLAW_CLOSE_BUN_DEG

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        # ── PREP MOVE ──────────────────────────────────────────────────────
        if state == "PREP_INITIAL_MOVE":
            print("[ARM] Opening claw for travel heading...")
            robot.enable_servo(CLAW_SERVO)
            requested_claw_deg = CLAW_OPEN_DEG
            state = "WAIT_PREP_INITIAL_CLAW"

        elif state == "WAIT_PREP_INITIAL_CLAW":
            if abs(current_claw_deg - requested_claw_deg) <= 0.5:
                print("[ARM] Claw opened. Raising lift to carry height...")
                robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                requested_final_ticks = LIFT_CARRY_TICKS
                action_timer = time.monotonic()
                state = "WAIT_PREP_INITIAL_LIFT"

        elif state == "WAIT_PREP_INITIAL_LIFT":
            current = get_lift_ticks(robot)
            if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or (
                time.monotonic() - action_timer > LIFT_TIMEOUT_S
            ):
                print("[ARM] Arm prepped at carry height. Moving out.")
                state = "BURGER_PICKUP"

        # ── NAVIGATION TO INGREDIENTS ──────────────────────────────────────
        elif state == "BURGER_PICKUP":
            led_moving(robot)
            print(f"[NAV] Driving to ingredient area: {DIST_TO_INGREDIENT_AREA:.0f} mm")
            robot.move_forward(
                distance=DIST_TO_INGREDIENT_AREA,
                velocity=DRIVE_VELOCITY,
                tolerance=POS_TOLERANCE_MM,
                blocking=True,
            )
            robot.stop()
            current_slot = None
            pick_attempts = 0
            state = "FETCH_MEAT"

        elif state == "FETCH_MEAT":
            target = FETCH_ORDER[0]
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
            action_sub_state = "LIFT_DOWN"
            next_fsm_state = "FETCH_TOP_BUN"
            state = "DO_PLACE"

        elif state == "FETCH_TOP_BUN":
            target = FETCH_ORDER[1]
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
            action_sub_state = "LIFT_DOWN_TO_STACK"
            next_fsm_state = "BURGER_DONE"
            state = "DO_PLACE_AND_GRAB"

        # ── PICK SEQUENCE ──────────────────────────────────────────────────
        elif state == "DO_PICK":
            if robot.was_button_pressed(Button.BTN_3):
                manual_skip_triggered = True

            if action_sub_state == "OPEN_CLAW":
                robot.enable_servo(CLAW_SERVO)
                requested_claw_deg = CLAW_OPEN_DEG
                action_sub_state = "WAIT_OPEN"

            elif action_sub_state == "WAIT_OPEN":
                if abs(current_claw_deg - requested_claw_deg) <= 0.5:
                    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                    requested_final_ticks = LIFT_PICKUP_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_DOWN"

            elif action_sub_state == "WAIT_LIFT_DOWN":
                current = get_lift_ticks(robot)
                elapsed = time.monotonic() - action_timer

                if abs(current - LIFT_PICKUP_TICKS) <= LIFT_TOLERANCE or elapsed > LIFT_TIMEOUT_S:
                    print(f"[ARM] Reached pickup height at: {current} ticks")
                    print(f"[ARM] Closing claw to {active_close_deg} degrees...")
                    requested_claw_deg = active_close_deg
                    action_sub_state = "WAIT_CLOSE"

            elif action_sub_state == "WAIT_CLOSE":
                if abs(current_claw_deg - requested_claw_deg) <= 0.5:
                    print("[ARM] Claw closed. Lifting to carry height...")
                    requested_final_ticks = LIFT_CARRY_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_UP"

            elif action_sub_state == "WAIT_LIFT_UP":
                current = get_lift_ticks(robot)
                elapsed = time.monotonic() - action_timer

                if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or elapsed > LIFT_TIMEOUT_S:
                    grabbed = claw_has_object(robot) or manual_skip_triggered
                    pick_attempts += 1

                    if grabbed or pick_attempts >= MAX_PICK_ATTEMPTS:
                        retreat_from_shelf(robot)
                        turn_away_from_shelf(robot)
                        pick_attempts = 0
                        manual_skip_triggered = False
                        state = next_fsm_state
                    else:
                        action_sub_state = "OPEN_CLAW"

        # ── PLACE SEQUENCE ─────────────────────────────────────────────────
        elif state == "DO_PLACE":
            if action_sub_state == "LIFT_DOWN":
                requested_final_ticks = active_drop_ticks
                action_timer = time.monotonic()
                action_sub_state = "WAIT_LIFT_DOWN"

            elif action_sub_state == "WAIT_LIFT_DOWN":
                current = get_lift_ticks(robot)
                elapsed = time.monotonic() - action_timer

                if abs(current - active_drop_ticks) <= LIFT_TOLERANCE or elapsed > LIFT_TIMEOUT_S:
                    print(f"[ARM] Lift stopped at dynamic dropoff: {current} ticks")
                    print(f"[ARM] Opening claw to {CLAW_OPEN_DEG} degrees...")
                    requested_claw_deg = CLAW_OPEN_DEG
                    action_sub_state = "WAIT_OPEN"

            elif action_sub_state == "WAIT_OPEN":
                if abs(current_claw_deg - requested_claw_deg) <= 0.5:
                    print("[ARM] Claw is open. Lifting safely to carry height FIRST...")
                    requested_final_ticks = LIFT_CARRY_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_UP"

            elif action_sub_state == "WAIT_LIFT_UP":
                current = get_lift_ticks(robot)
                elapsed = time.monotonic() - action_timer

                if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or elapsed > LIFT_TIMEOUT_S:
                    print(f"[ARM] Lift clear of burger at: {current} ticks. Leaving claw open.")
                    retreat_from_shelf(robot)
                    turn_away_from_shelf(robot)
                    pick_attempts = 0
                    state = next_fsm_state

        # ── PLACE THIRD PIECE AND GRAB STACK ───────────────────────────────
        elif state == "DO_PLACE_AND_GRAB":
            if action_sub_state == "LIFT_DOWN_TO_STACK":
                requested_final_ticks = active_drop_ticks
                action_timer = time.monotonic()
                action_sub_state = "WAIT_LIFT_DOWN_STACK"

            elif action_sub_state == "WAIT_LIFT_DOWN_STACK":
                current = get_lift_ticks(robot)
                if abs(current - active_drop_ticks) <= LIFT_TOLERANCE or (
                    time.monotonic() - action_timer > LIFT_TIMEOUT_S
                ):
                    print("[ARM] Reached stack height. Dropping top bun...")
                    requested_claw_deg = CLAW_OPEN_DEG
                    action_sub_state = "WAIT_OPEN"

            elif action_sub_state == "WAIT_OPEN":
                if abs(current_claw_deg - requested_claw_deg) <= 0.5:
                    print("[ARM] Dropped bun. Lowering to base to grab entire burger...")
                    requested_final_ticks = LIFT_PICKUP_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_DOWN_BASE"

            elif action_sub_state == "WAIT_LIFT_DOWN_BASE":
                current = get_lift_ticks(robot)
                if abs(current - LIFT_PICKUP_TICKS) <= LIFT_TOLERANCE or (
                    time.monotonic() - action_timer > LIFT_TIMEOUT_S
                ):
                    print("[ARM] At base height. Grabbing entire burger...")
                    requested_claw_deg = CLAW_CLOSE_BUN_DEG
                    action_sub_state = "WAIT_CLOSE"

            elif action_sub_state == "WAIT_CLOSE":
                if abs(current_claw_deg - requested_claw_deg) <= 0.5:
                    print("[ARM] Grabbed entire burger. Lifting to carry height...")
                    requested_final_ticks = LIFT_CARRY_TICKS
                    action_timer = time.monotonic()
                    action_sub_state = "WAIT_LIFT_UP"

            elif action_sub_state == "WAIT_LIFT_UP":
                current = get_lift_ticks(robot)
                if abs(current - LIFT_CARRY_TICKS) <= LIFT_TOLERANCE or (
                    time.monotonic() - action_timer > LIFT_TIMEOUT_S
                ):
                    print("[ARM] Secured all pieces at carry height.")
                    retreat_from_shelf(robot)
                    turn_away_from_shelf(robot)
                    state = next_fsm_state

        # ── DONE / CANCEL ──────────────────────────────────────────────────
        elif state == "BURGER_DONE":
            led_hold(robot)
            robot.stop()
            print("[FSM] BURGER_DONE — burger secured, proceeding to ramp/course navigation")
            return True

        # Global burger-section estop.
        if robot.was_button_pressed(Button.BTN_2):
            robot.stop()
            led_error(robot)
            lift_return_to_zero(robot)
            robot.estop()
            return False

        internal_target_ticks, last_step_time, current_claw_deg = update_arm_targets(
            robot=robot,
            requested_final_ticks=requested_final_ticks,
            internal_target_ticks=internal_target_ticks,
            last_step_time=last_step_time,
            requested_claw_deg=requested_claw_deg,
            current_claw_deg=current_claw_deg,
            step_delay_s=step_delay_s,
        )

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()


# ===========================================================================
# MAIN RUN FSM
# ===========================================================================

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)
    reset_pose(robot)

    state = "INIT"
    course_stage_index = 0
    motion_handle = None
    last_status_print_at = 0.0

    requested_final_ticks = 0
    internal_target_ticks = 0
    last_step_time = 0.0
    requested_claw_deg = CLAW_CLOSE_BUN_DEG
    current_claw_deg = CLAW_CLOSE_BUN_DEG

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print()
    print("=" * 72)
    print("  FULL MISSION — traffic light → burger → ramp/course/LAPF")
    print("=" * 72)

    while True:
        now = time.monotonic()

        # ── INIT / LIFT ALIGNMENT ──────────────────────────────────────────
        if state == "INIT":
            robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
            claw_close(robot, CLAW_CLOSE_BUN_DEG)
            requested_claw_deg = CLAW_CLOSE_BUN_DEG
            current_claw_deg = CLAW_CLOSE_BUN_DEG
            led_idle(robot)
            print_config(robot)
            print("  LIFT ALIGNMENT — align carriage to Sharpie mark")
            print("  BTN_1: UP | BTN_2: DOWN | BTN_10: Confirm")
            state = "INIT_JOG"

        elif state == "INIT_JOG":
            if robot.was_button_pressed(Button.BTN_1):
                requested_final_ticks = requested_final_ticks + LIFT_JOG_STEP
                robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                print(f"[JOG] Stepping target UP to: {requested_final_ticks}")

            elif robot.was_button_pressed(Button.BTN_2):
                requested_final_ticks = requested_final_ticks - LIFT_JOG_STEP
                robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                print(f"[JOG] Stepping target DOWN to: {requested_final_ticks}")

            elif robot.was_button_pressed(Button.BTN_10):
                robot.reset_motor_position(LIFT_MOTOR)
                requested_final_ticks = 0
                internal_target_ticks = 0
                time.sleep(0.15)
                print("[INIT] Encoder zeroed -> MOVE_BEFORE_LIGHT")
                dim_all_leds(robot)
                led_moving(robot)

                print(
                    f"[FSM] MOVE_BEFORE_LIGHT — moving forward "
                    f"{TRAFFIC_LIGHT_PRE_MOVE_MM:.0f} mm before looking for traffic light"
                )
                motion_handle = robot.move_forward(
                    distance=TRAFFIC_LIGHT_PRE_MOVE_MM,
                    velocity=TRAFFIC_LIGHT_PRE_MOVE_VELOCITY_MM_S,
                    tolerance=TRAFFIC_LIGHT_PRE_MOVE_TOLERANCE_MM,
                    blocking=False,
                )
                last_status_print_at = now
                state = "MOVE_BEFORE_LIGHT"

        # ── PRE-MOVE BEFORE TRAFFIC LIGHT ──────────────────────────────────
        elif state == "MOVE_BEFORE_LIGHT":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                led_idle(robot)
                print("[FSM] INIT_JOG — mission cancelled before traffic-light turn")
                state = "INIT_JOG"

            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    _, _, theta = robot.get_odometry_pose()
                    print(
                        f"  moving before traffic light: target={TRAFFIC_LIGHT_PRE_MOVE_MM:.0f} mm "
                        f"θ={theta:.1f}°"
                    )
                    last_status_print_at = now

                if motion_handle is not None and motion_handle.is_finished():
                    robot.stop()
                    motion_handle = None
                    print(
                        f"[FSM] TURN_TO_LIGHT — pre-move complete, turning to absolute heading "
                        f"{TRAFFIC_LIGHT_LOOK_ANGLE_DEG:.1f}°"
                    )
                    motion_handle = robot.turn_to(
                        TRAFFIC_LIGHT_LOOK_ANGLE_DEG,
                        blocking=False,
                        tolerance_deg=TRAFFIC_TURN_TOLERANCE_DEG,
                    )
                    last_status_print_at = time.monotonic()
                    state = "TURN_TO_LIGHT"

        # ── TRAFFIC LIGHT ──────────────────────────────────────────────────
        elif state == "TURN_TO_LIGHT":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                led_idle(robot)
                print("[FSM] INIT_JOG — mission cancelled while turning to traffic light")
                state = "INIT_JOG"

            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    _, _, theta = robot.get_odometry_pose()
                    print(
                        f"  turning to traffic light: θ={theta:.1f}° "
                        f"target={TRAFFIC_LIGHT_LOOK_ANGLE_DEG:.1f}°"
                    )
                    last_status_print_at = now

                if motion_handle is not None and motion_handle.is_finished():
                    robot.stop()
                    motion_handle = None
                    print("[FSM] WAIT_GREEN — traffic-light viewing angle reached")
                    state = "WAIT_GREEN"

        elif state == "WAIT_GREEN":
            robot.stop()

            if robot.was_button_pressed(Button.BTN_2):
                led_idle(robot)
                print("[FSM] INIT_JOG — mission cancelled while waiting for green")
                state = "INIT_JOG"

            elif stop_sign_detected(robot):
                robot.set_led(LED.RED, LED_BRIGHTNESS, mode=LEDMode.BLINK, period_ms=500)
                robot.set_led(LED.GREEN, 0)
                print("[VISION] stop sign detected — stopped")

            else:
                traffic_light_color = find_traffic_light_color(robot)

                if traffic_light_color == "green" or robot.was_button_pressed(Button.BTN_5):
                    show_traffic_light_color(robot, "green")
                    print(
                        f"[VISION] green light — turning back to absolute heading "
                        f"{TRAFFIC_LIGHT_RETURN_ANGLE_DEG:.1f}°"
                    )
                    motion_handle = robot.turn_to(
                        TRAFFIC_LIGHT_RETURN_ANGLE_DEG,
                        blocking=False,
                        tolerance_deg=TRAFFIC_TURN_TOLERANCE_DEG,
                    )
                    last_status_print_at = now
                    state = "RETURN_TO_FORWARD"

                elif traffic_light_color == "red":
                    show_traffic_light_color(robot, "red")

        elif state == "RETURN_TO_FORWARD":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                led_idle(robot)
                print("[FSM] INIT_JOG — mission cancelled while returning to forward")
                state = "INIT_JOG"

            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    _, _, theta = robot.get_odometry_pose()
                    print(
                        f"  returning to forward: θ={theta:.1f}° "
                        f"target={TRAFFIC_LIGHT_RETURN_ANGLE_DEG:.1f}°"
                    )
                    last_status_print_at = now

                if motion_handle is not None and motion_handle.is_finished():
                    robot.stop()
                    motion_handle = None
                    print("[FSM] Forward heading restored — starting burger build")
                    state = "BURGER_BUILD"

        # ── BURGER BUILD ───────────────────────────────────────────────────
        elif state == "BURGER_BUILD":
            ok = run_burger_build_sequence(robot)
            if not ok:
                print("[FSM] Burger build failed or was cancelled")
                break

            print("[FSM] Burger complete — resetting odometry for ramp/course")
            reset_pose(robot)
            course_stage_index = 0
            motion_handle = start_course_stage(robot, course_stage_index)
            last_status_print_at = time.monotonic()
            state = "COURSE_MOVING"

        # ── COURSE NAVIGATION ──────────────────────────────────────────────
        elif state == "COURSE_MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                led_error(robot)
                print("[FSM] Course mission cancelled")
                lift_return_to_zero(robot)
                robot.estop()
                break

            elif ENABLE_STOP_SIGN_OVERRIDE and stop_sign_detected(robot):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                robot.set_led(LED.RED, LED_BRIGHTNESS, mode=LEDMode.BLINK, period_ms=500)
                robot.set_led(LED.GREEN, 0)
                print("[VISION] stop sign detected during course — mission stopped")
                state = "DONE"

            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_course_status(robot, course_stage_index)
                    last_status_print_at = now

                if motion_handle is not None and motion_handle.is_finished():
                    print(
                        f"[FSM] DONE — course stage "
                        f"{course_stage_index + 1}/{len(COURSE_STAGES)} complete"
                    )
                    print_course_status(robot, course_stage_index)

                    robot.stop()
                    time.sleep(STAGE_PAUSE_S)

                    course_stage_index += 1
                    if course_stage_index >= len(COURSE_STAGES):
                        motion_handle = None
                        print("[FSM] DONE — full mission complete")
                        state = "DONE"
                    else:
                        motion_handle = start_course_stage(robot, course_stage_index)
                        last_status_print_at = time.monotonic()

        elif state == "DONE":
            robot.stop()
            led_done(robot)
            # Keep holding the burger; press BTN_5 to return lift to zero and end.
            if robot.was_button_pressed(Button.BTN_5):
                lift_return_to_zero(robot)
                break

        # ── ASYNCHRONOUS INIT-JOG ARM UPDATER ──────────────────────────────
        if state == "INIT_JOG":
            internal_target_ticks, last_step_time, current_claw_deg = update_arm_targets(
                robot=robot,
                requested_final_ticks=requested_final_ticks,
                internal_target_ticks=internal_target_ticks,
                last_step_time=last_step_time,
                requested_claw_deg=requested_claw_deg,
                current_claw_deg=current_claw_deg,
                step_delay_s=0.150,
            )

        # ── Tick-rate control ──────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
