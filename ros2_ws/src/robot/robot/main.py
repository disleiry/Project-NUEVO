"""
main_delivery.py — Section 2: Face Detection, Burger Delivery & Stop-Sign Finish
=================================================================================
Starts where the obstacle course ends (~arena 1800, 3700), robot facing +Y.
Odometry is reset on BTN_1 press. Lift is assumed to already be at carry height
with the burger secured from Section 1.

Mission sequence
────────────────
 1. Turn right to face +X  (theta 90° → 0°)
 2. Drive DIST_TO_SCAN_MM forward in +X
 3. Stop — detect face → Customer A (Female) or Customer B (Male)
 4. Turn right to face -Y  (theta 0° → -90°)
 5. Drive straight to customer row (distance depends on A or B)
 6. Drop off burger (turn to shelf, approach, lower, open claw, lift, retreat, turn back)
 7. Drive straight in -Y watching for stop sign (pure pursuit, non-blocking)
 8. Stop sign seen → halt 2 s → advance 610 mm → done

Controls
────────
  BTN_1 = start mission
  BTN_2 = cancel (checked in WATCH_STOP)
"""
import sys
import time

import cv2

sys.path.insert(0, '/ros2_ws/src/vision')
from vision.gender_detection import GenderDetector

from robot.hardware_map import (
    Button, DCMotorMode, Motor, ServoChannel, LED, LEDMode,
    POSITION_UNIT, WHEEL_DIAMETER, WHEEL_BASE, INITIAL_THETA_DEG,
    LEFT_WHEEL_MOTOR, LEFT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR, RIGHT_WHEEL_DIR_INVERTED,
    LIDAR_FOV_DEG, LIDAR_MOUNT_THETA_DEG, LIDAR_MOUNT_X_MM, LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM, LIDAR_RANGE_MIN_MM,
    TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM,
    DEFAULT_FSM_HZ,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline

# ==========================================
# --- ARM CONSTANTS (match Section 1) ---
# ==========================================
LIFT_MOTOR        = Motor.DC_M3
LIFT_CARRY_TICKS  = -12800   # travel height — burger already here on entry
LIFT_PICKUP_TICKS = -8300    # shelf height — same level used for drop-off
LIFT_MAX_VEL      = 1800     # ticks/s
LIFT_TOLERANCE    = 30       # ticks

CLAW_SERVO      = ServoChannel.CH_13
CLAW_OPEN_DEG   = 60.0
CLAW_CLOSE_DEG  = 146.0

# ==========================================
# --- DRIVE CONSTANTS ---
# ==========================================
DRIVE_VELOCITY    = 100.0   # mm/s
APPROACH_VELOCITY =  60.0   # mm/s
POS_TOLERANCE_MM  =  20.0   # mm
TURN_TOLERANCE_DEG =  2.0   # deg
TURN_VELOCITY_DEG  = 45.0   # deg/s

# Shelf approach — robot faces shelf, same values as Section 1
# Robot travels in -Y; shelf is to the left (+X side) → positive = left turn
TURN_TO_SHELF_DEG   =  79.0
TURN_FROM_SHELF_DEG = -79.0
APPROACH_SHELF_DIST =  18.0  # mm
ANGULAR_VELOCITY_DEG = 20

# ==========================================
# --- SENSOR SETUP ---
# ==========================================
ENABLE_VISION = True
ENABLE_LIDAR  = True
ENABLE_GPS    = True

TAG_ID                          = 25
GPS_POSITION_ALPHA              = 0.20
ENABLE_GPS_TANGENT_HEADING      = True
GPS_TANGENT_ALPHA               = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0

# ==========================================
# --- NAVIGATION TUNING ---
# ==========================================
PURE_PURSUIT_VELOCITY_MM_S   = 150.0
LOOKAHEAD_MM                 = 225.0
PURE_PURSUIT_TOLERANCE_MM    =  25.0
ADVANCE_RADIUS_MM            =  75.0
PURE_PURSUIT_MAX_ANGULAR_RAD_S = 1.5

STATUS_PRINT_INTERVAL_S = 0.5

# ==========================================
# --- MISSION GEOMETRY  (tune on arena) ---
# ==========================================
# Distance driven in +X before face detection scan
# Arena ref: x 1800 → 2200 mm
DIST_TO_SCAN_MM = 400.0

# Distance driven in -Y from the scan point to the customer row (100 mm buffer)
# Customer A (Female): y 3700 → stop at y~1800 (shelf at y=1700)
# Customer B (Male):   y 3700 → stop at y~1400 (shelf at y=1300)
CUSTOMER_A_TRAVEL_MM = 1900.0   # *** tune on arena ***
CUSTOMER_B_TRAVEL_MM = 2300.0   # *** tune on arena ***

# Stop-sign leg: distance in -Y from customer drop-off point to stop-sign zone
# A: from y~1800 to stop sign y~200  →  ~1800 mm incl. buffer
# B: from y~1400 to stop sign y~200  →  ~1400 mm incl. buffer
CUSTOMER_A_TO_STOP_MM = 1800.0  # *** tune on arena ***
CUSTOMER_B_TO_STOP_MM = 1400.0  # *** tune on arena ***

STOP_SIGN_PAUSE_S  = 2.0    # seconds to dwell at stop sign
FINAL_ADVANCE_MM   = 610.0  # mm to advance after the pause
MIN_STOP_CONF      = 0.50   # vision confidence threshold

# ==========================================
# --- FACE DETECTION ---
# ==========================================
CAMERA_DEVICE      = '/dev/video10'
FACE_SAMPLE_FRAMES = 15     # frames per voting pass
FACE_FRAME_DELAY_S = 0.10   # seconds between frames
FACE_MAX_ATTEMPTS  =  3     # voting passes before defaulting

# ==========================================
# --- HELPERS ---
# ==========================================
def configure_and_start_robot(robot: Robot) -> None:
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
    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
        if ENABLE_GPS_TANGENT_HEADING:
            robot.enable_gps_tangent_heading(
                alpha=GPS_TANGENT_ALPHA,
                min_displacement_mm=GPS_TANGENT_MIN_DISPLACEMENT_MM,
            )
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)


def get_best_pose(robot: Robot) -> tuple[str, float, float, float]:
    if ENABLE_GPS and robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        return "fused", x, y, theta
    x, y, theta = robot.get_odometry_pose()
    return "odom ", x, y, theta


def move_lift(robot: Robot, target_ticks: int) -> None:
    """Blocking lift move with spoon-feeder to avoid overcurrent."""
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    current_ticks = float(get_lift_ticks(robot))

    step_rate_hz   = 50.0
    delay_s        = 1.0 / step_rate_hz
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
            blocking=False,
        )
        time.sleep(delay_s)

    robot.set_motor_position(
        LIFT_MOTOR, int(target_ticks),
        max_vel_ticks=LIFT_MAX_VEL,
        tolerance_ticks=LIFT_TOLERANCE,
        blocking=True, timeout=2.0,
    )


def get_lift_ticks(robot: Robot) -> int:
    dc = robot.get_dc_state()
    if dc is None:
        return 0
    return int(dc.motors[LIFT_MOTOR - 1].position)


def claw_open(robot: Robot) -> None:
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_OPEN_DEG)
    time.sleep(0.5)


def claw_close(robot: Robot) -> None:
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_CLOSE_DEG)
    time.sleep(0.5)


def stop_sign_detected(robot: Robot) -> bool:
    if not ENABLE_VISION:
        return False
    if not robot.is_vision_active(timeout_s=3.0):
        return False
    for det in robot.get_detections("stop sign"):
        if float(det.get("confidence", 0)) >= MIN_STOP_CONF:
            return True
    return False


def detect_customer(detector: GenderDetector, cap: cv2.VideoCapture) -> str:
    """
    Votes over FACE_SAMPLE_FRAMES frames, repeated up to FACE_MAX_ATTEMPTS times.

    Rule (per spec):
      • 100% of votes are Male  →  Customer B
      • Any Female vote, or no face seen  →  Customer A  (Female / uncertain)

    Returns 'A' or 'B'.
    """
    for attempt in range(1, FACE_MAX_ATTEMPTS + 1):
        votes = {"Male": 0, "Female": 0}
        for _ in range(FACE_SAMPLE_FRAMES):
            ret, frame = cap.read()
            if not ret:
                continue
            gender, _ = detector.detect(frame)
            if gender in votes:
                votes[gender] += 1
            time.sleep(FACE_FRAME_DELAY_S)

        male_v, female_v = votes["Male"], votes["Female"]
        print(f"[FACE] Attempt {attempt}/{FACE_MAX_ATTEMPTS} — Male:{male_v}  Female:{female_v}")

        if male_v == 0 and female_v == 0:
            print("[FACE] No face detected — retrying...")
            continue

        if female_v == 0:
            print("[FACE] 100% Male → Customer B")
            return "B"
        else:
            print(f"[FACE] Not 100% Male → Customer A")
            return "A"

    print("[FACE] No face after all attempts — defaulting to Customer A")
    return "A"


def do_dropoff(robot: Robot) -> None:
    """
    Blocking drop-off sequence. Robot must already be stopped alongside the shelf,
    heading in -Y. Mirrors the pick sequence from Section 1.

      turn left to face shelf → approach → lower to shelf height →
      open claw → lift clear → retreat → turn right back to -Y
    """
    print("[DROPOFF] Turning to face shelf")
    robot.turn_by(TURN_TO_SHELF_DEG, ANGULAR_VELOCITY_DEG, TURN_VELOCITY_DEG,
                  tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

    print(f"[DROPOFF] Approaching {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_forward(APPROACH_SHELF_DIST, APPROACH_VELOCITY,
                       POS_TOLERANCE_MM, blocking=True)
    robot.stop()

    print(f"[DROPOFF] Lowering to shelf height ({LIFT_PICKUP_TICKS} ticks)")
    move_lift(robot, LIFT_PICKUP_TICKS)

    print("[DROPOFF] Opening claw — releasing burger")
    claw_open(robot)
    time.sleep(0.5)   # let burger settle on shelf

    print(f"[DROPOFF] Lifting clear to carry height ({LIFT_CARRY_TICKS} ticks)")
    move_lift(robot, LIFT_CARRY_TICKS)

    print(f"[DROPOFF] Retreating {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_backward(APPROACH_SHELF_DIST, APPROACH_VELOCITY,
                        POS_TOLERANCE_MM, blocking=True)

    print("[DROPOFF] Turning back to -Y heading")
    robot.turn_by(TURN_FROM_SHELF_DEG, ANGULAR_VELOCITY_DEG, TURN_VELOCITY_DEG,
                  tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
    robot.stop()
    print("[DROPOFF] Complete")


def led_moving(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def led_idle(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


# ==========================================
# --- STATE MACHINE CONTROL LOOP ---
# ==========================================
def run(robot: Robot) -> None:
    print("[INIT] Configuring sensors and odometry...")
    configure_and_start_robot(robot)

    # Open camera once for the whole mission
    detector = GenderDetector()
    cap = cv2.VideoCapture(CAMERA_DEVICE)
    if not cap.isOpened():
        print(f"[warn] Camera {CAMERA_DEVICE} unavailable — will default to Customer A")

    state = "INIT"
    customer = "A"
    motion_handle = None
    last_status_print_at = 0.0

    while True:

        # ------------------------------------------------------------------
        if state == "INIT":
            led_idle(robot)
            robot.stop()
            print("=== DELIVERY MISSION — Section 2 ===")
            print("Burger should already be secured at carry height.")
            print("Press BTN_1 to start.")
            state = "IDLE"

        # ------------------------------------------------------------------
        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                configure_and_start_robot(robot)   # reset odometry fresh
                led_moving(robot)
                state = "DELIVERY"
            time.sleep(0.05)

        # ------------------------------------------------------------------
        elif state == "DELIVERY":
            # ── 1. Turn right to face +X ──────────────────────────────────
            print("[NAV] Turning right to face +X (theta → 0°)")
            robot.turn_to(11.0, ANGULAR_VELOCITY_DEG, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

            # ── 2. Drive to face-detection zone ───────────────────────────
            print(f"[NAV] Driving {DIST_TO_SCAN_MM:.0f} mm in +X to scan zone")
            robot.move_forward(DIST_TO_SCAN_MM, DRIVE_VELOCITY,
                               POS_TOLERANCE_MM, blocking=True)
            robot.stop()
            robot.turn_by(8, ANGULAR_VELOCITY_DEG, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
            # ── 3. Face detection ──────────────────────────────────────────
            print("[FACE] Starting customer detection...")
            if cap.isOpened():
                customer = detect_customer(detector, cap)
            else:
                print("[FACE] No camera — defaulting to Customer A")
                customer = "A"

            travel_mm   = CUSTOMER_A_TRAVEL_MM   if customer == "A" else CUSTOMER_B_TRAVEL_MM
            to_stop_mm  = CUSTOMER_A_TO_STOP_MM  if customer == "A" else CUSTOMER_B_TO_STOP_MM
            print(f"[FACE] Customer {customer} ({'Female' if customer == 'A' else 'Male'}) "
                  f"— travel {travel_mm:.0f} mm to customer row")

            # ── 4. Turn right to face -Y ───────────────────────────────────
            robot.turn_by(-8, ANGULAR_VELOCITY_DEG, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
            robot.move_forward(300, DRIVE_VELOCITY,
                               POS_TOLERANCE_MM, blocking=True)
            print("[NAV] Turning right to face -Y (theta → -90°)")
            robot.turn_to(-60.0, ANGULAR_VELOCITY_DEG, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

            # ── 5. Drive to customer row ───────────────────────────────────
            # GPS fuses while tag is visible; gracefully falls back to odometry
            # once the tag leaves the camera FOV — no code change needed.
            print(f"[NAV] Driving {travel_mm:.0f} mm in -Y to Customer {customer}")
            robot.move_forward(travel_mm, DRIVE_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            robot.stop()

            label, x, y, theta = get_best_pose(robot)
            print(f"[NAV] Arrived at customer row — {label}=({x:.0f}, {y:.0f}) θ={theta:.1f}°")

            # ── 6. Drop off burger ─────────────────────────────────────────
            print(f"[DROPOFF] Delivering to Customer {customer}")
            do_dropoff(robot)

            # ── 7. Kick off pure-pursuit toward stop sign ──────────────────
            # Build waypoints from current position straight in -Y
            _, cur_x, cur_y, _ = get_best_pose(robot)
            stop_wps = densify_polyline(
                [(cur_x, cur_y), (cur_x, cur_y - to_stop_mm)],
                spacing=100.0,
            )
            print(f"[NAV] Starting stop-sign watch leg "
                  f"— {len(stop_wps)} waypoints, {to_stop_mm:.0f} mm in -Y")
            motion_handle = robot.purepursuit_follow_path(
                waypoints=stop_wps,
                velocity=PURE_PURSUIT_VELOCITY_MM_S,
                lookahead=LOOKAHEAD_MM,
                tolerance=PURE_PURSUIT_TOLERANCE_MM,
                advance_radius=ADVANCE_RADIUS_MM,
                max_angular_rad_s=PURE_PURSUIT_MAX_ANGULAR_RAD_S,
                blocking=False,
            )
            last_status_print_at = time.monotonic()
            state = "WATCH_STOP"

        # ------------------------------------------------------------------
        elif state == "WATCH_STOP":
            now = time.monotonic()

            # Cancel override
            if robot.was_button_pressed(Button.BTN_2):
                if motion_handle is not None:
                    motion_handle.cancel()
                    motion_handle.wait(timeout=1.0)
                robot.stop()
                print("[FSM] Mission cancelled via BTN_2")
                state = "DONE"

            # Stop sign detected — halt, wait, advance, finish
            elif stop_sign_detected(robot):
                if motion_handle is not None:
                    motion_handle.cancel()
                    motion_handle.wait(timeout=1.0)
                robot.stop()
                print(f"[VISION] Stop sign detected — pausing {STOP_SIGN_PAUSE_S:.0f} s")
                robot.set_led(LED.RED, 255, mode=LEDMode.BLINK, period_ms=500)
                robot.set_led(LED.GREEN, 0)
                time.sleep(STOP_SIGN_PAUSE_S)

                print(f"[NAV] Advancing {FINAL_ADVANCE_MM:.0f} mm past stop sign")
                led_moving(robot)
                robot.move_forward(FINAL_ADVANCE_MM, DRIVE_VELOCITY,
                                   POS_TOLERANCE_MM, blocking=True)
                robot.stop()
                state = "DONE"

            else:
                # Periodic status print
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    label, x, y, theta = get_best_pose(robot)
                    print(f"  [WATCH] {label}=({x:.0f}, {y:.0f}) θ={theta:.1f}° "
                          f"| watching for stop sign...")
                    last_status_print_at = now

                # Path exhausted before stop sign — extend slowly so we keep looking
                if motion_handle is not None and motion_handle.is_finished():
                    print("[WATCH] Path exhausted — extending 500 mm")
                    _, cur_x, cur_y, _ = get_best_pose(robot)
                    ext_wps = densify_polyline(
                        [(cur_x, cur_y), (cur_x, cur_y - 500.0)],
                        spacing=100.0,
                    )
                    motion_handle = robot.purepursuit_follow_path(
                        waypoints=ext_wps,
                        velocity=APPROACH_VELOCITY,   # slow — near the sign now
                        lookahead=LOOKAHEAD_MM,
                        tolerance=PURE_PURSUIT_TOLERANCE_MM,
                        advance_radius=ADVANCE_RADIUS_MM,
                        max_angular_rad_s=PURE_PURSUIT_MAX_ANGULAR_RAD_S,
                        blocking=False,
                    )

            time.sleep(0.05)

        # ------------------------------------------------------------------
        elif state == "DONE":
            robot.stop()
            if cap.isOpened():
                cap.release()
            print("=== DELIVERY MISSION COMPLETE ===")
            led_idle(robot)
            time.sleep(1.0)

