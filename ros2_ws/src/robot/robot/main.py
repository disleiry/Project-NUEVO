"""
main_delivery.py — Section 2: Face Detection, Burger Delivery & Stop-Sign Finish
=================================================================================
Runs after the obstacle course ends at ~(1800, 3700).
Odometry is reset at mission start; all distances below are relative.

Full sequence
─────────────
 1. INIT        — configure hardware, ensure lift at carry height, claw closed.
 2. IDLE        — wait for BTN_1.
 3. TURN_TO_X   — turn right to face +X  (abs heading   0°).
 4. DRIVE_SCAN  — drive DIST_TO_SCAN_MM forward (+X) to face-detection zone.
 5. FACE_DETECT — stop; gender-vote camera logic picks Customer A or B.
 6. TURN_NEG_Y  — turn right to face -Y  (abs heading -90°) toward customers.
 7. DRIVE_CUST  — drive straight in -Y to customer row (distance varies by A/B).
 8. DROPOFF     — turn to shelf (+79°), approach, lower lift to shelf height,
                  open claw, lift back to carry, retreat, turn back to -Y.
 9. WATCH_STOP  — pure-pursuit in -Y, stop-sign vision checked every FSM tick.
10. STOP_WAIT   — halt STOP_SIGN_PAUSE_S seconds (2 s).
11. FINAL_DRIVE — advance FINAL_ADVANCE_MM (610 mm), full stop.
12. DONE        — return lift to zero, release camera.

Controls
────────
  BTN_1 = start mission
  BTN_2 = cancel / emergency stop  (checked in all motion states)

Tuning notes
────────────
  • DIST_TO_SCAN_MM, CUSTOMER_A_TRAVEL_MM, CUSTOMER_B_TRAVEL_MM and the
    TO_END distances MUST be verified on the real arena.
  • TURN_TO_SHELF_DEG is +79° (left turn) — robot faces +X (shelf side)
    while travelling in -Y.  Adjust sign if shelf is on the other side.
  • Lift is assumed to already be at LIFT_CARRY_TICKS (burger on board from
    Section 1).  Starting fresh?  Jog the lift manually before pressing BTN_1.
"""
from __future__ import annotations

import sys
import time

import cv2

sys.path.insert(0, '/ros2_ws/src/vision')
from vision.gender_detection import GenderDetector

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LEDMode,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
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

# =============================================================================
# SENSOR TOGGLES
# =============================================================================
ENABLE_VISION = True
ENABLE_LIDAR  = True
ENABLE_GPS    = True   # GPS fuses when tag is visible; gracefully degrades to odom

TAG_ID                          = 25       # ArUco marker ID on the robot
GPS_POSITION_ALPHA              = 0.20
ENABLE_GPS_TANGENT_HEADING      = True
GPS_TANGENT_ALPHA               = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0

# =============================================================================
# LIFT MOTOR  — identical values to test_burger_pickup.py
# =============================================================================
LIFT_MOTOR        = Motor.DC_M3
LIFT_CARRY_TICKS  = -12800    # travel / carry height (burger on board)
LIFT_PICKUP_TICKS = -8300     # shelf height — same shelf level used for drop-off
LIFT_DOWN_TICKS   = 0         # fully lowered / zeroed
LIFT_MAX_VEL      = 1800      # ticks / s
LIFT_TOLERANCE    = 30        # ticks
LIFT_TIMEOUT_S    = 20.0

# =============================================================================
# CLAW SERVO  — identical values to test_burger_pickup.py
# =============================================================================
CLAW_SERVO         = ServoChannel.CH_13
CLAW_OPEN_DEG      = 60.0
CLAW_CLOSE_DEG     = 111.0
SERVO_DEG_PER_STEP = 0.8       # max degrees moved per FSM tick (speed governor)

# =============================================================================
# DRIVE / NAVIGATION
# =============================================================================
DRIVE_VELOCITY       = 100.0   # mm/s — normal travel
APPROACH_VELOCITY    =  60.0   # mm/s — shelf approach / retreat
POS_TOLERANCE_MM     =  20.0   # mm
TURN_TOLERANCE_DEG   =   2.0   # degrees

# Shelf approach  — same as test_burger_pickup.py
# Robot travels in -Y; shelf is at higher X (to the robot's left).
# Positive TURN_TO_SHELF_DEG = left turn → robot faces +X toward shelf.
TURN_TO_SHELF_DEG    =  79.0   # left turn to face shelf
TURN_FROM_SHELF_DEG  = -79.0   # right turn to return to -Y heading
APPROACH_SHELF_DIST  =  18.0   # mm

# Pure-pursuit settings for the stop-sign watch leg
PP_VELOCITY_MM_S     = 150.0
LOOKAHEAD_MM         = 225.0
PP_TOLERANCE_MM      =  25.0
ADVANCE_RADIUS_MM    =  75.0
PP_MAX_ANGULAR_RAD_S =   1.5

STATUS_PRINT_INTERVAL_S = 0.5

# =============================================================================
# ABSOLUTE HEADINGS  (degrees; INITIAL_THETA_DEG = 90 → robot starts facing +Y)
# =============================================================================
HEADING_POS_X  =   0.0    # after first right turn:  facing +X
HEADING_NEG_Y  = -79.0    # after second right turn:  facing -Y (toward customers)

# =============================================================================
# MISSION GEOMETRY  *** tune these distances on the real arena ***
# =============================================================================

# Distance to drive in +X direction before stopping for face detection.
# Arena reference: x 1800 → 2200 mm  →  400 mm
DIST_TO_SCAN_MM = 400.0

# Distance to drive in -Y from the scan point to arrive alongside each shelf.
# A 100 mm buffer keeps the claw clear of the shelf edge; turn happens there.
# Arena reference:
#   Customer A (Female): y 3700 → 1800  (shelf y=1700, stop 100 mm before)
#   Customer B (Male):   y 3700 → 1400  (shelf y=1300, stop 100 mm before)
CUSTOMER_A_TRAVEL_MM = 1900.0  # *** tune ***
CUSTOMER_B_TRAVEL_MM = 2300.0  # *** tune ***

# Distance to continue in -Y after drop-off until the stop-sign zone is reached.
# Added buffer so vision triggers before the planned path ends.
# Arena reference:
#   From y=1700 (A) to stop-sign y≈200  →  1500 mm + 300 mm buffer = 1800 mm
#   From y=1300 (B) to stop-sign y≈200  →  1100 mm + 300 mm buffer = 1400 mm
CUSTOMER_A_TO_END_MM = 1800.0  # *** tune ***
CUSTOMER_B_TO_END_MM = 1400.0  # *** tune ***

# Behaviour at the stop sign
STOP_SIGN_PAUSE_S   = 2.0     # seconds to wait after stopping
FINAL_ADVANCE_MM    = 610.0   # mm to advance after the pause
MIN_STOP_CONFIDENCE = 0.50    # vision confidence threshold for stop-sign trigger

# =============================================================================
# FACE DETECTION
# =============================================================================
CAMERA_DEVICE      = '/dev/video10'
FACE_SAMPLE_FRAMES = 15       # frames per voting pass
FACE_FRAME_DELAY_S = 0.10     # delay between frames (seconds)
FACE_MAX_ATTEMPTS  =  10       # retry passes before giving up

# =============================================================================
# ASYNC LIFT SPOON-FEEDER  (mirrors test_burger_pickup.py)
# =============================================================================
LIFT_STEP_TICKS  = 1500       # maximum ticks sent per step
LIFT_STEP_DELAY  = 0.150      # seconds between steps

# =============================================================================
# LED HELPERS
# =============================================================================
def _led(robot: Robot, o=0, g=0, r=0, b=0, b_mode=None, b_period=800) -> None:
    robot.set_led(LED.ORANGE, o)
    robot.set_led(LED.GREEN,  g, **({"mode": b_mode, "period_ms": b_period} if b_mode else {}))
    robot.set_led(LED.RED,    r)
    robot.set_led(LED.BLUE,   b)

def led_idle(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN,    0); robot.set_led(LED.RED, 0); robot.set_led(LED.BLUE, 0)

def led_moving(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200); robot.set_led(LED.RED, 0); robot.set_led(LED.BLUE, 0)

def led_detect(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0); robot.set_led(LED.GREEN, 0); robot.set_led(LED.RED, 0)
    robot.set_led(LED.BLUE, 200, mode=LEDMode.BLINK, period_ms=400)

def led_stop_sign(robot: Robot) -> None:
    robot.set_led(LED.RED, 255, mode=LEDMode.BLINK, period_ms=500)
    robot.set_led(LED.GREEN, 0); robot.set_led(LED.ORANGE, 0); robot.set_led(LED.BLUE, 0)

def led_error(robot: Robot) -> None:
    robot.set_led(LED.RED, 200)
    robot.set_led(LED.ORANGE, 0); robot.set_led(LED.GREEN, 0); robot.set_led(LED.BLUE, 0)

def led_done(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0); robot.set_led(LED.RED, 0); robot.set_led(LED.BLUE, 0)
    robot.set_led(LED.GREEN, 200, mode=LEDMode.BLINK, period_ms=400)

# =============================================================================
# ROBOT SETUP
# =============================================================================
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
        print("[sensor] vision enabled — stop-sign detection active")
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
        print("[sensor] lidar enabled")
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
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2 s; continuing")
    robot.wait_for_pose_update(timeout=0.5)


# =============================================================================
# LIFT HELPERS
# =============================================================================
def get_lift_ticks(robot: Robot) -> int:
    dc = robot.get_dc_state()
    return 0 if dc is None else int(dc.motors[LIFT_MOTOR - 1].position)


def lift_to_blocking(robot: Robot, target_ticks: int) -> bool:
    """Move lift to target_ticks, blocking until done or timeout."""
    travel   = abs(get_lift_ticks(robot) - target_ticks)
    timeout  = max(LIFT_TIMEOUT_S, travel / LIFT_MAX_VEL * 1.5)
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    ok = robot.set_motor_position(
        LIFT_MOTOR, target_ticks,
        max_vel_ticks=LIFT_MAX_VEL,
        tolerance_ticks=LIFT_TOLERANCE,
        blocking=True, timeout=timeout,
    )
    if not ok:
        print(f"[LIFT][warn] did not confirm target {target_ticks} ticks")
    return ok


def lift_return_to_zero(robot: Robot) -> None:
    current = get_lift_ticks(robot)
    print(f"[LIFT] Returning to zero (currently {current} ticks)")
    if abs(current) > LIFT_TOLERANCE:
        ok = lift_to_blocking(robot, LIFT_DOWN_TICKS)
        print("[LIFT] At zero." if ok else "[LIFT][warn] did not confirm zero.")
    robot.disable_motor(LIFT_MOTOR)


# =============================================================================
# CLAW HELPERS  — same pattern as test_burger_pickup.py
# =============================================================================
def claw_open(robot: Robot) -> None:
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_OPEN_DEG)
    time.sleep(0.5)


def claw_close(robot: Robot) -> None:
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_CLOSE_DEG)
    time.sleep(0.5)


# =============================================================================
# VISION HELPERS
# =============================================================================
def stop_sign_detected(robot: Robot) -> bool:
    """Return True if vision reports a stop sign above confidence threshold."""
    if not ENABLE_VISION:
        return False
    if not robot.is_vision_active(timeout_s=3.0):
        return False
    for det in robot.get_detections("stop sign"):
        if float(det.get("confidence", 0)) >= MIN_STOP_CONFIDENCE:
            return True
    return False


# =============================================================================
# FACE DETECTION
# =============================================================================
def run_face_detection(detector: GenderDetector, cap: cv2.VideoCapture) -> str:
    """
    Samples FACE_SAMPLE_FRAMES frames up to FACE_MAX_ATTEMPTS times.

    Decision rule (per user spec):
      • 100% of votes are Male  →  Customer B  (Male)
      • Any Female vote, or no face detected  →  Customer A  (Female / uncertain)

    Returns 'A' or 'B'.
    """
    for attempt in range(1, FACE_MAX_ATTEMPTS + 1):
        votes = {"Male": 0, "Female": 0}
        for _ in range(FACE_SAMPLE_FRAMES):
            ret, frame = cap.read()
            if not ret:
                continue
            gender, _conf = detector.detect(frame)
            if gender in votes:
                votes[gender] += 1
            time.sleep(FACE_FRAME_DELAY_S)

        male_v, female_v = votes["Male"], votes["Female"]
        total = male_v + female_v
        print(
            f"[FACE] Attempt {attempt}/{FACE_MAX_ATTEMPTS} — "
            f"Male:{male_v}  Female:{female_v}"
        )

        if total == 0:
            print("[FACE] No face detected — retrying...")
            continue  # try again

        if female_v == 0 and male_v > 0:
            # Every single vote was Male → 100% confident
            print("[FACE] 100% Male → Customer B")
            return "B"
        else:
            # At least one Female vote: uncertain or genuinely Female → A
            print(f"[FACE] Not 100% Male → Customer A (Female={female_v})")
            return "A"

    print("[FACE] Max attempts reached with no face — defaulting to Customer A")
    return "A"


# =============================================================================
# DROP-OFF SEQUENCE  — mirrors DO_PLACE in test_burger_pickup.py
# =============================================================================
def perform_dropoff(robot: Robot, customer: str) -> None:
    """
    Blocking sequence:
      1. Turn left TURN_TO_SHELF_DEG to face the shelf (+X direction).
      2. Approach APPROACH_SHELF_DIST mm.
      3. Lower lift to LIFT_PICKUP_TICKS (same shelf height as pickup).
      4. Open claw — burger released.
      5. Lift back to LIFT_CARRY_TICKS (clear of shelf before retreating).
      6. Retreat APPROACH_SHELF_DIST mm.
      7. Turn right TURN_FROM_SHELF_DEG to restore -Y heading.

    NOTE: BTN_2 is not checked during this function.  The sequence is short
          enough (~10 s) that aborting mid-release would make a mess anyway.
    """
    print(f"\n[DROPOFF] ── Customer {customer} ──────────────────────────────────")

    # 1. Face the shelf
    print(f"[DROPOFF] Turning {TURN_TO_SHELF_DEG:+.0f}° to face shelf")
    robot.turn_by(
        delta_deg=TURN_TO_SHELF_DEG,
        blocking=True,
        tolerance_deg=TURN_TOLERANCE_DEG,
    )

    # 2. Approach
    print(f"[DROPOFF] Approaching {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_forward(
        distance=APPROACH_SHELF_DIST,
        velocity=APPROACH_VELOCITY,
        tolerance=POS_TOLERANCE_MM,
        blocking=True,
    )
    robot.stop()

    # 3. Lower to shelf / pickup height
    print(f"[DROPOFF] Lowering to shelf height ({LIFT_PICKUP_TICKS} ticks)")
    lift_to_blocking(robot, LIFT_PICKUP_TICKS)

    # 4. Release burger
    print(f"[DROPOFF] Opening claw ({CLAW_OPEN_DEG}°) — releasing burger")
    claw_open(robot)
    time.sleep(0.5)   # let it settle on the shelf

    # 5. Lift clear before reversing (prevents knocking the burger)
    print(f"[DROPOFF] Lifting clear to carry height ({LIFT_CARRY_TICKS} ticks)")
    lift_to_blocking(robot, LIFT_CARRY_TICKS)

    # 6. Retreat
    print(f"[DROPOFF] Retreating {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_backward(
        distance=APPROACH_SHELF_DIST,
        velocity=APPROACH_VELOCITY,
        tolerance=POS_TOLERANCE_MM,
        blocking=True,
    )

    # 7. Restore -Y travel heading
    print(f"[DROPOFF] Turning {TURN_FROM_SHELF_DEG:+.0f}° back to -Y heading")
    robot.turn_by(
        delta_deg=TURN_FROM_SHELF_DEG,
        blocking=True,
        tolerance_deg=TURN_TOLERANCE_DEG,
    )
    robot.stop()
    print("[DROPOFF] Complete.\n")


# =============================================================================
# PURE-PURSUIT WAYPOINT BUILDER  (stop-sign watch leg)
# =============================================================================
def build_stop_leg_waypoints(customer: str) -> list[tuple[float, float]]:
    """
    Returns densified waypoints in odometry space for the -Y run after drop-off.

    Odometry frame after pose reset (INITIAL_THETA_DEG = 90):
      • Reset  → (0, 0), theta=90°
      • After turn_to(0°) + move DIST_TO_SCAN_MM in +X  → (DIST_TO_SCAN_MM, 0)
      • After turn_to(-90°)  → still (DIST_TO_SCAN_MM, 0), now facing -Y
      • After moving CUSTOMER_X_TRAVEL_MM in -Y  → (DIST_TO_SCAN_MM, -TRAVEL_MM)

    X stays constant; only Y changes during the customer drive and stop-sign leg.
    """
    x = DIST_TO_SCAN_MM
    if customer == "A":
        start_y = -CUSTOMER_A_TRAVEL_MM
        end_y   = start_y - CUSTOMER_A_TO_END_MM
    else:
        start_y = -CUSTOMER_B_TRAVEL_MM
        end_y   = start_y - CUSTOMER_B_TO_END_MM

    raw = [(x, start_y), (x, end_y)]
    return densify_polyline(raw, spacing=100.0)


# =============================================================================
# MOTION HELPER
# =============================================================================
def cancel_motion(robot: Robot, handle) -> None:
    """Cancel a non-blocking motion handle safely."""
    if handle is not None:
        try:
            handle.cancel()
            handle.wait(timeout=1.0)
        except Exception:
            pass
    robot.stop()


def get_pose(robot: Robot) -> tuple[str, float, float, float]:
    """Return best available pose: fused (GPS) if available, else odometry."""
    if ENABLE_GPS and robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        return "fused", x, y, theta
    x, y, theta = robot.get_odometry_pose()
    return "odom ", x, y, theta


# =============================================================================
# run() — FSM entry point called by robot_node.py
# =============================================================================
def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    # Open camera and detector once — kept open for the whole mission
    detector = GenderDetector()
    cap      = cv2.VideoCapture(CAMERA_DEVICE)
    if not cap.isOpened():
        print(f"[warn] camera {CAMERA_DEVICE} unavailable — face detection will default to A")

    # FSM state
    state         = "INIT"
    customer      = "A"          # default; overwritten by face detection
    motion_handle = None
    stop_timer    = 0.0
    last_status_t = 0.0

    # Tick control
    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    # ── Async lift spoon-feeder (mirrors test_burger_pickup.py) ──────────────
    requested_final_ticks = LIFT_CARRY_TICKS   # desired end position
    internal_target_ticks = LIFT_CARRY_TICKS   # current step target
    last_step_time        = 0.0

    # ── Async servo speed governor (mirrors test_burger_pickup.py) ───────────
    requested_claw_deg = CLAW_CLOSE_DEG
    current_claw_deg   = CLAW_CLOSE_DEG

    print()
    print("=" * 62)
    print("  DELIVERY MISSION — Section 2")
    print("  Face detection → Customer delivery → Stop-sign finish")
    print("=" * 62)

    while True:
        now = time.monotonic()

        # ==================================================================
        # INIT — one-shot hardware setup
        # ==================================================================
        if state == "INIT":
            robot.stop()
            robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
            requested_final_ticks = LIFT_CARRY_TICKS   # keep burger elevated
            claw_close(robot)
            led_idle(robot)
            print(
                "[FSM] IDLE — ensure burger is on board and lift is at carry height,\n"
                "             then press BTN_1 to start."
            )
            state = "IDLE"

        # ==================================================================
        # IDLE — wait for BTN_1
        # ==================================================================
        elif state == "IDLE":
            robot.stop()
            if robot.was_button_pressed(Button.BTN_1):
                reset_pose(robot)
                led_moving(robot)
                print(
                    f"[FSM] TURN_TO_X — turning to abs heading {HEADING_POS_X}° "
                    f"(facing +X)"
                )
                motion_handle = robot.turn_to(
                    HEADING_POS_X,
                    blocking=False,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )
                state = "TURN_TO_X"

        # ==================================================================
        # TURN_TO_X — first right turn: +Y → +X  (theta 90° → 0°)
        # ==================================================================
        elif state == "TURN_TO_X":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                led_idle(robot)
                print("[FSM] IDLE — cancelled during TURN_TO_X")
                state = "IDLE"

            elif motion_handle is not None and motion_handle.is_finished():
                robot.stop()
                motion_handle = None
                print(
                    f"[FSM] DRIVE_SCAN — driving {DIST_TO_SCAN_MM:.0f} mm in +X "
                    f"to face-detection zone"
                )
                # Short, straight drive → blocking is fine
                robot.move_forward(
                    distance=DIST_TO_SCAN_MM,
                    velocity=DRIVE_VELOCITY,
                    tolerance=POS_TOLERANCE_MM,
                    blocking=True,
                )
                robot.stop()
                print("[FSM] Reached scan position — stopping for face detection")
                state = "FACE_DETECT"

        # ==================================================================
        # FACE_DETECT — camera-based customer identification
        # NOTE: robot is stationary; BTN_2 is not checked during the scan
        #       (~4.5 s max).  Acceptable since the robot is stopped.
        # ==================================================================
        elif state == "FACE_DETECT":
            led_detect(robot)
            robot.stop()

            if cap.isOpened():
                customer = run_face_detection(detector, cap)
            else:
                print("[FACE] Camera unavailable — defaulting to Customer A")
                customer = "A"

            travel_mm = CUSTOMER_A_TRAVEL_MM if customer == "A" else CUSTOMER_B_TRAVEL_MM
            print(
                f"[FSM] Customer {customer} selected "
                f"({'Female' if customer == 'A' else 'Male'}), "
                f"travel {travel_mm:.0f} mm in -Y"
            )
            led_moving(robot)
            print(
                f"[FSM] TURN_NEG_Y — turning to abs heading {HEADING_NEG_Y}° "
                f"(facing -Y toward customers)"
            )
            motion_handle = robot.turn_to(
                HEADING_NEG_Y,
                blocking=False,
                tolerance_deg=TURN_TOLERANCE_DEG,
            )
            state = "TURN_NEG_Y"

        # ==================================================================
        # TURN_NEG_Y — second right turn: +X → -Y  (theta 0° → -90°)
        # ==================================================================
        elif state == "TURN_NEG_Y":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                led_idle(robot)
                print("[FSM] IDLE — cancelled during TURN_NEG_Y")
                state = "IDLE"

            elif motion_handle is not None and motion_handle.is_finished():
                robot.stop()
                motion_handle = None
                travel_mm = (
                    CUSTOMER_A_TRAVEL_MM if customer == "A"
                    else CUSTOMER_B_TRAVEL_MM
                )
                print(
                    f"[FSM] DRIVE_CUST — driving {travel_mm:.0f} mm straight in -Y "
                    f"to Customer {customer}"
                )
                # Straight run to customer row.
                # GPS fuses when the tag is visible; falls back to odometry
                # automatically once the tag leaves the camera's FOV.
                robot.move_forward(
                    distance=travel_mm,
                    velocity=DRIVE_VELOCITY,
                    tolerance=POS_TOLERANCE_MM,
                    blocking=True,
                )
                robot.stop()
                label, x, y, theta = get_pose(robot)
                print(
                    f"[FSM] Arrived at customer row — "
                    f"{label}=({x:.0f}, {y:.0f}) θ={theta:.1f}°"
                )
                state = "DROPOFF"

        # ==================================================================
        # DROPOFF — blocking shelf sequence
        # ==================================================================
        elif state == "DROPOFF":
            perform_dropoff(robot, customer)

            # Build non-blocking pure-pursuit path for the stop-sign leg
            waypoints = build_stop_leg_waypoints(customer)
            print(
                f"[FSM] WATCH_STOP — pure pursuit to stop sign "
                f"({len(waypoints)} waypoints)"
            )
            led_moving(robot)
            motion_handle = robot.purepursuit_follow_path(
                waypoints=waypoints,
                velocity=PP_VELOCITY_MM_S,
                lookahead=LOOKAHEAD_MM,
                tolerance=PP_TOLERANCE_MM,
                advance_radius=ADVANCE_RADIUS_MM,
                max_angular_rad_s=PP_MAX_ANGULAR_RAD_S,
                blocking=False,
            )
            last_status_t = now
            state = "WATCH_STOP"

        # ==================================================================
        # WATCH_STOP — drive in -Y; break on stop sign detected or BTN_2
        # ==================================================================
        elif state == "WATCH_STOP":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                led_idle(robot)
                print("[FSM] IDLE — cancelled during WATCH_STOP")
                state = "IDLE"

            elif stop_sign_detected(robot):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                led_stop_sign(robot)
                print("[VISION] Stop sign detected — halting")
                stop_timer = now
                state = "STOP_WAIT"

            else:
                if now - last_status_t >= STATUS_PRINT_INTERVAL_S:
                    label, x, y, theta = get_pose(robot)
                    print(
                        f"  [WATCH] {label}=({x:.0f}, {y:.0f}) θ={theta:.1f}° "
                        f"| watching for stop sign..."
                    )
                    last_status_t = now

                if motion_handle is not None and motion_handle.is_finished():
                    # Planned path ran out before stop sign → extend forward slowly
                    print(
                        "[FSM] Planned path exhausted — extending 500 mm "
                        "to keep looking for stop sign"
                    )
                    _, cur_x, cur_y, _ = get_pose(robot)
                    ext_wps = densify_polyline(
                        [(cur_x, cur_y), (cur_x, cur_y - 500.0)],
                        spacing=100.0,
                    )
                    motion_handle = robot.purepursuit_follow_path(
                        waypoints=ext_wps,
                        velocity=APPROACH_VELOCITY,   # slow — near the sign now
                        lookahead=LOOKAHEAD_MM,
                        tolerance=PP_TOLERANCE_MM,
                        advance_radius=ADVANCE_RADIUS_MM,
                        max_angular_rad_s=PP_MAX_ANGULAR_RAD_S,
                        blocking=False,
                    )
                    last_status_t = now

        # ==================================================================
        # STOP_WAIT — stationary pause at the stop sign
        # ==================================================================
        elif state == "STOP_WAIT":
            robot.stop()    # keep commanding stop in case of slip
            if now - stop_timer >= STOP_SIGN_PAUSE_S:
                print(
                    f"[FSM] FINAL_DRIVE — {STOP_SIGN_PAUSE_S:.0f} s elapsed, "
                    f"advancing {FINAL_ADVANCE_MM:.0f} mm"
                )
                led_moving(robot)
                robot.move_forward(
                    distance=FINAL_ADVANCE_MM,
                    velocity=DRIVE_VELOCITY,
                    tolerance=POS_TOLERANCE_MM,
                    blocking=True,
                )
                robot.stop()
                led_done(robot)
                lift_return_to_zero(robot)
                if cap.isOpened():
                    cap.release()
                print()
                print("=" * 62)
                print("  MISSION COMPLETE — all tasks done.")
                print("=" * 62)
                break   # exit FSM loop

        # ==================================================================
        # GLOBAL E-STOP  — BTN_2 from states not already checking it
        # ==================================================================
        # TURN_TO_X / TURN_NEG_Y / WATCH_STOP handle BTN_2 themselves.
        # FACE_DETECT and DROPOFF are short blocking states; tolerated.
        # STOP_WAIT is deliberate immobility — no cancel needed.
        # IDLE is already safe.

        # ==================================================================
        # ASYNC LIFT SPOON-FEEDER  (mirrors test_burger_pickup.py)
        # Keeps the lift tracking `requested_final_ticks` in the background
        # without overshooting; safe to run every tick.
        # ==================================================================
        current_phys = get_lift_ticks(robot)
        if abs(current_phys - requested_final_ticks) > LIFT_TOLERANCE:
            if now - last_step_time >= LIFT_STEP_DELAY:
                if requested_final_ticks < current_phys:
                    internal_target_ticks = max(
                        requested_final_ticks, current_phys - LIFT_STEP_TICKS
                    )
                else:
                    internal_target_ticks = min(
                        requested_final_ticks, current_phys + LIFT_STEP_TICKS
                    )
                robot.set_motor_position(
                    LIFT_MOTOR,
                    internal_target_ticks,
                    max_vel_ticks=LIFT_MAX_VEL,
                    tolerance_ticks=LIFT_TOLERANCE,
                    blocking=False,
                )
                last_step_time = now

        # ==================================================================
        # ASYNC SERVO SPEED GOVERNOR  (mirrors test_burger_pickup.py)
        # Smoothly interpolates the claw toward `requested_claw_deg`.
        # ==================================================================
        if abs(current_claw_deg - requested_claw_deg) > 0.5:
            if requested_claw_deg > current_claw_deg:
                current_claw_deg = min(
                    requested_claw_deg,
                    current_claw_deg + SERVO_DEG_PER_STEP,
                )
            else:
                current_claw_deg = max(
                    requested_claw_deg,
                    current_claw_deg - SERVO_DEG_PER_STEP,
                )
            robot.set_servo(CLAW_SERVO, current_claw_deg)

        # ==================================================================
        # TICK-RATE CONTROL  — paced at DEFAULT_FSM_HZ
        # ==================================================================
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
