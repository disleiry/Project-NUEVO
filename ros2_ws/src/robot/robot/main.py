"""
main_combined.py — burger pickup → pure pursuit → LAPF obstacle navigation
===========================================================================

Mission order:
1. INIT: robot enables lift motor, closes claw, waits for lift jog alignment.
2. INIT_JOG: BTN_1 jogs lift up, BTN_2 jogs lift down, BTN_10 zeroes encoder.
3. WAIT_GREEN: BTN_5 starts the burger pickup sequence.
4. BURGER_PICKUP: fetches meat, assembles burger, grabs full stack.
5. BURGER_DONE: odometry is reset automatically, course begins immediately.
6. COURSE_MOVING: pure pursuit for straight/ramp section, then LAPF for obstacle course.

How to run:
    cp main_combined.py ros2_ws/src/robot/main.py
    ros2 run robot robot

Make sure these are running in separate terminals as needed:
    ros2 launch vision vision_production.launch.py
    ros2 run robot bridge
    ros2 run robot robot

Controls:
    BTN_1 / BTN_2 / BTN_10  lift jog during INIT_JOG alignment phase.
    BTN_5                    starts the burger pickup sequence from WAIT_GREEN.
    BTN_2                    cancels and returns to COURSE_IDLE during the course phase.
"""

from __future__ import annotations


import statistics

import time
from typing import Any

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    LED,
    LEDMode,
    INITIAL_THETA_DEG,
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


# ---------------------------------------------------------------------------
# Sensor setup
# ---------------------------------------------------------------------------


# --- existing ---
APPROACH_SHELF_DIST = 45.0

# --- ADD these 4 lines ---
SHELF_TARGET_MM = 80.0           # target distance to shelf face during pickup
SHELF_SCAN_RANGE_MM = 350.0      # lidar max range while scanning shelf
SHELF_SCAN_MIN_MM = 20.0         # lidar min range while scanning shelf (allows close-range)
SHELF_ANGLE_THRESHOLD_DEG = 2.0  # skip correction if angle error is smaller than this






ENABLE_VISION = True
ENABLE_LIDAR = True
ENABLE_GPS = True

# IMPORTANT: update this to match the ArUco marker ID on your robot.
TAG_ID = 25

# GPS tuning.
GPS_POSITION_ALPHA = 0.15
ENABLE_GPS_TANGENT_HEADING = True
GPS_TANGENT_ALPHA = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0

ANGULAR_VELOCITY_DEG = 20
CUSTOMER_A_TO_STOP_MM = 1050.0
CUSTOMER_B_TO_STOP_MM = 700.0


WALL_TARGET_MM = 170.0
WALL_KP = 0.05
WALL_DEAD_BAND_MM = 10.0
WALL_MAX_ANGULAR_DEG_S = 5.0
WALL_LOOP_DT_S = 0.05

# ---------------------------------------------------------------------------
# Traffic-light start behavior
# ---------------------------------------------------------------------------

LED_BRIGHTNESS = 255
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_CONFIDENCE = 0.50

# Fixed traffic-light viewing turn.
# The robot turns this many degrees in place, then stops and waits.
# Use 45.0 if the traffic light is farther out of view.
# If the robot turns the wrong direction, flip the sign.
TRAFFIC_LIGHT_TURN_DEG = 30.0
TURN_TOLERANCE_DEG = 0.5

# Stop sign safety override from the traffic-light example.
ENABLE_STOP_SIGN_OVERRIDE = True


STOP_SIGN_APPROACH_MM = 220.0
# Forward speed while scanning for the stop sign (mm/s).
STOP_SIGN_SCAN_VELOCITY = 100.0

# ---------------------------------------------------------------------------
# Waypoint paths through the arena
# ---------------------------------------------------------------------------
# Units are millimeters.
# With INITIAL_THETA_DEG = 90, +Y is usually the robot's initial forward direction.

PURE_PURSUIT_CONTROL_POINTS = [
    #(0.0, 0.0),        # start
    (70.0, 3750.0),      # Waypoint 1: home straight
    (580.0, 3750.0),    # Waypoint 2: transition / turn
    (700.0, 810.0),     # Waypoint 3: ramp / return direction
    (1300.0, 775.0),    # Waypoint 4: entrance toward obstacle course
]

PURE_PURSUIT_CONTROL_POINTS_2 = [
    #(0.0, 0.0),        # start
    (1700.0, 3750.0),      # Waypoint 1: home straight
    (1800.0, 3750.0),
    (1900.0, 3750.0),
    (2000.0, 3750.0),    # Waypoint 2: transition / turn

]
# Optional: densify long pure-pursuit segments for smoother tracking.
PURE_PURSUIT_CONTROL_POINTS = densify_polyline(PURE_PURSUIT_CONTROL_POINTS, spacing=100.0)
#PURE_PURSUIT_CONTROL_POINTS_2 = densify_polyline(PURE_PURSUIT_CONTROL_POINTS_2, spacing=100.0)

# LAPF is only used in the obstacle-course section.
LAPF_CONTROL_POINTS = [
    (1700.0, 3400.0),   # Obstacle waypoint / finish
]

# Optional: densify LAPF segments so the obstacle-course path has intermediate goals.
#LAPF_CONTROL_POINTS = densify_polyline(LAPF_CONTROL_POINTS, spacing=50.0)


# ---------------------------------------------------------------------------
# Pure pursuit tuning for straight/ramp sections
# ---------------------------------------------------------------------------

PURE_PURSUIT_VELOCITY_MM_S = 150.0
LOOKAHEAD_MM = 200.0
PURE_PURSUIT_TOLERANCE_MM = 25.0
ADVANCE_RADIUS_MM = 150.0
PURE_PURSUIT_MAX_ANGULAR_RAD_S = 1.5


# ---------------------------------------------------------------------------
# LAPF tuning for obstacle-course section
# ---------------------------------------------------------------------------

LAPF_VELOCITY_MM_S = 150.0
LAPF_TOLERANCE_MM = 50.0
LAPF_MAX_ANGULAR_RAD_S = 0.4

LEASH_LENGTH_MM = 150.0
REPULSION_RANGE_MM = 250.0
TARGET_SPEED_MM_S = 200.0
REPULSION_GAIN = 350.0
ATTRACTION_GAIN = 1.2
FORCE_EMA_ALPHA = 0.15
INFLATION_MARGIN_MM = 250.0
LEASH_HALF_ANGLE_DEG = 25.0

STATUS_PRINT_INTERVAL_S = 0.5
STAGE_PAUSE_S = 0.00


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
    
MISSION_STAGES.append(
    {
        "name": "Pure Pursuit part 2",
        "type": "pure_pursuit_2",
        "waypoints": PURE_PURSUIT_CONTROL_POINTS_2,
    }
)


# ---------------------------------------------------------------------------
# Burger pickup constants
# ---------------------------------------------------------------------------

LIFT_MOTOR = Motor.DC_M3
LIFT_CARRY_TICKS = -13200
LIFT_PICKUP_TICKS = -8800
LIFT_ITEM_THICKNESS_TICKS_1 = -2000
LIFT_ITEM_THICKNESS_TICKS_2 = -1600

LIFT_MAX_VEL = 1650
LIFT_TOLERANCE = 30
LIFT_JOG_STEP = 100

CLAW_SERVO = ServoChannel.CH_13
CLAW_OPEN_DEG = 40.0
CLAW_CLOSE_MEAT_DEG = 150.0
CLAW_CLOSE_BUN_DEG = 141.0

DRIVE_VELOCITY = 150.0
APPROACH_VELOCITY = 60.0
POS_TOLERANCE_MM = 10.0

TURN_VELOCITY_DEG = 45.0
TURN_TO_SHELF_DEG = 79
TURN_FROM_SHELF_DEG = -79

DIST_TO_INGREDIENT_AREA = 670.0
APPROACH_SHELF_DIST = 45.0

INGREDIENT_SLOTS = {
    "bun_bottom": 207.0,
    "meat": 348.0,
    "bun_top": 480.0,
}





# Distance driven in -Y from the scan point to the customer row (100 mm buffer)
# Customer A (Female): y 3700 → stop at y~1800 (shelf at y=1700)
# Customer B (Male):   y 3700 → stop at y~1400 (shelf at y=1300)
CUSTOMER_A_TRAVEL_MM = 1900.0   # *** tune on arena ***
CUSTOMER_B_TRAVEL_MM = 2250.0   # *** tune on arena ***


MIN_STOP_CONF      = 0.50   # vision confidence threshold

# ==========================================
# --- FACE DETECTION ---
# ==========================================
CAMERA_DEVICE      = '/dev/video10'
FACE_SAMPLE_FRAMES = 15     # frames per voting pass
FACE_FRAME_DELAY_S = 0.10   # seconds between frames
FACE_MAX_ATTEMPTS  =  3     # voting passes before defaulting

# ==========================================
# --- Distance to Wall ---
# ==========================================

import math
def get_front_wall_distance_mm(robot: Robot) -> float:
    """
    Finds the closest obstacle directly in front of the robot using existing tracks.
    Returns distance in mm, or -1.0 if the path is clear.
    """
    obstacle_tracks = robot.get_obstacle_tracks()
    if not obstacle_tracks:
        return -1.0

    # Get current global pose
    _, rx, ry, rtheta_deg = get_best_pose(robot)
    rtheta_rad = math.radians(rtheta_deg)
    
    front_distances = []
    
    # Check every tracked obstacle
    for track in obstacle_tracks:
        ox = float(track["x"])
        oy = float(track["y"])
        radius = float(track["radius"])
        
        # Calculate delta from robot to obstacle
        dx = ox - rx
        dy = oy - ry
        
        # Rotate coordinates to the robot's local perspective (X is forward, Y is left)
        local_x = dx * math.cos(rtheta_rad) + dy * math.sin(rtheta_rad)
        local_y = -dx * math.sin(rtheta_rad) + dy * math.cos(rtheta_rad)
        
        # If the obstacle is in front (local_x > 0) and within a +/- 150mm lateral corridor
        if local_x > 0 and abs(local_y) < (150.0 + radius):
            # Distance to the edge of the obstacle
            dist = local_x - radius
            if dist > 0:
                front_distances.append(dist)
                
    if not front_distances:
        return -1.0
        
    return min(front_distances)

def get_robust_wall_distance(robot: Robot, num_samples: int = 10, delay_s: float = 0.1) -> float:
    """
    Takes multiple Lidar frames, throws away invalid (-1.0) readings, 
    and returns the median of the valid readings to reject outliers.
    """
    valid_distances = []
    
    print(f"[NAV] Taking {num_samples} Lidar samples...")
    for i in range(num_samples):
        dist = get_front_wall_distance_mm(robot)
        
        # Only keep the reading if it actually saw a wall
        if dist > 0.0:
            valid_distances.append(dist)
            
        # Keep the ROS heartbeat alive and wait for the Lidar to spin again
        robot.wait_for_pose_update(timeout=delay_s)
        
    # If every single frame was empty, the path is genuinely clear
    if not valid_distances:
        print("[NAV] All samples returned empty (-1.0).")
        return -1.0
        
    # The median ignores extreme high/low sensor glitches
    final_distance = statistics.median(valid_distances)
    print(f"[NAV] Valid samples: {valid_distances}")
    print(f"[NAV] Robust median distance: {final_distance:.0f} mm")
    
    return statistics.median(valid_distances)


import numpy as np
import math

def correct_shelf_angle(robot: Robot) -> float:
    """
    Call AFTER turn_by(81), BEFORE move_forward.
    Sets lidar to close-range mode, calculates the true surface angle of the shelf 
    using linear regression on the raw point cloud, and applies a corrective turn.
    Returns the angle corrected (deg) so the caller can include it in the return turn.
    """
    # 1. Prepare Lidar for close-range shelf scanning
    robot.set_lidar_filter(
        range_min_mm=SHELF_SCAN_MIN_MM,
        range_max_mm=SHELF_SCAN_RANGE_MM,
        fov_deg=LIDAR_FOV_DEG,
    )
    time.sleep(0.4)  # let sensor populate new points

    # 2. Get the raw point cloud (X, Y in millimeters relative to robot center)
    points = robot._obstacles_mm 
    
    if len(points) == 0:
        print("[SHELF] No LiDAR points found — no correction applied")
        return 0.0

    # 3. Define a forward bounding box to isolate the shelf face
    min_x = 20.0
    max_x = SHELF_SCAN_RANGE_MM + 50.0  
    max_y_width = 150.0  # Look 150mm left and 150mm right
    
    # Filter points to only keep those strictly in front of the robot
    mask = (points[:, 0] > min_x) & (points[:, 0] < max_x) & (np.abs(points[:, 1]) < max_y_width)
    shelf_points = points[mask]
    
    if len(shelf_points) < 10:
        print(f"[SHELF] Not enough points ({len(shelf_points)}) to find wall — no correction applied")
        return 0.0
        
    # 4. Calculate the Line of Best Fit (Linear Regression)
    y_coords = shelf_points[:, 1]
    x_coords = shelf_points[:, 0]
    
    slope, intercept = np.polyfit(y_coords, x_coords, 1)
    
    # Calculate the physical angle of the wall and reverse it to find the corrective turn
    angle_rad = math.atan(slope)
    shelf_angle_deg = math.degrees(angle_rad)
    angle_corr = -shelf_angle_deg  # Invert so we turn into the correction

    print(f"[SHELF] Calculated wall slope: {shelf_angle_deg:.2f}°, correction turn: {angle_corr:.2f}°")

    # 5. Apply correction turn if it's large enough to matter
    if abs(angle_corr) > SHELF_ANGLE_THRESHOLD_DEG:
        print(f"[SHELF] Squaring up: Turning by {angle_corr:.2f} degrees")
        robot.turn_by(
            angle_corr, 
            TURN_VELOCITY_DEG, 
            tolerance_deg=TURN_TOLERANCE_DEG, 
            blocking=True
        )
        time.sleep(0.2) # Give the robot a moment to settle mechanically
    else:
        print("[SHELF] Robot is already square enough — no turn applied")
        angle_corr = 0.0 # Don't return tiny values so we don't accumulate math noise

    return angle_corr






def correct_far_wall_angle(robot: Robot) -> float:
    """
    Uses raw LiDAR linear regression to square up perfectly to a wall that is
    up to 1500mm away.
    """
    from robot.hardware_map import LIDAR_RANGE_MIN_MM, LIDAR_FOV_DEG
    
    # Temporarily increase Lidar range so we can see the far wall
    robot.set_lidar_filter(
        range_min_mm=LIDAR_RANGE_MIN_MM,
        range_max_mm=1500.0,
        fov_deg=LIDAR_FOV_DEG,
    )
    time.sleep(0.5)  # Let the point cloud populate

    points = robot._obstacles_mm 
    if len(points) == 0:
        print("[ALIGN] No LiDAR points found")
        return 0.0

    # Filter for points roughly in front (up to 1.5 meters away, 200mm left/right)
    mask = (points[:, 0] > 100.0) & (points[:, 0] < 1500.0) & (np.abs(points[:, 1]) < 200.0)
    wall_points = points[mask]

    if len(wall_points) < 10:
        print("[ALIGN] Not enough points to detect wall")
        return 0.0

    # Calculate Line of Best Fit
    y_coords = wall_points[:, 1]
    x_coords = wall_points[:, 0]
    slope, _ = np.polyfit(y_coords, x_coords, 1)

    # Convert slope to degrees and invert for the correction turn
    angle_rad = math.atan(slope)
    wall_angle_deg = math.degrees(angle_rad)
    angle_corr = -wall_angle_deg

    print(f"[ALIGN] Wall slope: {wall_angle_deg:.2f}°, correction: {angle_corr:.2f}°")

    if abs(angle_corr) > 1.0:
        robot.turn_by(angle_corr, 15, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
        time.sleep(0.2)

    return angle_corr






def get_shelf_approach_mm(robot: Robot) -> float:
    """
    Call AFTER correct_shelf_angle, BEFORE move_forward.
    Uses raw Lidar point cloud to find the true physical distance to the shelf face,
    bypassing the obstacle tracker's circular boundaries.
    """
    valid = []
    for _ in range(8):
        # Grab the raw, unfiltered point cloud
        points = robot._obstacles_mm 
        
        if len(points) > 0:
            # Create a narrow corridor dead ahead
            # min_x=35.0 creates a tight blind spot to ignore the robot's own claw/lift
            min_x = 35.0  
            max_x = SHELF_SCAN_RANGE_MM + 50.0
            max_y_width = 80.0  # Look strictly 80mm left/right to avoid clutter
            
            mask = (points[:, 0] > min_x) & (points[:, 0] < max_x) & (np.abs(points[:, 1]) < max_y_width)
            shelf_points = points[mask]
            
            if len(shelf_points) > 5:
                # The true distance is just the median X coordinate of the wall hits
                wall_dist = np.median(shelf_points[:, 0])
                valid.append(wall_dist)
                
        robot.wait_for_pose_update(timeout=0.1)

    # Restore default filter now that both shelf measurements are done
    robot.set_lidar_filter(
        range_min_mm=LIDAR_RANGE_MIN_MM,
        range_max_mm=LIDAR_RANGE_MAX_MM,
        fov_deg=LIDAR_FOV_DEG,
    )

    if not valid:
        print("[SHELF] No raw LiDAR points found — using fixed APPROACH_SHELF_DIST")
        return APPROACH_SHELF_DIST

    # Average the valid frames to reject noise
    shelf_dist = statistics.median(valid)
    approach_mm = shelf_dist - SHELF_TARGET_MM
    
    print(f"[SHELF] True shelf dist={shelf_dist:.0f} mm, target={SHELF_TARGET_MM:.0f} mm, approach={approach_mm:.0f} mm")
    
    # Notice we removed the max(..., 0.0) clamp!
    # If approach_mm is negative, it will pass right through to your new backup logic.
    return approach_mm


def get_right_wall_distance_mm(robot: Robot) -> float:
    obstacle_tracks = robot.get_obstacle_tracks()
    if not obstacle_tracks:
        return -1.0

    _, rx, ry, rtheta_deg = get_best_pose(robot)
    rtheta_rad = math.radians(rtheta_deg)

    right_distances = []

    for track in obstacle_tracks:
        ox = float(track["x"])
        oy = float(track["y"])
        radius = float(track["radius"])

        dx = ox - rx
        dy = oy - ry

        local_x =  dx * math.cos(rtheta_rad) + dy * math.sin(rtheta_rad)
        local_y = -dx * math.sin(rtheta_rad) + dy * math.cos(rtheta_rad)

        # Right side: local_y < 0, within ±150 mm fore-aft corridor
        if local_y < 0 and abs(local_x) < (150.0 + radius):
            dist = abs(local_y) - radius
            if dist > 0:
                right_distances.append(dist)

    if not right_distances:
        return -1.0

    return min(right_distances)


def drive_with_wall_following(robot: Robot, distance_mm: float, velocity_mm_s: float) -> None:
    _, start_x, start_y, _ = get_best_pose(robot)
    traveled_mm = 0.0

    print(f"[WALL] Starting wall-following drive: {distance_mm:.0f} mm "
          f"target={WALL_TARGET_MM:.0f} mm  Kp={WALL_KP}")

    while traveled_mm < distance_mm:
        remaining_mm = distance_mm - traveled_mm

        right_dist = get_right_wall_distance_mm(robot)

        if right_dist > 0.0:
            error_mm = right_dist - WALL_TARGET_MM  # + = too far, - = too close
            if abs(error_mm) < WALL_DEAD_BAND_MM:
                angular_cmd = 0.0
            else:
                angular_cmd = -WALL_KP * error_mm
                angular_cmd = max(-WALL_MAX_ANGULAR_DEG_S,
                                  min(WALL_MAX_ANGULAR_DEG_S, angular_cmd))
            print(f"[WALL] dist={right_dist:6.1f} mm  err={error_mm:+6.1f} mm  "
                  f"angular={angular_cmd:+5.2f} deg/s  traveled={traveled_mm:.0f}/{distance_mm:.0f} mm")
        else:
            angular_cmd = 0.0
            print(f"[WALL] dist=NO READING  traveled={traveled_mm:.0f}/{distance_mm:.0f} mm — driving straight")

        if remaining_mm < 100.0:
            speed = max(velocity_mm_s * (remaining_mm / 100.0), 40.0)
        else:
            speed = velocity_mm_s

        robot.set_velocity(speed, angular_cmd)
        time.sleep(WALL_LOOP_DT_S)

        _, cx, cy, _ = get_best_pose(robot)
        traveled_mm = math.hypot(cx - start_x, cy - start_y)

    robot.stop()
    print(f"[WALL] Wall-following complete. Total traveled ≈ {traveled_mm:.0f} mm")


def drive_until_stop_sign(robot: Robot, final_distance_mm: float) -> None:
    """
    1. Drives forward STOP_SIGN_APPROACH_MM at full speed (no detection yet).
    2. Creeps forward at STOP_SIGN_SCAN_VELOCITY watching for a stop sign.
    3. On detection: stops, waits 2 s, then drives forward final_distance_mm.
    """
    robot.enable_vision()
    time.sleep(2.0)

    if not robot.is_vision_active(timeout_s=5.0):
        print("[STOP] WARNING — vision node not detected as active. Stop sign detection may fail.")
    else:
        print("[STOP] Vision node confirmed active.")
        
    # ── Phase 1: pre-detection advance ────────────────────────────────────
    print(f"[STOP] Pre-detection advance {STOP_SIGN_APPROACH_MM:.0f} mm...")
    robot.move_forward(STOP_SIGN_APPROACH_MM, DRIVE_VELOCITY, POS_TOLERANCE_MM, blocking=True)

    # ── Phase 2: creep forward watching for stop sign ─────────────────────
    # ── Phase 2: creep forward watching for stop sign ─────────────────────
    print("[STOP] Scanning for stop sign...")
    robot.set_velocity(STOP_SIGN_SCAN_VELOCITY, 0.0)
    
    while True:
        if robot.get_detections("stop sign"):
            for det in robot.get_detections("stop sign"):
                print(f"[STOP] Stop sign detected — confidence: {det.get('confidence', 'N/A'):.4f}")
            robot.stop()
            print("[STOP] Stopping — waiting 2 s")
            time.sleep(2.0)
            break
    
        # Keep velocity command alive (set_velocity is not latched)
        robot.set_velocity(STOP_SIGN_SCAN_VELOCITY, 0.0)
        robot.wait_for_pose_update(timeout=0.05)

    # ── Phase 3: final fixed drive ─────────────────────────────────────────
    print(f"[STOP] Resuming — driving {final_distance_mm:.0f} mm to final stop.")
    robot.move_forward(final_distance_mm, DRIVE_VELOCITY, POS_TOLERANCE_MM, blocking=True)
    print("[STOP] Final stop reached.")
# ---------------------------------------------------------------------------
# General helpers
# ---------------------------------------------------------------------------

def normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


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

    # NOTE: GPS is intentionally NOT enabled here.
    # It is enabled in BURGER_DONE, after the burger pickup is complete,
    # so it does not interfere with odometry-based burger pickup motion.


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


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


def get_best_pose(robot: Robot) -> tuple[str, float, float, float]:
    if ENABLE_GPS and robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        return "fused", x, y, theta

    x, y, theta = robot.get_odometry_pose()
    return "odom ", x, y, theta


# ---------------------------------------------------------------------------
# Vision helpers
# ---------------------------------------------------------------------------

def find_traffic_light_color(robot: Robot):
    """Return highest-confidence red/green traffic-light color, or None."""
    if not ENABLE_VISION:
        return None

    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best_color = None
    best_confidence = -1.0

    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
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


# ---------------------------------------------------------------------------
# Motion helpers (course)
# ---------------------------------------------------------------------------

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
    stage = MISSION_STAGES[stage_index]

    if stage["type"] == "pure_pursuit":
        return start_pure_pursuit_stage(robot, stage)

    if stage["type"] == "lapf":
        return start_lapf_stage(robot, stage)
        
    if stage["type"] == "pure_pursuit_2":
        return start_pure_pursuit_stage(robot, stage)

    raise ValueError(f"Unknown stage type: {stage['type']}")


# ---------------------------------------------------------------------------
# Burger pickup helpers
# ---------------------------------------------------------------------------

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

def detect_customer(robot: Robot) -> str:
    """
    Polls robot.get_detections('person') and votes over FACE_SAMPLE_FRAMES
    frames. Any person detection with a 'gender' attribute counts as a vote.
    Returns 'A' for Female, 'B' for Male, defaults to 'A' if no votes.
    """
    for attempt in range(1, FACE_MAX_ATTEMPTS + 1):
        votes = {"Female": 0, "Male": 0}
        samples = 0

        print(f"[FACE] Voting pass {attempt}/{FACE_MAX_ATTEMPTS}...")

        for _ in range(FACE_SAMPLE_FRAMES):
            detections = robot.get_detections("person")
            for det in detections:
                attributes = det.get("attributes", {})
                gender_attr = attributes.get("gender", {})
                gender = gender_attr.get("value")
                if gender in votes:
                    votes[gender] += 1
                    samples += 1
            time.sleep(FACE_FRAME_DELAY_S)

        print(f"[FACE] Pass {attempt} votes: {votes} ({samples} valid samples)")

        if samples > 0:
            winner = max(votes, key=votes.get)
            print(f"[FACE] Result: {winner}")
            return "A" if winner == "Female" else "B"

    print("[FACE] No valid detections after all attempts — defaulting to Customer A")
    return "A"


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


# ---------------------------------------------------------------------------
# Status / config printing
# ---------------------------------------------------------------------------

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
        f"  stage {stage_index + 1}/{len(MISSION_STAGES)} {stage['name']} "
        f"goal=({goal_x:.0f}, {goal_y:.0f}) mm | "
        f"{label}=({x:6.0f}, {y:6.0f}) mm θ={theta:5.1f}°"
        f"{gps_summary}{vt_summary}{track_summary}"
    )


def print_config(robot: Robot) -> None:
    lapf_cfg = resolve_lapf_config()

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

    if ENABLE_LIDAR:
        print(
            f"[CFG] lidar mount=({LIDAR_MOUNT_X_MM:.0f}, {LIDAR_MOUNT_Y_MM:.0f}) mm "
            f"theta={LIDAR_MOUNT_THETA_DEG:.1f}° filter={LIDAR_RANGE_MIN_MM:.0f}-"
            f"{LIDAR_RANGE_MAX_MM:.0f} mm fov={LIDAR_FOV_DEG}"
        )
        print(
            f"[CFG] tracker ttl={robot.OBSTACLE_TRACK_TTL_S:.1f}s "
            f"max_tracks={robot.OBSTACLE_TRACK_MAX_TRACKS} "
            f"planner_tracks={robot.LAPF_MAX_PLANNER_TRACKS}"
        )

    if ENABLE_GPS:
        print(
            f"[CFG] gps tag_id={TAG_ID} tag_body=({TAG_BODY_OFFSET_X_MM:.0f}, "
            f"{TAG_BODY_OFFSET_Y_MM:.0f}) mm position_alpha={GPS_POSITION_ALPHA:.2f}"
        )
        if ENABLE_GPS_TANGENT_HEADING:
            print(
                f"[CFG] heading=gps_tangent alpha={GPS_TANGENT_ALPHA:.2f} "
                f"min_displacement={GPS_TANGENT_MIN_DISPLACEMENT_MM:.0f} mm"
            )


# ---------------------------------------------------------------------------
# run() — entry point called by robot_node.py
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)

    state = "INIT"

    # Burger pickup state
    current_x = 0.0
    jog_ticks = 0

    # Course state
    course_stage_index = 0
    motion_handle = None
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
            claw_close(robot, CLAW_CLOSE_BUN_DEG)
            print("=== LIFT ALIGNMENT ===")
            print(" BTN_1: UP | BTN_2: DOWN | BTN_10: Confirm")
            state = "INIT_JOG"

        # ── INIT_JOG ─────────────────────────────────────────────────────────
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
                move_lift(robot, LIFT_CARRY_TICKS)
                robot.move_forward(300, 50, POS_TOLERANCE_MM, blocking=True)
                time.sleep(0.4)
                robot.turn_to(120, 10, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            time.sleep(0.05)
            

        # ── WAIT_GREEN ────────────────────────────────────────────────────────
        elif state == "WAIT_GREEN":
            

            # Start burger sequence on button press
            if robot.was_button_pressed(Button.BTN_2):
                show_idle_leds(robot)
                print("[FSM] IDLE — mission cancelled while waiting for green")
                state = "IDLE"
            else:
                
                traffic_light_color = find_traffic_light_color(robot)
    
                if traffic_light_color == "green":
                    show_traffic_light_color(robot, "green")
                    print("[VISION] green light — turning back to forward heading")
                    

                    motion_handle = robot.turn_to(90,10,blocking=True,tolerance_deg=TURN_TOLERANCE_DEG)
                    last_status_print_at = now
                    state = "BURGER_PICKUP"

                elif traffic_light_color == "red":
                    show_traffic_light_color(robot, "red")

                else:
                    # No detection yet: remain stopped at the fixed 15-degree viewing angle.
                    pass

        # ── BURGER_PICKUP ─────────────────────────────────────────────────────
        elif state == "BURGER_PICKUP":
            #robot.turn_to(90, 10, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

            # 1. PREP INITIAL MOVE
            claw_open(robot)
            

            # 2. NAVIGATE TO INGREDIENT AREA
            robot.move_forward(DIST_TO_INGREDIENT_AREA, 150 , POS_TOLERANCE_MM, blocking=True)

            # 3. FETCH MEAT
            current_x = drive_to_slot(robot, current_x, "meat")
            robot.turn_by(81, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            angle_corr  = correct_shelf_angle(robot)
            
            approach_mm = get_shelf_approach_mm(robot)

            if approach_mm > 0.0:
                print(f"[SHELF] Too far. Moving forward {approach_mm:.1f} mm")
                robot.move_forward(approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            elif approach_mm < 0.0:
                backup_dist = abs(approach_mm) # Convert -9.0 to a positive 9.0 for the move_backward command
                print(f"[SHELF] Too close. Moving backward {backup_dist:.1f} mm")
                robot.move_backward(backup_dist, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            else:
                print(f"[SHELF] Already at perfect target distance ({SHELF_TARGET_MM} mm). Skipping drive.")

            move_lift(robot, LIFT_PICKUP_TICKS)
            claw_close(robot, CLAW_CLOSE_MEAT_DEG)
            move_lift(robot, LIFT_CARRY_TICKS)
            
            if approach_mm > 0.0:
                robot.move_backward(approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            #elif approach_mm < 0.0:
                #robot.move_forward(-approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)

            robot.turn_by(-(81 + angle_corr), TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

            # 4. PLACE MEAT
            current_x = drive_to_slot(robot, current_x, "bun_bottom")
            robot.turn_by(81, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            angle_corr  = correct_shelf_angle(robot)
            
            approach_mm = get_shelf_approach_mm(robot)

            if approach_mm > 0.0:
                print(f"[SHELF] Too far. Moving forward {approach_mm:.1f} mm")
                robot.move_forward(approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            elif approach_mm < 0.0:
                backup_dist = abs(approach_mm) # Convert -9.0 to a positive 9.0 for the move_backward command
                print(f"[SHELF] Too close. Moving backward {backup_dist:.1f} mm")
                robot.move_backward(backup_dist, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            else:
                print(f"[SHELF] Already at perfect target distance ({SHELF_TARGET_MM} mm). Skipping drive.")

            move_lift(robot, LIFT_PICKUP_TICKS + LIFT_ITEM_THICKNESS_TICKS_1)
            claw_open(robot)
            move_lift(robot, LIFT_CARRY_TICKS)

            if approach_mm > 0.0:
                robot.move_backward(approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            #elif approach_mm < 0.0:
                #robot.move_forward(-approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)            
                
            robot.turn_by(-(81 + angle_corr), TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

            # 5. FETCH TOP BUN
            current_x = drive_to_slot(robot, current_x, "bun_top")
            robot.turn_by(81, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            angle_corr  = correct_shelf_angle(robot)
            
            approach_mm = get_shelf_approach_mm(robot)

            if approach_mm > 0.0:
                print(f"[SHELF] Too far. Moving forward {approach_mm:.1f} mm")
                robot.move_forward(approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            elif approach_mm < 0.0:
                backup_dist = abs(approach_mm) # Convert -9.0 to a positive 9.0 for the move_backward command
                print(f"[SHELF] Too close. Moving backward {backup_dist:.1f} mm")
                robot.move_backward(backup_dist, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            else:
                print(f"[SHELF] Already at perfect target distance ({SHELF_TARGET_MM} mm). Skipping drive.")

            move_lift(robot, LIFT_PICKUP_TICKS)
            claw_close(robot, CLAW_CLOSE_BUN_DEG)
            move_lift(robot, LIFT_CARRY_TICKS)

            if approach_mm > 0.0:
                robot.move_backward(approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            #elif approach_mm < 0.0:
                #robot.move_forward(-approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
                
            robot.turn_by(-(81 + angle_corr), TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

            # 6. PLACE TOP BUN & GRAB FULL STACK
            current_x = drive_to_slot(robot, current_x, "bun_bottom")
            robot.turn_by(81, TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
            angle_corr  = correct_shelf_angle(robot)
            
            approach_mm = get_shelf_approach_mm(robot)

            if approach_mm > 0.0:
                print(f"[SHELF] Too far. Moving forward {approach_mm:.1f} mm")
                robot.move_forward(approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            elif approach_mm < 0.0:
                backup_dist = abs(approach_mm) # Convert -9.0 to a positive 9.0 for the move_backward command
                print(f"[SHELF] Too close. Moving backward {backup_dist:.1f} mm")
                robot.move_backward(backup_dist, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            else:
                print(f"[SHELF] Already at perfect target distance ({SHELF_TARGET_MM} mm). Skipping drive.")

            move_lift(robot, LIFT_PICKUP_TICKS + (LIFT_ITEM_THICKNESS_TICKS_1 + LIFT_ITEM_THICKNESS_TICKS_2))
            claw_open(robot)

            move_lift(robot, LIFT_PICKUP_TICKS)
            claw_close(robot, CLAW_CLOSE_BUN_DEG)
            move_lift(robot, LIFT_CARRY_TICKS)

            if approach_mm > 0.0:
                robot.move_backward(approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
            #elif approach_mm < 0.0:
                #robot.move_forward(-approach_mm, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
                
            robot.turn_by(-(81 + angle_corr), TURN_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

            print("[FSM] Burger pickup complete.")
            state = "BURGER_DONE"

        # ── BURGER_DONE ───────────────────────────────────────────────────────
        # Enable GPS, reset odometry, and immediately begin the course (pure pursuit -> LAPF).
        elif state == "BURGER_DONE":
            print("[FSM] Burger pickup complete — enabling GPS.")
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

            print("[FSM] Resetting odometry and starting course.")
            reset_mission_pose(robot)
            dim_all_leds(robot)
            show_running_leds(robot)
            print_config(robot)
            course_stage_index = 0
            motion_handle = start_course_stage(robot, course_stage_index)
            last_status_print_at = time.monotonic()
            state = "COURSE_MOVING"

        # ── COURSE_MOVING ─────────────────────────────────────────────────────
        # Run pure pursuit first, then LAPF waypoints.
        elif state == "COURSE_MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] COURSE_IDLE — course mission cancelled")
                state = "COURSE_IDLE"

            

            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_course_status(robot, course_stage_index)
                    last_status_print_at = now

                if motion_handle is not None and motion_handle.is_finished():
                    print(f"[FSM] DONE — course stage {course_stage_index + 1}/{len(MISSION_STAGES)} complete")
                    print_course_status(robot, course_stage_index)

                    robot.stop()
                    time.sleep(STAGE_PAUSE_S)

                    course_stage_index += 1
                    if course_stage_index >= len(MISSION_STAGES):
                        motion_handle = None
                        show_idle_leds(robot)
                        print("[FSM] Pure Pursuit course complete.")
                        state = "FACE RECOGNITION"
                    else:
                        motion_handle = start_course_stage(robot, course_stage_index)
                        last_status_print_at = time.monotonic()

        elif state == "FACE RECOGNITION":
            
            
            if robot.was_button_pressed(Button.BTN_2):
                    cancel_motion(robot, motion_handle)
                    motion_handle = None
                    show_idle_leds(robot)
                    print("[FSM] COURSE_IDLE — course mission cancelled")
                    state = "COURSE_IDLE"
            
                
            else:
                _, x, y, theta = get_best_pose(robot)
                print(f"[FACE] Odometry theta: {theta:.1f}")
                
                # 1. Use odometry to roughly face the wall (unwind wheel slip)
                robot.turn_by(-(theta), 15, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
                time.sleep(0.5)
                
                # 2. Perfect the alignment using LiDAR regression!
                correct_far_wall_angle(robot)


                 # Overwrite the 400mm limit just for this final check!
                from robot.hardware_map import LIDAR_RANGE_MIN_MM, LIDAR_FOV_DEG

                robot.set_lidar_filter(
                    range_min_mm=LIDAR_RANGE_MIN_MM,
                    range_max_mm=1200.0,  
                    fov_deg=LIDAR_FOV_DEG,
                )
                
                print("[NAV] Waiting for Lidar to settle...")
                time.sleep(0.3) 
                
                # --- BULLETPROOF RETRY LOOP ---
                distance_to_wall = -1.0
                max_retries = 3
                attempt = 0
                
                while distance_to_wall < 0.0 and attempt < max_retries:
                    print(f"\n[NAV] Wall Measurement Attempt {attempt + 1}/{max_retries}...")
                    
                    # Increased to 10 samples (takes 1 full second of scanning)
                    distance_to_wall = get_robust_wall_distance(robot, num_samples=10, delay_s=0.1)
                    
                    if distance_to_wall < 0.0:
                        print("[WARNING] Lidar completely missed the wall. Retrying in 1 second...")
                        time.sleep(1.0) # Give the sensor/environment a moment to clear
                    
                    attempt += 1
                
                # --- FINAL SAFETY CHECK & MOVEMENT ---
                if distance_to_wall > 340.0:
                    robot.move_forward(distance_to_wall - 340, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
                elif distance_to_wall < 340.0:
                    robot.move_backward(-(distance_to_wall-340), APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)


                time.sleep(0.3)

                

                # 3. Turn exactly 8 degrees for the camera angle
                robot.turn_by(10, 15, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
                time.sleep(1.5)

                # ── 3. Face detection ──────────────────────────────────────────

                # ── 3. Face detection ──────────────────────────────────────────
                print("[FACE] Starting customer detection...")
                customer = detect_customer(robot)
    
                travel_mm   = CUSTOMER_A_TRAVEL_MM   if customer == "A" else CUSTOMER_B_TRAVEL_MM
                to_stop_mm  = CUSTOMER_A_TO_STOP_MM  if customer == "A" else CUSTOMER_B_TO_STOP_MM
                print(f"[FACE] Customer {customer} ({'Female' if customer == 'A' else 'Male'}) "
                      f"— travel {travel_mm:.0f} mm to customer row")

                
                # ── 4. Turn right to face -Y ─────────────────────────────────── 
                robot.turn_by(-10, 15, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG) 
                
                print("[NAV] Temporarily boosting Lidar range to 2000mm to see the far wall...")
                from robot.hardware_map import LIDAR_RANGE_MIN_MM, LIDAR_FOV_DEG
                
                # Overwrite the 400mm limit just for this final check!
                robot.set_lidar_filter(
                    range_min_mm=LIDAR_RANGE_MIN_MM,
                    range_max_mm=1000.0,  
                    fov_deg=LIDAR_FOV_DEG,
                )
                
                print("[NAV] Waiting for Lidar to settle...")
                time.sleep(0.3) 
                
                # --- BULLETPROOF RETRY LOOP ---
                distance_to_wall = -1.0
                max_retries = 3
                attempt = 0
                
                while distance_to_wall < 0.0 and attempt < max_retries:
                    print(f"\n[NAV] Wall Measurement Attempt {attempt + 1}/{max_retries}...")
                    
                    # Increased to 10 samples (takes 1 full second of scanning)
                    distance_to_wall = get_robust_wall_distance(robot, num_samples=10, delay_s=0.1)
                    
                    if distance_to_wall < 0.0:
                        print("[WARNING] Lidar completely missed the wall. Retrying in 1 second...")
                        time.sleep(1.0) # Give the sensor/environment a moment to clear
                    
                    attempt += 1
                
                # --- FINAL SAFETY CHECK & MOVEMENT ---
                if distance_to_wall > 100.0:
                    robot.move_forward(distance_to_wall - 45, APPROACH_VELOCITY, POS_TOLERANCE_MM, blocking=True)
                else:
                    print("[ERROR] Wall not found after all retries or too close! Halting.")
                    robot.stop()

                time.sleep(0.3)
                robot.turn_by(-70, ANGULAR_VELOCITY_DEG, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
                time.sleep(0.3)
                drive_with_wall_following(robot, travel_mm, DRIVE_VELOCITY)

                
                time.sleep(0.5)
                robot.turn_by(72, 20, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)
                robot.move_forward(55, DRIVE_VELOCITY, POS_TOLERANCE_MM, blocking=True)

                move_lift(robot, LIFT_PICKUP_TICKS)
                claw_open(robot)
                move_lift(robot, LIFT_CARRY_TICKS)
                robot.move_backward(65, DRIVE_VELOCITY, POS_TOLERANCE_MM, blocking=True)
                robot.turn_by(-68, 20, tolerance_deg=TURN_TOLERANCE_DEG, blocking=True)

                drive_until_stop_sign(robot, to_stop_mm)

                


                state = "COURSE_IDLE"
                print("END OF COURSE")
                

        # ── COURSE_IDLE ───────────────────────────────────────────────────────
        # Resting state after course completion or cancellation.
        elif state == "COURSE_IDLE":
            robot.stop()
            time.sleep(1.0)

        # ── Tick-rate control ─────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

