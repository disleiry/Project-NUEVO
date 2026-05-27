"""
main_delivery_stage.py
Combines pure pursuit, gender detection, and dynamic burger drop-off.
Starts where the obstacle course ends.
"""

from __future__ import annotations
import sys
import time
import cv2
from typing import Any

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    Motor,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    ServoChannel,
    WHEEL_BASE,
    WHEEL_DIAMETER,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline

# Import gender detector
sys.path.insert(0, '/ros2_ws/src/vision')
try:
    from vision.gender_detection import GenderDetector
except ImportError:
    print("Warning: GenderDetector not found in /ros2_ws/src/vision.")
    GenderDetector = None


# ==========================================
# CONSTANTS & PARAMETERS
# ==========================================

# --- SENSOR & GPS SETTINGS ---
ENABLE_GPS = True
TAG_ID = 25
GPS_POSITION_ALPHA = 0.20
ENABLE_GPS_TANGENT_HEADING = True
GPS_TANGENT_ALPHA = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0

# --- LIFT AND CLAW SETTINGS ---
LIFT_MOTOR = Motor.DC_M3
LIFT_CARRY_TICKS = -12800  
LIFT_PICKUP_TICKS = -8300  
LIFT_MAX_VEL = 1800
LIFT_TOLERANCE = 30
CLAW_SERVO = ServoChannel.CH_13
CLAW_OPEN_DEG = 60.0
CLAW_CLOSE_DEG = 111.0

# --- NAVIGATION SETTINGS ---
DRIVE_VELOCITY = 150.0
APPROACH_VELOCITY = 60.0
APPROACH_SHELF_DIST = 18.0   
TURN_TO_SHELF_DEG = 79.0     
TURN_FROM_SHELF_DEG = -79.0  
TURN_TOLERANCE_DEG = 2.0

# --- PURE PURSUIT SETTINGS ---
PURE_PURSUIT_VELOCITY_MM_S = 150.0
LOOKAHEAD_MM = 225.0
PURE_PURSUIT_TOLERANCE_MM = 25.0
ADVANCE_RADIUS_MM = 75.0
PURE_PURSUIT_MAX_ANGULAR_RAD_S = 1.5

# ==========================================
# WAYPOINT PATHS THROUGH THE ARENA
# ==========================================

# 1. Travel forward in positive X axis to the camera station.
STATION_CONTROL_POINTS = [
    (2000.0, 3700.0),
    (2500.0, 3700.0),
]

# 2A. Dropoff path for Customer A (Female)
# (ADJUST THE Y=2000 TO MATCH YOUR FIELD)
CUSTOMER_A_DROPOFF_POINTS = [
    (2500.0, 3700.0),
    (2500.0, 1700.0), 
]

# 2B. Dropoff path for Customer B (Male)
# (ADJUST THE Y=1000 TO MATCH YOUR FIELD)
CUSTOMER_B_DROPOFF_POINTS = [
    (2500.0, 3700.0),
    (2500.0, 1300.0), 
]

# Densify segments for smoother tracking 
STATION_CONTROL_POINTS = densify_polyline(STATION_CONTROL_POINTS, spacing=100.0)
CUSTOMER_A_DROPOFF_POINTS = densify_polyline(CUSTOMER_A_DROPOFF_POINTS, spacing=100.0)
CUSTOMER_B_DROPOFF_POINTS = densify_polyline(CUSTOMER_B_DROPOFF_POINTS, spacing=100.0)


# ==========================================
# HARDWARE / ROBOT CONFIGURATION
# ==========================================

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
    
    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
        print(f"[sensor] GPS enabled - tracking ArUco tag {TAG_ID}")

        if ENABLE_GPS_TANGENT_HEADING:
            robot.enable_gps_tangent_heading(
                alpha=GPS_TANGENT_ALPHA,
                min_displacement_mm=GPS_TANGENT_MIN_DISPLACEMENT_MM,
            )
            print("[sensor] GPS tangent heading enabled")


# ==========================================
# HELPERS
# ==========================================

def claw_close(robot: Robot) -> None:
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_CLOSE_DEG)
    time.sleep(0.5)

def claw_open(robot: Robot) -> None:
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_OPEN_DEG)
    time.sleep(0.5)

def detect_customer_gender() -> str:
    """Takes 15 frames to process a biased vote for Male vs. Female."""
    if not GenderDetector:
        return "Unknown"
        
    detector = GenderDetector()
    cap = cv2.VideoCapture('/dev/video10')
    if not cap.isOpened():
        print("[ERROR] Cannot open /dev/video10")
        return "Unknown"

    votes = {"Male": 0, "Female": 0}
    
    for _ in range(15):
        ret, frame = cap.read()
        if not ret:
            continue
        gender, conf = detector.detect(frame)
        if gender:
            votes[gender] += 1
        time.sleep(0.1)

    cap.release()

    if votes["Male"] == 0 and votes["Female"] == 0:
        return "Unknown"

    # Biased check: 3 or more Female votes means it's the Female customer
    if votes["Female"] >= 3:
        return "Female"
    else:
        return "Male"

def start_pure_pursuit_stage(robot: Robot, waypoints: list[tuple[float, float]]):
    print(f"[FSM] MOVING pure pursuit stage with {len(waypoints)} waypoints.")
    return robot.purepursuit_follow_path(
        waypoints=waypoints,
        velocity=PURE_PURSUIT_VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=PURE_PURSUIT_TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=PURE_PURSUIT_MAX_ANGULAR_RAD_S,
        blocking=False,
    )


# ==========================================
# MAIN MISSION LOGIC
# ==========================================

def run(robot: Robot) -> None:
    configure_robot(robot)
    state = "INIT"
    motion_handle = None
    timer = 0.0

    print("=====================================================")
    print(" DELIVERY COURSE STARTING")
    print(" Press BTN_1 to begin.")
    print("=====================================================")

    while True:
        if state == "INIT":
            if robot.was_button_pressed(Button.BTN_1):
                robot.reset_odometry()
                robot.wait_for_odometry_reset(timeout=2.0)
                
                claw_close(robot)
                robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                robot.set_motor_position(LIFT_MOTOR, LIFT_CARRY_TICKS, max_vel_ticks=LIFT_MAX_VEL)
                
                motion_handle = start_pure_pursuit_stage(robot, STATION_CONTROL_POINTS)
                state = "NAV_TO_STATION"

        elif state == "NAV_TO_STATION":
            if motion_handle.is_done():
                print("[FSM] Arrived at camera station.")
                robot.stop()
                state = "DETECT_CUSTOMER"

        elif state == "DETECT_CUSTOMER":
            print("[VISION] Beginning 15-frame gender detection...")
            result = detect_customer_gender()
            
            # Decide which waypoint path to use based on the camera
            if result == "Female":
                customer = "Customer A (Female)"
                active_dropoff_points = CUSTOMER_A_DROPOFF_POINTS
            else:
                customer = "Customer B (Male)"
                active_dropoff_points = CUSTOMER_B_DROPOFF_POINTS

            print(f"[DETECTED] {result} -> Deliver to {customer}")
            
            print("[FSM] Turning Right 90 degrees to face down Y-axis...")
            robot.turn_by(delta_deg=-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
            
            print("[FSM] Disabling GPS. Relying strictly on Odometry/Pure Pursuit.")
            if ENABLE_GPS:
                robot.disable_gps()
            
            # Start pure pursuit using the dynamically selected path
            motion_handle = start_pure_pursuit_stage(robot, active_dropoff_points)
            state = "NAV_TO_DROPOFF"

        elif state == "NAV_TO_DROPOFF":
            if motion_handle.is_done():
                print(f"[FSM] Reached {customer} drop-off spot on Y-axis.")
                robot.stop()
                
                print(f"[FSM] Turning Left {TURN_TO_SHELF_DEG} degrees to face shelf.")
                robot.turn_by(delta_deg=TURN_TO_SHELF_DEG, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
                
                print(f"[FSM] Lowering lift to drop-off height: {LIFT_PICKUP_TICKS}")
                robot.set_motor_position(
                    LIFT_MOTOR, LIFT_PICKUP_TICKS, 
                    max_vel_ticks=LIFT_MAX_VEL, tolerance_ticks=LIFT_TOLERANCE, 
                    blocking=True, timeout=5.0
                )
                
                print(f"[FSM] Moving towards shelf for deposit: {APPROACH_SHELF_DIST}mm")
                robot.move_forward(distance=APPROACH_SHELF_DIST, velocity=APPROACH_VELOCITY, blocking=True)
                state = "DROP_BURGER"

        elif state == "DROP_BURGER":
            print("[FSM] Opening claw to release burger...")
            claw_open(robot)

            print("[FSM] Retreating from shelf...")
            robot.move_backward(distance=APPROACH_SHELF_DIST, velocity=APPROACH_VELOCITY, blocking=True)

            print("[FSM] Raising arm back to travel height.")
            robot.set_motor_position(
                LIFT_MOTOR, LIFT_CARRY_TICKS, 
                max_vel_ticks=LIFT_MAX_VEL, tolerance_ticks=LIFT_TOLERANCE, 
                blocking=True, timeout=5.0
            )
            claw_close(robot)

            print(f"[FSM] Turning Right ({TURN_FROM_SHELF_DEG} degrees) to continue down Y-axis...")
            robot.turn_by(delta_deg=TURN_FROM_SHELF_DEG, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
            
            print("[FSM] Driving forward indefinitely until stop sign is seen...")
            robot.set_wheels_velocity(DRIVE_VELOCITY, DRIVE_VELOCITY)
            state = "FIND_STOP_SIGN"

        elif state == "FIND_STOP_SIGN":
            if robot.get_detections("stop sign"):
                print("[VISION] Stop sign detected! Stopping...")
                robot.stop()
                timer = time.monotonic()
                state = "WAIT_STOP_SIGN"

        elif state == "WAIT_STOP_SIGN":
            if time.monotonic() - timer >= 2.0:
                print("[FSM] 2-second wait complete. Moving forward 610mm.")
                robot.move_forward(distance=610.0, velocity=DRIVE_VELOCITY, blocking=True)
                print("[FSM] Final position reached. Stopping entirely.")
                state = "DONE"

        elif state == "DONE":
            robot.stop()
            time.sleep(0.1)

        time.sleep(1.0 / DEFAULT_FSM_HZ)

if __name__ == "__main__":
    from robot.robot import Robot
    print("Place inside ros2_ws and run using standard parameters.")
