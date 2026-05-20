"""
main.py — fixed traffic-light turn + pure pursuit + LAPF obstacle navigation
============================================================================

Mission order:
1. Robot starts facing forward and DOES NOT move forward.
2. Robot turns in place by a fixed angle, usually 15 degrees left, so the
   camera can face the traffic light.
3. Robot stops and waits for the traffic light to be green.
4. Once green is detected, robot turns back to the original forward heading.
5. Robot resets odometry and begins the course:
   - Pure pursuit for the straight/ramp section.
   - LAPF + LiDAR for the obstacle-course section.

How to run:
    cp main.py ros2_ws/src/robot/main.py
    ros2 run robot robot

Make sure these are running in separate terminals as needed:
    ros2 launch vision vision_production.launch.py
    ros2 run robot bridge
    ros2 run robot robot

Controls:
    BTN_1 starts the full mission.
    BTN_2 cancels and returns to IDLE.
"""

from __future__ import annotations

import time
from typing import Any

from robot.hardware_map import (
    Button,
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
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
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

ENABLE_VISION = True
ENABLE_LIDAR = True
ENABLE_GPS = True

# IMPORTANT: update this to match the ArUco marker ID on your robot.
TAG_ID = 25

# GPS tuning.
GPS_POSITION_ALPHA = 0.01
ENABLE_GPS_TANGENT_HEADING = True
GPS_TANGENT_ALPHA = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0


# ---------------------------------------------------------------------------
# Traffic-light start behavior
# ---------------------------------------------------------------------------

LED_BRIGHTNESS = 255
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_CONFIDENCE = 0.50

# Fixed traffic-light viewing turn.
# The robot turns this many degrees in place, then stops and waits.
# If the robot turns the wrong direction, flip the sign to -15.0.
TRAFFIC_LIGHT_TURN_DEG = 15.0

# Speed for the fixed 15-degree turn and the turn back to forward.
# 1.2 rad/s is about 69 deg/s, so 15 degrees should take about 0.22 s.
TRAFFIC_LIGHT_TURN_RAD_S = 1.2
RETURN_TO_FORWARD_RAD_S = 1.2
TURN_TOLERANCE_DEG = 4.0

# Stop sign safety override from the traffic-light example.
ENABLE_STOP_SIGN_OVERRIDE = True


# ---------------------------------------------------------------------------
# Waypoint paths through the arena
# ---------------------------------------------------------------------------
# Units are millimeters.
# With INITIAL_THETA_DEG = 90, +Y is usually the robot's initial forward direction.

PURE_PURSUIT_CONTROL_POINTS = [
    #(0.0, 0.0),        # start
    (0.0, 3250.0),      # Waypoint 1: home straight
    (600.0, 3250.0),    # Waypoint 2: transition / turn
    (600.0, 100.0),     # Waypoint 3: ramp / return direction
    (1200.0, 100.0),    # Waypoint 4: entrance toward obstacle course
]

# Optional: densify long pure-pursuit segments for smoother tracking.
PURE_PURSUIT_CONTROL_POINTS = densify_polyline(PURE_PURSUIT_CONTROL_POINTS, spacing=100.0)

# LAPF is only used in the obstacle-course section.
LAPF_CONTROL_POINTS = [
    (1200.0, 3250.0),   # Obstacle waypoint / finish
]

# Optional: densify LAPF segments so the obstacle-course path has intermediate goals.
LAPF_CONTROL_POINTS = densify_polyline(LAPF_CONTROL_POINTS, spacing=100.0)


# ---------------------------------------------------------------------------
# Pure pursuit tuning for straight/ramp sections
# ---------------------------------------------------------------------------

PURE_PURSUIT_VELOCITY_MM_S = 150.0
LOOKAHEAD_MM = 120.0
PURE_PURSUIT_TOLERANCE_MM = 25.0
ADVANCE_RADIUS_MM = 80.0
PURE_PURSUIT_MAX_ANGULAR_RAD_S = 1.5


# ---------------------------------------------------------------------------
# LAPF tuning for obstacle-course section
# ---------------------------------------------------------------------------

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
# Motion helpers
# ---------------------------------------------------------------------------

def cancel_motion(robot: Robot, handle) -> None:
    if handle is not None:
        handle.cancel()
        handle.wait(timeout=1.0)
    robot.stop()


class TurnToHeadingHandle:
    """Non-blocking in-place turn handle using robot.set_velocity(0, omega)."""

    def __init__(
        self,
        robot: Robot,
        target_theta_deg: float,
        tolerance_deg: float,
        angular_rad_s: float,
    ) -> None:
        self.robot = robot
        self.target_theta_deg = normalize_angle_deg(float(target_theta_deg))
        self.tolerance_deg = float(tolerance_deg)
        self.angular_rad_s = abs(float(angular_rad_s))
        self.cancelled = False
        self.done = False

    def cancel(self) -> None:
        self.cancelled = True
        self.robot.stop()

    def wait(self, timeout: float | None = None) -> bool:
        self.robot.stop()
        return True

    def is_finished(self) -> bool:
        if self.cancelled or self.done:
            self.robot.stop()
            return True

        _, _, current_theta = self.robot.get_odometry_pose()
        error_deg = normalize_angle_deg(self.target_theta_deg - float(current_theta))

        if abs(error_deg) <= self.t
