"""
main.py — traffic-light start + pure pursuit + LAPF obstacle navigation
========================================================================

Mission order:
1. Robot starts facing forward and DOES NOT move forward.
2. Robot turns left in place until the camera sees a traffic light.
3. Robot stops and waits until the traffic light is green.
4. Once green is detected, robot turns back to its original forward heading.
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
LIGHT_LOST_SEARCH_DELAY_SEC = 1.0
MIN_TRAFFIC_CONFIDENCE = 0.50

# The robot turns in place to find the traffic light.
# Positive/negative direction depends on your robot coordinate convention.
# If it turns right instead of left, flip the sign.
SCAN_LEFT_ANGULAR_RAD_S = 0.25

# After green, the robot turns back to the heading it had before scanning.
RETURN_TO_FORWARD_TOLERANCE_DEG = 2.0
RETURN_TO_FORWARD_ANGULAR_RAD_S = 0.35

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

        if abs(error_deg) <= self.tolerance_deg:
            self.robot.stop()
            self.done = True
            return True

        direction = 1.0 if error_deg > 0.0 else -1.0
        angular_command = direction * self.angular_rad_s

        # Slow down near the target to reduce overshoot.
        if abs(error_deg) < 8.0:
            angular_command *= 0.45

        self.robot.set_velocity(0.0, angular_command)
        return False


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

    raise ValueError(f"Unknown stage type: {stage['type']}")


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

    print("[CFG] Traffic-light start:")
    print(
        f"      scan_left_angular={SCAN_LEFT_ANGULAR_RAD_S:.2f} rad/s, "
        f"return_angular={RETURN_TO_FORWARD_ANGULAR_RAD_S:.2f} rad/s, "
        f"return_tolerance={RETURN_TO_FORWARD_TOLERANCE_DEG:.1f}°"
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

    state = "INIT"
    course_stage_index = 0
    motion_handle = None
    forward_theta_deg = None
    last_status_print_at = 0.0
    light_lost_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            dim_all_leds(robot)
            show_idle_leds(robot)
            robot.stop()
            print_config(robot)
            print("[FSM] IDLE — press BTN_1 to start traffic-light/course mission")
            state = "IDLE"

        # ── IDLE ─────────────────────────────────────────────────────────────
        elif state == "IDLE":
            robot.stop()
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                _, _, forward_theta_deg = robot.get_odometry_pose()
                dim_all_leds(robot)
                show_running_leds(robot)
                print(
                    f"[FSM] SEARCH_LIGHT — turning left in place from forward θ={forward_theta_deg:.1f}°"
                )
                state = "SEARCH_LIGHT"

        # ── SEARCH_LIGHT ─────────────────────────────────────────────────────
        # Turn left in place. Do NOT move forward.
        elif state == "SEARCH_LIGHT":
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — mission cancelled while searching for traffic light")
                state = "IDLE"

            elif stop_sign_detected(robot):
                robot.stop()
                robot.set_led(LED.RED, LED_BRIGHTNESS, mode=LEDMode.BLINK, period_ms=500)
                robot.set_led(LED.GREEN, 0)
                print("[VISION] stop sign detected — stopped")

            else:
                traffic_light_color = find_traffic_light_color(robot)

                if traffic_light_color in ("red", "green"):
                    robot.stop()
                    show_traffic_light_color(robot, traffic_light_color)
                    light_lost_at = 0.0
                    print(f"[VISION] traffic light detected: {traffic_light_color} — waiting for green")
                    state = "WAIT_FOR_GREEN"
                else:
                    # Keep rotating left in place until the traffic light enters the camera view.
                    robot.set_velocity(0.0, SCAN_LEFT_ANGULAR_RAD_S)

        # ── WAIT_FOR_GREEN ───────────────────────────────────────────────────
        # Stay stopped. Red = wait. Green = turn back to original forward heading.
        elif state == "WAIT_FOR_GREEN":
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — mission cancelled while waiting for green")
                state = "IDLE"

            elif stop_sign_detected(robot):
                robot.stop()
                robot.set_led(LED.RED, LED_BRIGHTNESS, mode=LEDMode.BLINK, period_ms=500)
                robot.set_led(LED.GREEN, 0)
                print("[VISION] stop sign detected — stopped")

            else:
                robot.stop()
                traffic_light_color = find_traffic_light_color(robot)

                if traffic_light_color == "green":
                    show_traffic_light_color(robot, "green")
                    print("[VISION] green light — turning back to forward heading")
                    if forward_theta_deg is None:
                        _, _, forward_theta_deg = robot.get_odometry_pose()
                    motion_handle = TurnToHeadingHandle(
                        robot=robot,
                        target_theta_deg=forward_theta_deg,
                        tolerance_deg=RETURN_TO_FORWARD_TOLERANCE_DEG,
                        angular_rad_s=RETURN_TO_FORWARD_ANGULAR_RAD_S,
                    )
                    last_status_print_at = now
                    state = "RETURN_TO_FORWARD"

                elif traffic_light_color == "red":
                    show_traffic_light_color(robot, "red")
                    light_lost_at = 0.0
                    print("[VISION] red light — waiting")

                else:
                    # If detection is lost, stay stopped briefly, then scan again.
                    if light_lost_at == 0.0:
                        light_lost_at = now
                    elif now - light_lost_at >= LIGHT_LOST_SEARCH_DELAY_SEC:
                        print("[VISION] traffic light lost — scanning left again")
                        state = "SEARCH_LIGHT"

        # ── RETURN_TO_FORWARD ────────────────────────────────────────────────
        # Turn in place back to the heading the robot had before scanning.
        elif state == "RETURN_TO_FORWARD":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE — mission cancelled while returning to forward")
                state = "IDLE"

            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    _, _, theta = robot.get_odometry_pose()
                    target_theta = getattr(motion_handle, "target_theta_deg", forward_theta_deg)
                    print(
                        f"  returning to forward: θ={theta:.1f}° target={target_theta:.1f}°"
                    )
                    last_status_print_at = now

                if motion_handle is not None and motion_handle.is_finished():
                    robot.stop()
                    print("[FSM] Forward heading restored — resetting odometry and starting course")
                    reset_mission_pose(robot)
                    course_stage_index = 0
                    motion_handle = start_course_stage(robot, course_stage_index)
                    last_status_print_at = time.monotonic()
                    state = "COURSE_MOVING"

        # ── COURSE_MOVING ───────────────────────────────────────────────────
        # Run pure pursuit first, then LAPF waypoints.
        elif state == "COURSE_MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE — course mission cancelled")
                state = "IDLE"

            elif ENABLE_STOP_SIGN_OVERRIDE and stop_sign_detected(robot):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                robot.set_led(LED.RED, LED_BRIGHTNESS, mode=LEDMode.BLINK, period_ms=500)
                robot.set_led(LED.GREEN, 0)
                print("[VISION] stop sign detected during course — mission stopped")
                state = "IDLE"

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
                        print("[FSM] IDLE — full traffic-light/course mission complete. Press BTN_1 to run again")
                        state = "IDLE"
                    else:
                        motion_handle = start_course_stage(robot, course_stage_index)
                        last_status_print_at = time.monotonic()

        # ── Tick-rate control ────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
