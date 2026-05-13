"""
main.py — waypoint-guided LAPF arena navigation
===============================================

This combines the waypoint idea from pure pursuit with the obstacle-avoidance
behavior from LAPF:

- PATH_CONTROL_POINTS describe the intended path through the arena.
- The robot drives to each waypoint using lapf_to_goal().
- LiDAR lets LAPF move around obstacles while still progressing toward the
  current waypoint.
- GPS/fused pose is enabled for global position feedback.

How to run:
    cp main.py ros2_ws/src/robot/main.py
    ros2 run robot robot

Controls:
    BTN_1 starts the waypoint mission.
    BTN_2 cancels the active motion and returns to IDLE.
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
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

ENABLE_LIDAR = True
ENABLE_GPS = True

# IMPORTANT: update this to match the ArUco marker ID on your robot.
TAG_ID = 25

# GPS tuning from the pure-pursuit example.
GPS_POSITION_ALPHA = 0.05
ENABLE_GPS_TANGENT_HEADING = True
GPS_TANGENT_ALPHA = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0


# ---------------------------------------------------------------------------
# Waypoint path through the arena
# ---------------------------------------------------------------------------
# Units are millimeters.
# With INITIAL_THETA_DEG = 90, +Y is usually the robot's initial forward direction.
#
# These waypoints should trace the intended path. The robot will still be able
# to move around obstacles because each waypoint is reached using LAPF.
#
# Edit these to match your course measurements.

PATH_CONTROL_POINTS = [
    #(0.0, 0.0),        # start
    (0.0, 3250.0),      # Waypoint 1: home straight
    (600.0, 3250.0)
    (600.0, 100.0) 
    (1200.0, 100.0),    # entrance toward obstacle course
    (1200.0, 3250.0),   # end obstacle course
]

# Optional: densify long segments for smoother tracking.
PATH_CONTROL_POINTS = densify_polyline(PATH_CONTROL_POINTS, spacing=50.0)


# ---------------------------------------------------------------------------
# LAPF movement tuning
# ---------------------------------------------------------------------------

VELOCITY_MM_S = 150.0
TOLERANCE_MM = 50.0
MAX_ANGULAR_RAD_S = 0.6

# LAPF obstacle avoidance tuning.
LEASH_LENGTH_MM = 50.0
REPULSION_RANGE_MM = 300.0
TARGET_SPEED_MM_S = 200.0
REPULSION_GAIN = 550.0
ATTRACTION_GAIN = 1.0
FORCE_EMA_ALPHA = 0.35
INFLATION_MARGIN_MM = 150.0
LEASH_HALF_ANGLE_DEG = 25.0

STATUS_PRINT_INTERVAL_S = 0.5
STAGE_PAUSE_S = 0.10


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


def cancel_motion(robot: Robot, handle) -> None:
    if handle is not None:
        handle.cancel()
        handle.wait(timeout=1.0)
    robot.stop()


def get_best_pose(robot: Robot) -> tuple[str, float, float, float]:
    if ENABLE_GPS and robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        return "fused", x, y, theta

    x, y, theta = robot.get_odometry_pose()
    return "odom ", x, y, theta


def print_status(robot: Robot, waypoint_index: int) -> None:
    label, x, y, theta = get_best_pose(robot)
    goal_x, goal_y = PATH_CONTROL_POINTS[waypoint_index]

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
        f"  waypoint {waypoint_index + 1}/{len(PATH_CONTROL_POINTS)} goal=({goal_x:.0f}, {goal_y:.0f}) mm | "
        f"{label}=({x:6.0f}, {y:6.0f}) mm θ={theta:5.1f}°"
        f"{gps_summary}{vt_summary}{track_summary}"
    )


def start_waypoint(robot: Robot, waypoint_index: int):
    cfg = resolve_lapf_config()
    goal_x, goal_y = PATH_CONTROL_POINTS[waypoint_index]

    print(
        f"[FSM] MOVING — waypoint {waypoint_index + 1}/{len(PATH_CONTROL_POINTS)} "
        f"goal=({goal_x:.0f}, {goal_y:.0f}) mm"
    )

    return robot.lapf_to_goal(
        goal_x,
        goal_y,
        velocity=VELOCITY_MM_S,
        tolerance=TOLERANCE_MM,
        leash_length_mm=cfg["leash_length_mm"],
        repulsion_range_mm=cfg["repulsion_range_mm"],
        target_speed_mm_s=cfg["target_speed_mm_s"],
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        repulsion_gain=cfg["repulsion_gain"],
        attraction_gain=cfg["attraction_gain"],
        force_ema_alpha=cfg["force_ema_alpha"],
        inflation_margin_mm=cfg["inflation_margin_mm"],
        leash_half_angle_deg=cfg["leash_half_angle_deg"],
        blocking=False,
    )


def print_config(robot: Robot) -> None:
    lapf_cfg = resolve_lapf_config()

    print("[CFG] Waypoint mission:")
    for i, waypoint in enumerate(PATH_CONTROL_POINTS, start=1):
        print(f"      {i:02d}: ({waypoint[0]:.0f}, {waypoint[1]:.0f}) mm")

    print(
        f"[CFG] velocity={VELOCITY_MM_S:.0f} mm/s tolerance={TOLERANCE_MM:.0f} mm "
        f"max_angular={MAX_ANGULAR_RAD_S:.2f} rad/s"
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


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None
    waypoint_index = 0
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print_config(robot)
            print("[FSM] IDLE — press BTN_1 to start waypoint/LAPF mission, BTN_2 to cancel")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_running_leds(robot)
                waypoint_index = 0
                motion_handle = start_waypoint(robot, waypoint_index)
                last_status_print_at = now
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE — waypoint mission cancelled")
                state = "IDLE"

            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot, waypoint_index)
                    last_status_print_at = now

                if motion_handle is not None and motion_handle.is_finished():
                    print(f"[FSM] DONE — waypoint {waypoint_index + 1}/{len(PATH_CONTROL_POINTS)} reached")
                    print_status(robot, waypoint_index)

                    robot.stop()
                    time.sleep(STAGE_PAUSE_S)

                    waypoint_index += 1
                    if waypoint_index >= len(PATH_CONTROL_POINTS):
                        motion_handle = None
                        show_idle_leds(robot)
                        print("[FSM] IDLE — full waypoint/LAPF mission complete. Press BTN_1 to run again")
                        state = "IDLE"
                    else:
                        motion_handle = start_waypoint(robot, waypoint_index)
                        last_status_print_at = time.monotonic()

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
