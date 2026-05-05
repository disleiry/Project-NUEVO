"""
main.py — Vision Node Lab Tasks 3, 4, and 5
=============================================
Task 3 (Deliverable i):  Traffic light LED demo.
Task 4 (Deliverable ii): Drive forward on green, stop on red / no signal.
Task 5 (Deliverable iii, Bonus): Stop sign overrides everything.

HOW TO RUN
----------
Terminal A — start the vision node:
    ros2 launch vision vision_production.launch.py

Terminal B — run the robot node:
    ros2 run robot robot
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LEDMode,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

LED_BRIGHTNESS          = 255
LIGHT_HOLD_SEC          = 2.0    # seconds to keep LEDs on after last detection
VISION_STALE_SEC        = 3.0    # seconds before vision is considered stale
MIN_TRAFFIC_CONFIDENCE  = 0.50   # minimum YOLO confidence for traffic light
DRIVE_LINEAR_SPEED      = 100.0  # mm/s forward speed (Task 4)


# ---------------------------------------------------------------------------
# Helpers  (unchanged from traffic_light_leds.py except configure_robot)
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    # Task 4 requires odometry parameters to be set
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


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


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


def find_traffic_light_color(robot: Robot) -> str | None:
    """Return the highest-confidence red/green traffic-light color, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best_color      = None
    best_confidence = -1.0

    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
        if confidence < MIN_TRAFFIC_CONFIDENCE:
            continue
        attributes      = detection.get("attributes", {})
        color_attribute = attributes.get("color", {})
        color           = color_attribute.get("value")
        if color not in ("red", "green"):
            continue
        if confidence > best_confidence:
            best_confidence = confidence
            best_color      = str(color)

    return best_color


# ---------------------------------------------------------------------------
# run() — entry point called by robot_node.py
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state           = "INIT"
    lights_off_at   = 0.0
    last_shown_color = None

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # ── INIT ────────────────────────────────────────────────────────────
        if state == "INIT":
            start_robot(robot)
            dim_all_leds(robot)
            print("[FSM] WATCHING — show a red or green traffic light")
            state = "WATCHING"

        # ── WATCHING ─────────────────────────────────────────────────────────
        elif state == "WATCHING":
            now = time.monotonic()

            # ── Task 5 (Bonus): stop sign takes absolute priority ────────────
            if robot.get_detections("stop sign"):
                robot.stop()
                robot.set_led(LED.RED, LED_BRIGHTNESS, mode=LEDMode.BLINK, period_ms=500)
                robot.set_led(LED.GREEN, 0)
                last_shown_color = None
                lights_off_at    = 0.0
                print("[VISION] stop sign — robot stopped, red LED blinking")

            else:
                # ── Tasks 3 & 4: traffic light logic ─────────────────────────
                traffic_light_color = find_traffic_light_color(robot)

                if traffic_light_color in ("red", "green"):
                    # Task 3: mirror LED
                    show_traffic_light_color(robot, traffic_light_color)
                    lights_off_at = now + LIGHT_HOLD_SEC

                    if traffic_light_color != last_shown_color:
                        print(f"[VISION] traffic light: {traffic_light_color}")
                    last_shown_color = traffic_light_color

                    # Task 4: drive on green, stop on red
                    if traffic_light_color == "green":
                        robot.set_velocity(DRIVE_LINEAR_SPEED, 0)
                    else:
                        robot.stop()

                elif lights_off_at > 0.0 and now >= lights_off_at:
                    # Hold expired — no recent detection
                    robot.stop()
                    dim_all_leds(robot)
                    lights_off_at = 0.0
                    if last_shown_color is not None:
                        print("[VISION] no recent red/green light — LEDs off, robot stopped")
                    last_shown_color = None

        # ── Tick-rate control (do not modify) ────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
