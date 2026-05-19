"""
test_burger_pickup.py — Burger Pickup Isolated Test
====================================================
Runs ONLY Section 1 of the full mission so you can test and tune the
pickup sequence independently, without the ramp / obstacle / drop-off code
getting in the way.

What this script does
---------------------
  1. INIT        — interactive lift alignment to the Sharpie mark (same as
                   main_full_mission.py so encoder discipline is preserved)
  2. WAIT_GREEN  — waits for a green traffic light, or BTN_5 to skip
  3. BURGER_PICKUP → ... → PICK_UP_BURGER
                 — drives to the shelf row, picks each ingredient in order,
                   stacks them at the assembly slot, grips the assembled burger
  4. HOLD        — lift stays at carry height, robot stops in place.
                   This is the position it would be in just before the ramp.
                   Press BTN_5 to trigger the safe shutdown sequence.
  5. RETURN_HOME — drives the lift back to the Sharpie-mark origin and
                   disables the motor. Safe to power off.

Controls
--------
  INIT_JOG state:
    BTN_1   jog lift UP   (+LIFT_JOG_STEP ticks)
    BTN_2   jog lift DOWN (-LIFT_JOG_STEP ticks)
    BTN_10  confirm Sharpie-mark alignment → zero encoder → proceed

  WAIT_GREEN state:
    BTN_5   skip traffic-light detection (bench-test override)

  HOLD state (test complete, burger gripped):
    BTN_5   return lift to origin and exit

  Any active state (except INIT / INIT_JOG / WAIT_GREEN):
    BTN_2   emergency stop → lift returns to origin → halt

HOW TO RUN
----------
  cp test_burger_pickup.py main.py
  ros2 run robot robot

PRE-RUN CHECKLIST
-----------------
  1. Run lift_calibration.py first — paste the resulting tick values below.
  2. Align the robot at the start position (before the traffic light).
  3. Confirm INGREDIENT_SLOTS distances match the physical shelf.
  4. Confirm DIST_TO_INGREDIENT_AREA matches the distance from start to shelf.
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LEDMode,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    Limit,
    Motor,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    ServoChannel,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot


# ===========================================================================
# LIFT MOTOR — paste values from lift_calibration.py
# ===========================================================================

LIFT_MOTOR         = Motor.DC_M3
LIFT_CARRY_TICKS   = 11500   # TODO: paste from lift_calibration.py
LIFT_PICKUP_TICKS  = 7500    # TODO: paste from lift_calibration.py
LIFT_DROPOFF_TICKS = 7500    # TODO: paste from lift_calibration.py
LIFT_DOWN_TICKS    = 0      # always 0 — the Sharpie-mark origin
LIFT_MAX_VEL       = 800    # ticks/s
LIFT_TOLERANCE     = 10     # ticks
LIFT_JOG_STEP      = 50     # ticks per BTN press in INIT_JOG


# ===========================================================================
# CLAW SERVO
# ===========================================================================

CLAW_SERVO      = ServoChannel.CH_1
CLAW_OPEN_DEG   = 30.0    # TODO: calibrate
CLAW_CLOSE_DEG  = 80.0    # TODO: calibrate


# ===========================================================================
# ULTRASONIC SENSOR ON CLAW
# ===========================================================================

CLAW_ULTRASONIC_LIM = Limit.LIM_2
CLAW_GRAB_CONFIRMED = True   # polarity: True = object detected


# ===========================================================================
# DRIVE BASE
# ===========================================================================

DRIVE_VELOCITY     = 100.0   # mm/s — general cruise
APPROACH_VELOCITY  = 60.0    # mm/s — shelf approach creep
TURN_TOLERANCE_DEG = 3.0     # degrees
POS_TOLERANCE_MM   = 20.0    # mm


# ===========================================================================
# BURGER PICKUP PARAMETERS — match these to main_full_mission.py
# ===========================================================================

# Distance along Row 1 from the start position to the ingredient shelf row.
DIST_TO_INGREDIENT_AREA = 610.0    # TODO: measure (mm)

# How far the robot creeps toward the shelf after turning 90° to face it.
APPROACH_SHELF_DIST = 200.0        # TODO: measure (mm)

# Ingredient slot positions — distance from the shelf-row entry point to each
# slot, measured forward along the row (not toward the shelf).
# 0.0 = the slot the robot faces when it first stops at the shelf row.
INGREDIENT_SLOTS = {
    "bun_bottom": 0.0,    # TODO: measure (mm)
    "meat":       150.0,  # TODO: measure (mm)
    "bun_top":    300.0,  # TODO: measure (mm)
}

# Order in which to pick ingredients.  First item becomes the stack base.
INGREDIENT_ORDER = ["bun_bottom", "meat", "bun_top"]

# Slot used as the stacking / assembly zone.
ASSEMBLY_SLOT = "bun_bottom"

# Maximum pick attempts before the FSM gives up and moves on.
MAX_PICK_ATTEMPTS = 3

# E-stop immune states (BTN_2 is jog-down during INIT_JOG)
_ESTOP_IMMUNE = frozenset({"INIT", "INIT_JOG", "WAIT_GREEN"})


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
    """Slow blue blink = test done, holding position, waiting for BTN_5."""
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


# ===========================================================================
# STARTUP
# ===========================================================================

def start_robot(robot: Robot) -> None:
    state = robot.get_state()
    if state in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


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
    robot.enable_vision()   # for traffic-light detection


def reset_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed; continuing")
        robot.wait_for_pose_update(timeout=0.5)


# ===========================================================================
# LIFT HELPERS
# ===========================================================================

def get_lift_ticks(robot: Robot) -> int:
    """Current encoder position in positive ticks (0 = Sharpie-mark origin)."""
    dc = robot.get_dc_state()
    if dc is None:
        return 0
    return int(dc.motors[LIFT_MOTOR - 1].position)


def _lift_move(robot: Robot, ticks: int, label: str, timeout: float = 20.0) -> None:
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    ok = robot.set_motor_position(
        LIFT_MOTOR, ticks,
        max_vel_ticks=LIFT_MAX_VEL,
        tolerance_ticks=LIFT_TOLERANCE,
        blocking=True, timeout=timeout,
    )
    if not ok:
        print(f"[warn] lift did not reach {label} ({ticks} ticks) within {timeout:.0f}s")


def lift_to_carry(robot: Robot) -> None:
    print(f"[ARM] Lift → carry height ({LIFT_CARRY_TICKS} ticks)")
    _lift_move(robot, LIFT_CARRY_TICKS, "CARRY")


def lift_to_pickup(robot: Robot) -> None:
    print(f"[ARM] Lift → pickup height ({LIFT_PICKUP_TICKS} ticks)")
    _lift_move(robot, LIFT_PICKUP_TICKS, "PICKUP")


def lift_to_dropoff(robot: Robot) -> None:
    print(f"[ARM] Lift → drop-off height ({LIFT_DROPOFF_TICKS} ticks)")
    _lift_move(robot, LIFT_DROPOFF_TICKS, "DROPOFF")


def lift_return_to_zero(robot: Robot) -> None:
    """Return lift to Sharpie-mark origin (0 ticks) and disable motor."""
    current = get_lift_ticks(robot)
    print(f"\n[LIFT] Returning to origin — current: {current} ticks")
    if abs(current) <= LIFT_TOLERANCE:
        print("[LIFT] Already at origin.")
    else:
        travel_s = abs(current) / LIFT_MAX_VEL
        timeout  = max(15.0, travel_s * 1.5)
        robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
        ok = robot.set_motor_position(
            LIFT_MOTOR, 0,
            max_vel_ticks=LIFT_MAX_VEL,
            tolerance_ticks=LIFT_TOLERANCE,
            blocking=True, timeout=timeout,
        )
        if ok:
            print("[LIFT] Reached origin. Safe to power off.")
        else:
            print("[warn] LIFT — did not confirm origin. Align carriage to Sharpie mark manually.")
    robot.disable_motor(LIFT_MOTOR)


# ===========================================================================
# CLAW HELPERS
# ===========================================================================

def claw_open(robot: Robot) -> None:
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_OPEN_DEG)
    time.sleep(0.5)


def claw_close(robot: Robot) -> None:
    robot.set_servo(CLAW_SERVO, CLAW_CLOSE_DEG)
    time.sleep(0.5)


def claw_has_object(robot: Robot) -> bool:
    return robot.get_limit(CLAW_ULTRASONIC_LIM) == CLAW_GRAB_CONFIRMED


# ===========================================================================
# ARM SEQUENCE PRIMITIVES
# ===========================================================================

def pick_ingredient(robot: Robot) -> bool:
    """
    One pick attempt at the current robot position.
    Robot must already face the shelf and be at approach distance.
    Returns True if the ultrasonic sensor confirms a successful grab.
    """
    claw_open(robot)
    lift_to_pickup(robot)
    claw_close(robot)
    lift_to_carry(robot)

    grabbed = claw_has_object(robot)
    print("[ARM] Grab confirmed ✓" if grabbed else "[ARM] No object detected in claw")
    return grabbed


def place_ingredient(robot: Robot) -> None:
    """Release the held object at the current position."""
    lift_to_pickup(robot)
    claw_open(robot)
    time.sleep(0.3)
    lift_to_carry(robot)


# ===========================================================================
# SHELF NAVIGATION HELPERS
# ===========================================================================

def turn_to_face_shelf(robot: Robot) -> None:
    print("[NAV] Turn +90° to face shelf")
    robot.turn_by(delta_deg=90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)


def turn_away_from_shelf(robot: Robot) -> None:
    print("[NAV] Turn −90° to resume heading")
    robot.turn_by(delta_deg=-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)


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
    robot.move_forward(
        distance=-APPROACH_SHELF_DIST,
        velocity=APPROACH_VELOCITY,
        tolerance=POS_TOLERANCE_MM,
        blocking=True,
    )


def drive_to_slot(robot: Robot, from_slot: str | None, to_slot: str) -> None:
    """Drive along the row to align with to_slot, relative to from_slot."""
    from_dist = INGREDIENT_SLOTS.get(from_slot, 0.0) if from_slot else 0.0
    to_dist   = INGREDIENT_SLOTS[to_slot]
    delta     = to_dist - from_dist
    if abs(delta) < 1.0:
        return
    direction = "fwd" if delta > 0 else "bwd"
    print(f"[NAV] Drive {direction} {abs(delta):.0f} mm ({from_slot} → {to_slot})")
    robot.move_forward(
        distance=delta,
        velocity=DRIVE_VELOCITY,
        tolerance=POS_TOLERANCE_MM,
        blocking=True,
    )


# ===========================================================================
# VISION HELPER
# ===========================================================================

def detect_green_light(robot: Robot) -> bool:
    if not robot.is_vision_active(timeout_s=3.0):
        return False
    for det in robot.get_detections("traffic light"):
        if float(det.get("confidence", 0)) < 0.50:
            continue
        color = det.get("attributes", {}).get("color", {}).get("value", "")
        if color == "green":
            return True
    return False


# ===========================================================================
# RUN — PICKUP TEST FSM
# ===========================================================================

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)
    reset_pose(robot)

    state         = "INIT"
    current_slot  = None
    pick_attempts = 0

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print()
    print("=" * 56)
    print("  BURGER PICKUP TEST  —  Section 1 only")
    print("=" * 56)
    print(f"  Ingredient order : {INGREDIENT_ORDER}")
    print(f"  Assembly slot    : {ASSEMBLY_SLOT}")
    print(f"  Slot distances   : {INGREDIENT_SLOTS}")
    print(f"  Dist to shelf    : {DIST_TO_INGREDIENT_AREA:.0f} mm")
    print()
    print("  LIFT_CARRY_TICKS   =", LIFT_CARRY_TICKS)
    print("  LIFT_PICKUP_TICKS  =", LIFT_PICKUP_TICKS)
    print("  LIFT_DROPOFF_TICKS =", LIFT_DROPOFF_TICKS)
    print("=" * 56)

    while True:

        # ==================================================================
        # INIT — enable motor, open claw, show alignment instructions
        # ==================================================================
        if state == "INIT":
            robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
            claw_open(robot)
            led_idle(robot)
            print()
            print("─" * 50)
            print("  LIFT ALIGNMENT — align carriage to Sharpie mark")
            print("─" * 50)
            print(f"  BTN_1  jog UP   (+{LIFT_JOG_STEP} ticks)")
            print(f"  BTN_2  jog DOWN (-{LIFT_JOG_STEP} ticks)")
            print(f"  BTN_10 confirm alignment → zero encoder → start")
            print("─" * 50)
            state = "INIT_JOG"

        # ==================================================================
        # INIT_JOG — operator jogs lift to Sharpie mark
        # ==================================================================
        elif state == "INIT_JOG":
            current_ticks = get_lift_ticks(robot)

            if robot.was_button_pressed(Button.BTN_1):
                target = current_ticks + LIFT_JOG_STEP
                print(f"[JOG] UP  {current_ticks:+6d} → {target:+6d} ticks")
                robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                robot.set_motor_position(
                    LIFT_MOTOR, target,
                    max_vel_ticks=LIFT_MAX_VEL,
                    tolerance_ticks=LIFT_TOLERANCE,
                    blocking=True, timeout=3.0,
                )
                print(f"[JOG] Now at: {get_lift_ticks(robot)} ticks")

            elif robot.was_button_pressed(Button.BTN_2):
                target = current_ticks - LIFT_JOG_STEP
                print(f"[JOG] DOWN {current_ticks:+6d} → {target:+6d} ticks")
                robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
                robot.set_motor_position(
                    LIFT_MOTOR, target,
                    max_vel_ticks=LIFT_MAX_VEL,
                    tolerance_ticks=LIFT_TOLERANCE,
                    blocking=True, timeout=3.0,
                )
                print(f"[JOG] Now at: {get_lift_ticks(robot)} ticks")

            elif robot.was_button_pressed(Button.BTN_10):
                pre = get_lift_ticks(robot)
                print(f"[INIT] Zeroing encoder at {pre} ticks …")
                robot.reset_motor_position(LIFT_MOTOR)
                time.sleep(0.15)
                print(f"[INIT] Encoder zeroed. Reading after reset: {get_lift_ticks(robot)} ticks")
                print("[INIT] Sharpie-mark origin confirmed → WAIT_GREEN")
                led_moving(robot)
                state = "WAIT_GREEN"

        # ==================================================================
        # WAIT_GREEN — watch camera for green light before starting
        # BTN_5 skips for bench testing
        # ==================================================================
        elif state == "WAIT_GREEN":
            if detect_green_light(robot):
                print("[FSM] Green light detected → BURGER_PICKUP")
                state = "BURGER_PICKUP"
            elif robot.was_button_pressed(Button.BTN_5):
                print("[FSM] BTN_5 override — skipping traffic light → BURGER_PICKUP")
                state = "BURGER_PICKUP"

        # ==================================================================
        # BURGER_PICKUP — drive forward to the shelf row
        # ==================================================================
        elif state == "BURGER_PICKUP":
            print(f"\n[FSM] BURGER_PICKUP — driving {DIST_TO_INGREDIENT_AREA:.0f} mm to shelf row")
            robot.move_forward(
                distance=DIST_TO_INGREDIENT_AREA,
                velocity=DRIVE_VELOCITY,
                tolerance=POS_TOLERANCE_MM,
                blocking=True,
            )
            robot.stop()
            current_slot  = None
            pick_attempts = 0
            print("[FSM] BURGER_PICKUP → MOVE_TO_MEAT")
            state = "MOVE_TO_MEAT"

        # ==================================================================
        # MOVE_TO_MEAT — align with first ingredient slot
        # ==================================================================
        elif state == "MOVE_TO_MEAT":
            target = INGREDIENT_ORDER[0]
            print(f"\n[FSM] MOVE_TO_MEAT — aligning with '{target}' slot")
            drive_to_slot(robot, current_slot, target)
            current_slot  = target
            pick_attempts = 0
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            state = "PICK_UP_MEAT"

        # ==================================================================
        # PICK_UP_MEAT — pick first ingredient, retry on miss
        # ==================================================================
        elif state == "PICK_UP_MEAT":
            print(f"\n[FSM] PICK_UP_MEAT — attempt {pick_attempts + 1}/{MAX_PICK_ATTEMPTS}")
            grabbed = pick_ingredient(robot)
            pick_attempts += 1

            if grabbed or pick_attempts >= MAX_PICK_ATTEMPTS:
                if not grabbed:
                    print("[warn] PICK_UP_MEAT — max retries; continuing anyway")
                    led_error(robot)
                    time.sleep(0.5)
                    led_moving(robot)
                retreat_from_shelf(robot)
                turn_away_from_shelf(robot)
                pick_attempts = 0
                print("[FSM] PICK_UP_MEAT → MOVE_TO_BURGER_BUN1")
                state = "MOVE_TO_BURGER_BUN1"

        # ==================================================================
        # MOVE_TO_BURGER_BUN1 — carry first ingredient to assembly slot
        # ==================================================================
        elif state == "MOVE_TO_BURGER_BUN1":
            print(f"\n[FSM] MOVE_TO_BURGER_BUN1 — carrying to assembly slot '{ASSEMBLY_SLOT}'")
            drive_to_slot(robot, current_slot, ASSEMBLY_SLOT)
            current_slot = ASSEMBLY_SLOT
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            state = "PLACE_HELD_OBJECT"

        # ==================================================================
        # PLACE_HELD_OBJECT — set first ingredient down at assembly slot
        # ==================================================================
        elif state == "PLACE_HELD_OBJECT":
            print(f"\n[FSM] PLACE_HELD_OBJECT — placing '{INGREDIENT_ORDER[0]}'")
            place_ingredient(robot)
            retreat_from_shelf(robot)
            turn_away_from_shelf(robot)
            pick_attempts = 0
            print("[FSM] PLACE_HELD_OBJECT → MOVE_TO_BURGER_BUN2")
            state = "MOVE_TO_BURGER_BUN2"

        # ==================================================================
        # MOVE_TO_BURGER_BUN2 — align with second ingredient slot
        # ==================================================================
        elif state == "MOVE_TO_BURGER_BUN2":
            target = INGREDIENT_ORDER[1]
            print(f"\n[FSM] MOVE_TO_BURGER_BUN2 — aligning with '{target}' slot")
            drive_to_slot(robot, current_slot, target)
            current_slot  = target
            pick_attempts = 0
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            state = "PICK_UP_BUN2"

        # ==================================================================
        # PICK_UP_BUN2 — pick second ingredient, retry on miss
        # ==================================================================
        elif state == "PICK_UP_BUN2":
            print(f"\n[FSM] PICK_UP_BUN2 — attempt {pick_attempts + 1}/{MAX_PICK_ATTEMPTS}")
            grabbed = pick_ingredient(robot)
            pick_attempts += 1

            if grabbed or pick_attempts >= MAX_PICK_ATTEMPTS:
                if not grabbed:
                    print("[warn] PICK_UP_BUN2 — max retries; continuing anyway")
                    led_error(robot)
                    time.sleep(0.5)
                    led_moving(robot)
                retreat_from_shelf(robot)
                turn_away_from_shelf(robot)
                pick_attempts = 0
                print("[FSM] PICK_UP_BUN2 → MOVE_TO_BURGER_BUN3")
                state = "MOVE_TO_BURGER_BUN3"

        # ==================================================================
        # MOVE_TO_BURGER_BUN3 — carry second ingredient back to assembly slot
        # ==================================================================
        elif state == "MOVE_TO_BURGER_BUN3":
            print(f"\n[FSM] MOVE_TO_BURGER_BUN3 — returning to assembly slot '{ASSEMBLY_SLOT}'")
            drive_to_slot(robot, current_slot, ASSEMBLY_SLOT)
            current_slot = ASSEMBLY_SLOT
            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()
            state = "PLACE_HELD_OBJECT1"

        # ==================================================================
        # PLACE_HELD_OBJECT1 — stack second ingredient onto the first
        # ==================================================================
        elif state == "PLACE_HELD_OBJECT1":
            print(f"\n[FSM] PLACE_HELD_OBJECT1 — stacking '{INGREDIENT_ORDER[1]}'")
            place_ingredient(robot)
            retreat_from_shelf(robot)
            turn_away_from_shelf(robot)
            pick_attempts = 0
            print("[FSM] PLACE_HELD_OBJECT1 → PICK_UP_BURGER")
            state = "PICK_UP_BURGER"

        # ==================================================================
        # PICK_UP_BURGER — grip the fully assembled burger stack
        # ==================================================================
        elif state == "PICK_UP_BURGER":
            print(f"\n[FSM] PICK_UP_BURGER — attempt {pick_attempts + 1}/{MAX_PICK_ATTEMPTS}")
            # The assembly slot is current_slot; approach it to grip the stack.
            turn_to_face_shelf(robot)
            approach_shelf(robot)

            grabbed = pick_ingredient(robot)
            pick_attempts += 1

            retreat_from_shelf(robot)
            turn_away_from_shelf(robot)

            if grabbed or pick_attempts >= MAX_PICK_ATTEMPTS:
                if not grabbed:
                    print("[warn] PICK_UP_BURGER — max retries; burger may not be gripped")
                    led_error(robot)
                    time.sleep(0.5)
                    led_moving(robot)
                pick_attempts = 0
                print()
                print("=" * 56)
                print("  ✓  PICKUP SEQUENCE COMPLETE")
                print("  Burger is gripped at carry height.")
                print("  Robot stopped — would proceed to ramp here.")
                print()
                print("  BTN_5  → return lift to origin and exit")
                print("  BTN_2  → emergency stop (also returns to origin)")
                print("=" * 56)
                led_hold(robot)
                state = "HOLD"

        # ==================================================================
        # HOLD — test complete; robot is stationary, burger held at carry height
        # Operator inspects the result, then presses BTN_5 to finish cleanly.
        # ==================================================================
        elif state == "HOLD":
            if robot.was_button_pressed(Button.BTN_5):
                print("\n[FSM] HOLD — BTN_5 pressed → RETURN_HOME")
                state = "RETURN_HOME"

        # ==================================================================
        # RETURN_HOME — lower lift to Sharpie-mark origin, disable motor, done
        # ==================================================================
        elif state == "RETURN_HOME":
            robot.stop()
            lift_return_to_zero(robot)
            led_done(robot)
            print()
            print("=" * 56)
            print("  Lift at origin. Robot safe to power off.")
            print("=" * 56)
            break   # exit the FSM loop — node stays running but run() returns

        # ==================================================================
        # GLOBAL EMERGENCY STOP — BTN_2 from any non-immune state
        # ==================================================================
        if state not in _ESTOP_IMMUNE:
            if robot.was_button_pressed(Button.BTN_2):
                print("\n[FSM] !! BTN_2 — EMERGENCY STOP !!")
                robot.stop()
                led_error(robot)
                print("[ESTOP] Returning lift to origin …")
                try:
                    lift_return_to_zero(robot)
                    print("[ESTOP] Lift at origin. Safe to power off.")
                except Exception as exc:
                    print(f"[ESTOP] lift_return_to_zero raised: {exc}")
                    print("[ESTOP] Manually align carriage to Sharpie mark before powering off.")
                robot.estop()
                break

        # ── tick-rate control ──────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
