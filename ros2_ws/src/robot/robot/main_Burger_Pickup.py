"""
main.py — Full Mission FSM (Burger Pickup — Left Side of State Diagram)
=======================================================================
State machine for the MAE 162D/E capstone rover.

This file covers the LEFT side of the FSM diagram:
  Idle → Traffic_Light_Detection → Burger_Pickup → Scan_Object_Order
    → [Meat branch]  Move_To_Meat → Pick_Up_Meat → Move_To_Burger_Bun1
                     → Place_Held_Object → Idle5
    → [Bun branch]   Move_To_Burger_Bun2 → Pick_Up_Bun2 → Move_To_Burger_Bun3
                     → Place_Held_Object1 → Idle6 → Pick_Up_Burger → [done]

HARDWARE LAYOUT:
  - Claw/lift arm  : FRONT of the robot
  - Drive wheels   : BACK of the robot
  - Ingredient shelf: LEFT side of the robot's travel path
  - To face shelf  : turn_by(+90 deg)  [CCW, robot's left = shelf]
  - To resume path : turn_by(-90 deg)  [CW, back to original heading]

MOTOR / SERVO ASSIGNMENT:
  - DC_M1             : left drive wheel
  - DC_M2             : right drive wheel (direction inverted)
  - DC_M3             : vertical lift (POSITION mode)
  - ServoChannel.CH_1 : claw jaw servo

PRE-RUN CONFIGURATION — fill in before each run:
  Set the ingredient slot distances in the "Ingredient shelf layout" section
  below. Measure each slot's distance along the shelf row from the robot's
  starting position at the shelf. The order list tells the FSM which slot
  to visit first, second, and third.

CAMERA:
  camera_detect_green() — used only for the traffic light. Replace the body
  with real OpenCV code when ready. No camera is used for ingredient picking.
"""

from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    LED,
    Motor,
    ServoChannel,
)
from robot.util import TaskHandle, run_task

# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

POSITION_UNIT            = Unit.MM
WHEEL_DIAMETER           = 74.0    # mm
WHEEL_BASE               = 333.0   # mm
INITIAL_THETA_DEG        = 90.0    # degrees — heading at odometry reset

LEFT_WHEEL_MOTOR         = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED  = False
RIGHT_WHEEL_MOTOR        = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# ---------------------------------------------------------------------------
# Lift motor (DC_M3, position control)
# ---------------------------------------------------------------------------

LIFT_MOTOR        = Motor.DC_M3
LIFT_UP_TICKS     = 3000    # TODO: calibrate — encoder ticks for raised position
LIFT_DOWN_TICKS   = 0       # TODO: calibrate — encoder ticks for lowered position
LIFT_MAX_VEL      = 250     # ticks/s
LIFT_TOLERANCE    = 30      # ticks — position dead-band

# ---------------------------------------------------------------------------
# Claw servo (CH_1)
# ---------------------------------------------------------------------------

CLAW_SERVO        = ServoChannel.CH_1
CLAW_OPEN_DEG     = 20.0    # TODO: calibrate — jaw fully open
CLAW_CLOSE_DEG    = 90.0    # TODO: calibrate — jaw gripping

# ---------------------------------------------------------------------------
# Navigation parameters
# ---------------------------------------------------------------------------

DRIVE_VELOCITY        = 100.0   # mm/s — general cruise speed
APPROACH_VELOCITY     = 60.0    # mm/s — slower speed when closing on shelf
TURN_TOLERANCE_DEG    = 3.0     # degrees — turn completion threshold
POSITION_TOLERANCE    = 20.0    # mm — arrival tolerance for move_forward

# Distance the robot drives forward from the start position (after the
# traffic light) to reach the shelf row.
DIST_TO_INGREDIENT_AREA = 610.0   # TODO: measure on course (mm)

# How far the robot creeps toward the shelf after turning 90 degrees to face it.
APPROACH_SHELF_DIST = 200.0       # TODO: measure on course (mm)

# ---------------------------------------------------------------------------
# Ingredient shelf layout  ← SET THESE BEFORE EACH RUN
# ---------------------------------------------------------------------------
# Each value is the forward distance (mm) the robot must drive along the shelf
# row — starting from where it first stops at the row — to align with that
# ingredient slot.  0.0 means the robot is already aligned when it arrives.
#
# INGREDIENT_ORDER lists the three pick-up positions in the sequence the robot
# will visit them.  Each entry is a key into INGREDIENT_SLOTS.
# Adjust the order and distances to match the actual shelf layout on competition
# day (the venue spec says positions are fixed 5 weeks before the event).

INGREDIENT_SLOTS = {
    "meat":       0.0,    # TODO: measure — distance along row to meat slot (mm)
    "bun_bottom": 150.0,  # TODO: measure — distance along row to bottom bun (mm)
    "bun_top":    300.0,  # TODO: measure — distance along row to top bun (mm)
}

# Visit order: first item is picked and placed as the base; the assembled
# burger is grabbed at the end.  Change to match the actual shelf arrangement.
INGREDIENT_ORDER = ["bun_bottom", "meat", "bun_top"]

# Assembly zone: where the robot places each ingredient to stack the burger.
# This is typically one of the shelf slots reused as a staging area.
ASSEMBLY_SLOT = "bun_bottom"   # TODO: set to whichever slot is the assembly point

# ---------------------------------------------------------------------------
# Retry policy  (state diagram: "Failure_Repeat_3_times")
# ---------------------------------------------------------------------------

MAX_PICK_ATTEMPTS = 3

# ---------------------------------------------------------------------------
# Camera — traffic light only
# ---------------------------------------------------------------------------

def camera_detect_green() -> bool:
    """
    Return True when the traffic light is GREEN.
    Replace with real OpenCV code using the Pi camera.

    Example skeleton:
        frame = camera.capture()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, GREEN_LOW, GREEN_HIGH)
        return green_mask.sum() > THRESHOLD
    """
    # TODO: implement real traffic-light detection
    return False   # set to True to skip light detection during bench tests


# ---------------------------------------------------------------------------
# LED helpers
# ---------------------------------------------------------------------------

def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 200)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def show_error_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.RED, 200)


# ---------------------------------------------------------------------------
# Robot startup
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    """Apply unit, wheel geometry, and motor mapping. Called once at startup."""
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


def start_robot(robot: Robot) -> None:
    """Transition firmware to RUNNING and zero the odometry pose."""
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)


# ---------------------------------------------------------------------------
# Low-level arm helpers  (all blocking)
# ---------------------------------------------------------------------------

def lift_raise(robot: Robot) -> None:
    """Move the lift to the raised (travel) position."""
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    robot.set_motor_position(
        LIFT_MOTOR, LIFT_UP_TICKS,
        max_vel_ticks=LIFT_MAX_VEL,
        tolerance_ticks=LIFT_TOLERANCE,
        blocking=True, timeout=6.0,
    )


def lift_lower(robot: Robot) -> None:
    """Move the lift to the lowered (pick/place) position."""
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    robot.set_motor_position(
        LIFT_MOTOR, LIFT_DOWN_TICKS,
        max_vel_ticks=LIFT_MAX_VEL,
        tolerance_ticks=LIFT_TOLERANCE,
        blocking=True, timeout=6.0,
    )


def claw_open(robot: Robot) -> None:
    """Open the claw and wait for the servo to reach position."""
    robot.enable_servo(CLAW_SERVO)
    robot.set_servo(CLAW_SERVO, CLAW_OPEN_DEG)
    time.sleep(0.4)


def claw_close(robot: Robot) -> None:
    """Close the claw and wait for it to settle onto the object."""
    robot.set_servo(CLAW_SERVO, CLAW_CLOSE_DEG)
    time.sleep(0.5)


# ---------------------------------------------------------------------------
# pick_ingredient — one attempt to grab whatever is in front of the claw
# ---------------------------------------------------------------------------
# Sequence (robot already faces shelf, approach already done):
#   1. Open claw   — safety, avoids dragging on shelf while lowering
#   2. Lower lift  — bring claw down to ingredient height
#   3. Close claw  — grip
#   4. Raise lift  — lift ingredient clear of the shelf
#
# Returns True always (no camera verification). The retry loop in the FSM
# handles the case where a pick fails mechanically — just re-attempt.

def pick_ingredient(robot: Robot) -> bool:
    """Execute one pick attempt at the current arm position. Returns True."""
    print("[ARM] Opening claw")
    claw_open(robot)

    print("[ARM] Lowering lift to pick height")
    lift_lower(robot)

    print("[ARM] Closing claw to grip ingredient")
    claw_close(robot)

    print("[ARM] Raising lift with ingredient")
    lift_raise(robot)

    return True   # assume success; operator retries manually if needed


# ---------------------------------------------------------------------------
# place_ingredient — release the held ingredient at the current position
# ---------------------------------------------------------------------------
# Sequence:
#   1. Lower lift  — descend to shelf/assembly height
#   2. Open claw   — release
#   3. Brief pause — ingredient settles
#   4. Raise lift  — clear the placed item

def place_ingredient(robot: Robot) -> None:
    """Release the currently held ingredient at the current arm position."""
    print("[ARM] Lowering lift to place height")
    lift_lower(robot)

    print("[ARM] Opening claw to release ingredient")
    claw_open(robot)
    time.sleep(0.3)   # let ingredient settle

    print("[ARM] Raising lift clear")
    lift_raise(robot)


# ---------------------------------------------------------------------------
# Navigation helpers for shelf approach / retreat
# ---------------------------------------------------------------------------

def turn_to_face_shelf(robot: Robot) -> None:
    """Turn 90 deg CCW (left) so the front claw faces the ingredient shelf."""
    print("[NAV] Turning 90 deg left to face shelf")
    robot.turn_by(delta_deg=90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)


def turn_away_from_shelf(robot: Robot) -> None:
    """Turn 90 deg CW (right) to resume the original forward heading."""
    print("[NAV] Turning 90 deg right to resume course heading")
    robot.turn_by(delta_deg=-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)


def approach_shelf(robot: Robot) -> None:
    """Creep toward the shelf after turning to face it."""
    print(f"[NAV] Approaching shelf {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_forward(
        distance=APPROACH_SHELF_DIST,
        velocity=APPROACH_VELOCITY,
        tolerance=POSITION_TOLERANCE,
        blocking=True,
    )


def retreat_from_shelf(robot: Robot) -> None:
    """Back straight away from the shelf the same distance we approached."""
    print(f"[NAV] Retreating from shelf {APPROACH_SHELF_DIST:.0f} mm")
    robot.move_forward(
        distance=-APPROACH_SHELF_DIST,
        velocity=APPROACH_VELOCITY,
        tolerance=POSITION_TOLERANCE,
        blocking=True,
    )


def drive_to_slot(robot: Robot, from_slot: str, to_slot: str) -> None:
    """
    Drive forward or backward along the shelf row to move from one ingredient
    slot to another.  Uses the pre-configured distances in INGREDIENT_SLOTS.
    Pass ASSEMBLY_SLOT as either argument when going to/from the assembly zone.
    """
    from_dist = INGREDIENT_SLOTS.get(from_slot, INGREDIENT_SLOTS[ASSEMBLY_SLOT])
    to_dist   = INGREDIENT_SLOTS.get(to_slot,   INGREDIENT_SLOTS[ASSEMBLY_SLOT])
    delta = to_dist - from_dist
    if abs(delta) < 1.0:
        return   # already aligned, nothing to do
    direction = "forward" if delta > 0 else "backward"
    print(f"[NAV] Moving {direction} {abs(delta):.0f} mm ({from_slot} → {to_slot})")
    robot.move_forward(
        distance=delta,
        velocity=DRIVE_VELOCITY,
        tolerance=POSITION_TOLERANCE,
        blocking=True,
    )


# ---------------------------------------------------------------------------
# run() — entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    # ── FSM state variable ─────────────────────────────────────────────────
    state = "INIT"

    # Tracks which ingredient the FSM is currently handling and
    # which slot the robot is positioned at (for drive_to_slot math).
    current_ingredient_index = 0   # index into INGREDIENT_ORDER
    current_slot = None            # name of the slot the robot is at (or None)

    # Per-pick attempt counter; reset each time we enter a new pick state
    pick_attempts = 0

    # FSM tick-rate control
    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    # ── Main FSM loop ──────────────────────────────────────────────────────
    while True:

        # ==================================================================
        # INIT — firmware startup, odometry zero, arm home
        # ==================================================================
        if state == "INIT":
            print("[FSM] INIT — starting robot firmware")
            start_robot(robot)

            print("[FSM] INIT — homing arm: raise lift, open claw")
            lift_raise(robot)
            claw_open(robot)

            show_idle_leds(robot)
            print("[FSM] INIT complete → IDLE")
            print("      Verify INGREDIENT_SLOTS and INGREDIENT_ORDER are set.")
            print("      Press BTN_1 to begin mission.")
            state = "IDLE"

        # ==================================================================
        # IDLE — wait for operator start  (state diagram: "Idle")
        # ==================================================================
        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                print("[FSM] IDLE → TRAFFIC_LIGHT_DETECTION")
                # Print pre-run config so operator can confirm before moving
                print(f"  Ingredient order : {INGREDIENT_ORDER}")
                print(f"  Slot distances   : {INGREDIENT_SLOTS}")
                print(f"  Assembly slot    : {ASSEMBLY_SLOT}")
                show_moving_leds(robot)
                state = "TRAFFIC_LIGHT_DETECTION"

        # ==================================================================
        # TRAFFIC_LIGHT_DETECTION — wait until camera sees green
        # State diagram edge: [camera.detectGreen == true]
        # ==================================================================
        elif state == "TRAFFIC_LIGHT_DETECTION":
            if camera_detect_green():
                print("[FSM] Green light detected → BURGER_PICKUP")
                state = "BURGER_PICKUP"
            # BTN_2 provides a manual override during testing so you can
            # skip the traffic light without real camera code yet.
            elif robot.was_button_pressed(Button.BTN_2):
                print("[FSM] (Test override) BTN_2 — skipping traffic light")
                state = "BURGER_PICKUP"

        # ==================================================================
        # BURGER_PICKUP — drive forward to the ingredient shelf row
        # State diagram: "Burger_Pickup"
        # ==================================================================
        elif state == "BURGER_PICKUP":
            print(f"[FSM] BURGER_PICKUP — driving {DIST_TO_INGREDIENT_AREA:.0f} mm to shelf row")
            show_moving_leds(robot)
            robot.move_forward(
                distance=DIST_TO_INGREDIENT_AREA,
                velocity=DRIVE_VELOCITY,
                tolerance=POSITION_TOLERANCE,
                blocking=True,
            )
            robot.stop()
            print("[FSM] Reached ingredient area → SCAN_OBJECT_ORDER")
            state = "SCAN_OBJECT_ORDER"

        # ==================================================================
        # SCAN_OBJECT_ORDER — no camera scan needed; order is pre-configured
        # State diagram: "Scan_Object_Order"
        # We still keep this state so the FSM structure matches the diagram.
        # ==================================================================
        elif state == "SCAN_OBJECT_ORDER":
            print("[FSM] SCAN_OBJECT_ORDER — using pre-configured ingredient order")
            print(f"  Order: {INGREDIENT_ORDER}")
            current_ingredient_index = 0
            pick_attempts = 0
            current_slot = None   # robot is at the row start, not yet at any slot
            print("[FSM] SCAN_OBJECT_ORDER → MOVE_TO_MEAT")
            state = "MOVE_TO_MEAT"

        # ==================================================================
        # MOVE_TO_MEAT — drive to the first ingredient slot (typically meat)
        # State diagram: "Move_To_Meat"
        # ==================================================================
        elif state == "MOVE_TO_MEAT":
            target_slot = INGREDIENT_ORDER[0]   # first ingredient in pre-set order
            print(f"[FSM] MOVE_TO_MEAT — moving to '{target_slot}' slot")
            show_moving_leds(robot)

            # Drive along the row from the current position to the target slot.
            # current_slot is None on the first move, meaning we start from 0.0.
            from_offset = INGREDIENT_SLOTS.get(current_slot, 0.0) if current_slot else 0.0
            to_offset   = INGREDIENT_SLOTS[target_slot]
            delta = to_offset - from_offset
            if abs(delta) > 1.0:
                robot.move_forward(
                    distance=delta,
                    velocity=DRIVE_VELOCITY,
                    tolerance=POSITION_TOLERANCE,
                    blocking=True,
                )
            current_slot = target_slot

            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()

            current_ingredient_index = 0
            pick_attempts = 0
            print(f"[FSM] MOVE_TO_MEAT → PICK_UP_MEAT")
            state = "PICK_UP_MEAT"

        # ==================================================================
        # PICK_UP_MEAT — pick the first ingredient (up to MAX_PICK_ATTEMPTS)
        # State diagram: "Pick_Up_Meat" + "Failure_Repeat_3_times" loop
        # ==================================================================
        elif state == "PICK_UP_MEAT":
            print(f"[FSM] PICK_UP_MEAT attempt {pick_attempts + 1}/{MAX_PICK_ATTEMPTS} "
                  f"(ingredient: {INGREDIENT_ORDER[0]})")

            pick_ingredient(robot)
            pick_attempts += 1

            # After picking, withdraw and reorient to travel forward
            retreat_from_shelf(robot)
            turn_away_from_shelf(robot)

            if pick_attempts < MAX_PICK_ATTEMPTS:
                # Assume success on first attempt; retries are manual re-runs.
                # If you add a sensor later, check it here before proceeding.
                pass

            print("[FSM] PICK_UP_MEAT → MOVE_TO_BURGER_BUN1")
            pick_attempts = 0
            state = "MOVE_TO_BURGER_BUN1"

        # ==================================================================
        # MOVE_TO_BURGER_BUN1 — carry held ingredient to the assembly slot
        # State diagram: "Move_To_Burger_Bun1"
        # ==================================================================
        elif state == "MOVE_TO_BURGER_BUN1":
            print(f"[FSM] MOVE_TO_BURGER_BUN1 — moving to assembly slot '{ASSEMBLY_SLOT}'")
            show_moving_leds(robot)

            drive_to_slot(robot, from_slot=current_slot, to_slot=ASSEMBLY_SLOT)
            current_slot = ASSEMBLY_SLOT

            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()

            print("[FSM] MOVE_TO_BURGER_BUN1 → PLACE_HELD_OBJECT")
            state = "PLACE_HELD_OBJECT"

        # ==================================================================
        # PLACE_HELD_OBJECT — release the first ingredient at the assembly slot
        # State diagram: "Place_Held_Object" → "Idle5"
        # ==================================================================
        elif state == "PLACE_HELD_OBJECT":
            print(f"[FSM] PLACE_HELD_OBJECT — placing '{INGREDIENT_ORDER[0]}' at assembly slot")
            place_ingredient(robot)

            retreat_from_shelf(robot)
            turn_away_from_shelf(robot)
            robot.stop()

            # "Idle5" in the diagram is a sync point; flow straight into bun-2 branch
            print("[FSM] PLACE_HELD_OBJECT (Idle5) → MOVE_TO_BURGER_BUN2")
            current_ingredient_index = 1
            pick_attempts = 0
            state = "MOVE_TO_BURGER_BUN2"

        # ==================================================================
        # MOVE_TO_BURGER_BUN2 — align with the second ingredient slot
        # State diagram: "Move_To_Burger_Bun2"
        # ==================================================================
        elif state == "MOVE_TO_BURGER_BUN2":
            target_slot = INGREDIENT_ORDER[1]
            print(f"[FSM] MOVE_TO_BURGER_BUN2 — moving to '{target_slot}' slot")
            show_moving_leds(robot)

            drive_to_slot(robot, from_slot=current_slot, to_slot=target_slot)
            current_slot = target_slot

            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()

            pick_attempts = 0
            print(f"[FSM] MOVE_TO_BURGER_BUN2 → PICK_UP_BUN2")
            state = "PICK_UP_BUN2"

        # ==================================================================
        # PICK_UP_BUN2 — pick the second ingredient (up to MAX_PICK_ATTEMPTS)
        # State diagram: "Pick_Up_Bun2" + "Failure_repeat_3_times" loop
        # ==================================================================
        elif state == "PICK_UP_BUN2":
            print(f"[FSM] PICK_UP_BUN2 attempt {pick_attempts + 1}/{MAX_PICK_ATTEMPTS} "
                  f"(ingredient: {INGREDIENT_ORDER[1]})")

            pick_ingredient(robot)
            pick_attempts += 1

            retreat_from_shelf(robot)
            turn_away_from_shelf(robot)

            print("[FSM] PICK_UP_BUN2 → MOVE_TO_BURGER_BUN3")
            pick_attempts = 0
            state = "MOVE_TO_BURGER_BUN3"

        # ==================================================================
        # MOVE_TO_BURGER_BUN3 — carry second ingredient back to assembly slot
        # State diagram: "Move_To_Burger_Bun3"  (second visit to assembly zone)
        # ==================================================================
        elif state == "MOVE_TO_BURGER_BUN3":
            print(f"[FSM] MOVE_TO_BURGER_BUN3 — returning to assembly slot '{ASSEMBLY_SLOT}'")
            show_moving_leds(robot)

            drive_to_slot(robot, from_slot=current_slot, to_slot=ASSEMBLY_SLOT)
            current_slot = ASSEMBLY_SLOT

            turn_to_face_shelf(robot)
            approach_shelf(robot)
            robot.stop()

            print("[FSM] MOVE_TO_BURGER_BUN3 → PLACE_HELD_OBJECT1")
            state = "PLACE_HELD_OBJECT1"

        # ==================================================================
        # PLACE_HELD_OBJECT1 — release second ingredient onto the assembly stack
        # State diagram: "Place_Held_Object1" → "Idle6"
        # ==================================================================
        elif state == "PLACE_HELD_OBJECT1":
            print(f"[FSM] PLACE_HELD_OBJECT1 — placing '{INGREDIENT_ORDER[1]}' at assembly slot")
            place_ingredient(robot)

            retreat_from_shelf(robot)
            turn_away_from_shelf(robot)
            robot.stop()

            # "Idle6" → "Pick_Up_Burger" in the state diagram
            print("[FSM] PLACE_HELD_OBJECT1 (Idle6) → PICK_UP_BURGER")
            pick_attempts = 0
            state = "PICK_UP_BURGER"

        # ==================================================================
        # PICK_UP_BURGER — grip the fully assembled burger (up to 3 retries)
        # State diagram: "Pick_Up_Burger" + "Failure_repeat_3_times" loop
        # ==================================================================
        elif state == "PICK_UP_BURGER":
            print(f"[FSM] PICK_UP_BURGER attempt {pick_attempts + 1}/{MAX_PICK_ATTEMPTS}")
            show_moving_leds(robot)

            # The assembled burger is at the assembly slot; approach it
            turn_to_face_shelf(robot)
            approach_shelf(robot)

            pick_ingredient(robot)
            pick_attempts += 1

            retreat_from_shelf(robot)
            turn_away_from_shelf(robot)

            if pick_attempts < MAX_PICK_ATTEMPTS:
                print("[FSM] PICK_UP_BURGER → BURGER_PICKUP_COMPLETE")
                state = "BURGER_PICKUP_COMPLETE"
            else:
                # All retries exhausted — continue the mission regardless
                print("[FSM] PICK_UP_BURGER — attempts exhausted, continuing")
                show_error_leds(robot)
                state = "BURGER_PICKUP_COMPLETE"

        # ==================================================================
        # BURGER_PICKUP_COMPLETE — burger section done; robot holds burger
        # Transition to the next FSM section (ramp / Move_Course) when ready
        # ==================================================================
        elif state == "BURGER_PICKUP_COMPLETE":
            print("[FSM] ═══════════════════════════════════════════════")
            print("[FSM]  Burger pickup sequence COMPLETE")
            print("[FSM]  Robot is holding the assembled burger.")
            print("[FSM]  Ready to transition to: Move_Course / Ramp")
            print("[FSM] ═══════════════════════════════════════════════")
            show_moving_leds(robot)

            # TODO: wire in the next FSM section here, e.g.:
            #   state = "MOVE_COURSE"
            state = "HOLDING"

        # ==================================================================
        # HOLDING — temporary terminal state until next section is added
        # ==================================================================
        elif state == "HOLDING":
            show_idle_leds(robot)
            time.sleep(0.5)

        # ── Global emergency stop — BTN_2 halts from any active state ──────
        if state not in ("INIT", "IDLE", "TRAFFIC_LIGHT_DETECTION", "HOLDING"):
            if robot.was_button_pressed(Button.BTN_2):
                print("[FSM] !! BTN_2 — EMERGENCY STOP !!")
                robot.stop()
                show_error_leds(robot)
                robot.estop()
                state = "IDLE"

        # ── FSM tick-rate control ──────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
