"""
lift_calibration.py — Interactive Lift Motor Calibration
=========================================================
Use this tool to find and verify all encoder tick values for the lift motor
before running the full mission.

TICK VALUES THIS TOOL MEASURES
-------------------------------
  LIFT_CARRY_TICKS   — height the lift holds while the robot is driving
                       (burger gripped, arm clear of the floor)
  LIFT_PICKUP_TICKS  — height the claw lowers to in order to grab an
                       ingredient off the shelf
  LIFT_DROPOFF_TICKS — height the claw lowers to for the final drop-off
                       placement

  LIFT_DOWN_TICKS    — always 0 (the Sharpie-mark origin); no save button
                       needed, this value never changes.

ENCODER / ORIGIN RULE
---------------------
The Arduino does NOT persist encoder positions across power cycles.
Every power-on resets the encoder to 0 at whatever physical position the
lift happens to be in.

To keep the saved tick values valid across runs, the lift carriage must be
at the Sharpie-mark origin before every power-off.  This tool enforces that:

  • At startup it jogs the lift interactively to the Sharpie mark and zeros
    the encoder there (BTN_1 → HOMING state does this automatically with the
    limit switch, or you confirm manually when USE_LIMIT_SWITCH = False).
  • All tick values are measured relative to that zeroed origin.
  • Before exit it always returns the lift to the origin so the next
    power-on starts from the same physical height.

HOW TO RUN
----------
  cp lift_calibration.py main.py
  ros2 run robot robot

CONTROLS  (active in JOG mode)
--------
  BTN_1  — start calibration / retry homing
  BTN_2  — jog lift UP   (+JOG_STEP_TICKS per press)
  BTN_3  — jog lift DOWN (-JOG_STEP_TICKS per press)
  BTN_4  — save current position as LIFT_CARRY_TICKS
  BTN_6  — save current position as LIFT_PICKUP_TICKS
  BTN_7  — save current position as LIFT_DROPOFF_TICKS
  BTN_5  — return to Sharpie-mark origin and print results (also on exit)

READING YOUR RESULTS
--------------------
At the end of the session the tool prints a config block to copy directly
into main_full_mission.py:

    LIFT_CARRY_TICKS   = <value>
    LIFT_PICKUP_TICKS  = <value>
    LIFT_DROPOFF_TICKS = <value>
    LIFT_DOWN_TICKS    = 0        # always 0 — the Sharpie-mark origin
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    LED,
    LEDMode,
    Limit,
    Motor,
    POSITION_UNIT,
)
from robot.robot import FirmwareState, Robot


# ===========================================================================
# CALIBRATION CONFIGURATION — edit these to match your build
# ===========================================================================

# Which DC motor drives the lift.
LIFT_MOTOR = Motor.DC_M3

# Set True if a limit switch is wired at the BOTTOM of the lift travel.
# Set False if you have no limit switch and will manually lower the lift first.
USE_LIMIT_SWITCH = False   # set True only if you have a physical limit switch

# Which limit-switch input the bottom-of-travel switch is wired to.
LIFT_HOME_LIMIT = Limit.LIM_1   # change to match your wiring

# Homing direction: -1 = motor runs in the direction that lowers the lift.
# If homing moves the lift the wrong way, flip the sign.
HOME_DIRECTION = -1

# Slow velocity used only for homing (ticks/s).  Keep it gentle.
HOME_VELOCITY_TICKS = 80

# Velocity used for jog moves and return-to-origin (ticks/s).
JOG_VELOCITY_TICKS = 200

# How many ticks each BTN_2 / BTN_3 press moves the lift.
# Smaller = more precise alignment; increase for faster coarse moves.
JOG_STEP_TICKS = 50

# Position tolerance accepted as "at target" (ticks).
POSITION_TOLERANCE = 30

# Safety: how many ticks above origin the lift is allowed to travel.
# Acts as a soft upper limit during calibration to protect the mechanism.
# Set to a generous value — you will save the real heights via BTN_4/6/7 in JOG mode.
SOFT_MAX_TICKS = 8000

# Timeouts
HOME_TIMEOUT_S     = 15.0   # homing can take a while
MOVE_TIMEOUT_S     = 10.0   # position moves
RETURN_TIMEOUT_S   = 12.0   # return-to-origin on shutdown

# Backoff after homing: move this many ticks away from the limit switch so
# the switch is no longer pressed when we call reset_motor_position().
HOME_BACKOFF_TICKS = 30


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


def led_jog(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 0)
    robot.set_led(LED.BLUE, 200)


def led_done(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200, mode=LEDMode.BLINK, period_ms=400)
    robot.set_led(LED.RED, 0)
    robot.set_led(LED.BLUE, 0)


def led_error(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 200)
    robot.set_led(LED.BLUE, 0)


# ===========================================================================
# LOW-LEVEL MOTOR HELPERS
# ===========================================================================

def get_lift_ticks(robot: Robot) -> int:
    """Return the current encoder position of the lift motor (ticks)."""
    dc = robot.get_dc_state()
    if dc is None:
        return 0
    return int(dc.motors[LIFT_MOTOR - 1].position)


def move_lift_to(robot: Robot, ticks: int, timeout: float = MOVE_TIMEOUT_S) -> bool:
    """
    Move the lift to an absolute encoder position.
    Clamps to [0, SOFT_MAX_TICKS] to protect the mechanism.
    Returns True if the target was reached within tolerance.
    """
    clamped = max(0, min(int(ticks), SOFT_MAX_TICKS))
    if clamped != ticks:
        print(f"[warn] Target {ticks} ticks clamped to {clamped} (soft limit)")
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    ok = robot.set_motor_position(
        LIFT_MOTOR,
        clamped,
        max_vel_ticks=JOG_VELOCITY_TICKS,
        tolerance_ticks=POSITION_TOLERANCE,
        blocking=True,
        timeout=timeout,
    )
    if not ok:
        print(f"[warn] Lift did not reach {clamped} ticks within {timeout:.1f}s")
    return ok


# ===========================================================================
# HOMING — RETURN TO ORIGIN
# ===========================================================================

def home_lift(robot: Robot) -> bool:
    """
    Drive the lift to the origin (lowest position) and zero the encoder there.

    Option A  (USE_LIMIT_SWITCH = True):
        Runs the motor in HOME_DIRECTION at HOME_VELOCITY_TICKS until the
        limit switch fires, then backs off HOME_BACKOFF_TICKS and zeroes.

    Option B  (USE_LIMIT_SWITCH = False):
        The lift must already be at its hard-stop (lowered manually).
        Just resets the encoder at the current position.

    Returns True on success.
    """
    print("[HOME] Returning lift to origin …")
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)

    if USE_LIMIT_SWITCH:
        print(f"[HOME] Homing against limit switch (LIM_{LIFT_HOME_LIMIT}) …")
        ok = robot.home_motor(
            LIFT_MOTOR,
            direction=HOME_DIRECTION,
            home_velocity=HOME_VELOCITY_TICKS,
            blocking=True,
            timeout=HOME_TIMEOUT_S,
        )
        if not ok:
            print("[warn] HOME — limit switch not triggered within timeout.")
            print("       Check the limit switch wiring and HOME_DIRECTION setting.")
            return False

        # Back off so the switch is released before zeroing
        if HOME_BACKOFF_TICKS > 0:
            backoff_dir = -HOME_DIRECTION   # opposite of homing direction
            back_target = HOME_BACKOFF_TICKS if backoff_dir > 0 else -HOME_BACKOFF_TICKS
            # Use raw velocity mode for the tiny backoff to avoid chicken-and-egg
            # (we don't have a zeroed reference yet)
            robot.enable_motor(LIFT_MOTOR, DCMotorMode.VELOCITY)
            robot.set_motor_velocity(LIFT_MOTOR, backoff_dir * HOME_VELOCITY_TICKS)
            time.sleep(abs(HOME_BACKOFF_TICKS) / HOME_VELOCITY_TICKS + 0.1)
            robot.set_motor_velocity(LIFT_MOTOR, 0)
            time.sleep(0.1)
            robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)

    else:
        # No limit switch — trust that the operator lowered the lift manually
        print("[HOME] No limit switch — zeroing at current (manually lowered) position.")

    # Zero the encoder — this is the repeatable origin
    robot.reset_motor_position(LIFT_MOTOR)
    time.sleep(0.15)   # allow the firmware to process the reset
    ticks_after = get_lift_ticks(robot)
    print(f"[HOME] Encoder zeroed.  Current reading: {ticks_after} ticks (should be ~0)")
    return True


def return_to_origin(robot: Robot) -> None:
    """
    Move the lift back to encoder position 0 (the homed origin) and then
    disable the motor so the next power-on also reads 0 at that position.

    Call this before every shutdown — calibration or mission.
    """
    print("\n[ORIGIN] ── Returning lift to origin before shutdown ──")
    current = get_lift_ticks(robot)
    print(f"[ORIGIN] Current position: {current} ticks")

    if abs(current) <= POSITION_TOLERANCE:
        print("[ORIGIN] Already at origin — no move needed.")
    else:
        robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
        ok = robot.set_motor_position(
            LIFT_MOTOR,
            0,
            max_vel_ticks=JOG_VELOCITY_TICKS,
            tolerance_ticks=POSITION_TOLERANCE,
            blocking=True,
            timeout=RETURN_TIMEOUT_S,
        )
        if ok:
            print("[ORIGIN] Lift reached origin (0 ticks).")
        else:
            print("[warn] ORIGIN — lift may not have reached 0 — check mechanism.")

    robot.disable_motor(LIFT_MOTOR)
    print("[ORIGIN] Motor disabled.  Safe to power off.")


# ===========================================================================
# STARTUP
# ===========================================================================

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


# ===========================================================================
# MAIN FSM
# ===========================================================================


def _print_save_status(
    carry:   "int | None",
    pickup:  "int | None",
    dropoff: "int | None",
) -> None:
    """Print a compact one-line status showing which positions are saved."""
    def _s(label: str, val: "int | None") -> str:
        return f"{label}={val}" if val is not None else f"{label}=--"
    print(
        f"  Status → {_s('CARRY', carry)}  "
        f"{_s('PICKUP', pickup)}  "
        f"{_s('DROPOFF', dropoff)}"
    )
    missing = [n for n, v in [("CARRY", carry), ("PICKUP", pickup), ("DROPOFF", dropoff)] if v is None]
    if missing:
        print(f"  Still needed: {', '.join(missing)}")
    else:
        print("  All three positions saved — press BTN_5 to finish.")


def run(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)

    state = "INIT"

    # ── Saved tick values — filled by BTN_4 / BTN_6 / BTN_7 in JOG state ──
    # None means that button has not been pressed yet this session.
    saved_carry_ticks:   int | None = None   # BTN_4 → LIFT_CARRY_TICKS
    saved_pickup_ticks:  int | None = None   # BTN_6 → LIFT_PICKUP_TICKS
    saved_dropoff_ticks: int | None = None   # BTN_7 → LIFT_DROPOFF_TICKS
    session_log:         list[str]  = []

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print("=" * 62)
    print("  LIFT CALIBRATION TOOL — MAE 162D/E")
    print("=" * 62)
    lim_str = "LIM_" + str(int(LIFT_HOME_LIMIT)) if USE_LIMIT_SWITCH else "DISABLED (Sharpie-mark)"
    print(f"  Limit switch : {lim_str}")
    print(f"  Jog step     : {JOG_STEP_TICKS} ticks per press")
    print(f"  Soft max     : {SOFT_MAX_TICKS} ticks")
    print()
    print("  JOG CONTROLS (active after homing):")
    print(f"    BTN_2  jog UP   (+{JOG_STEP_TICKS} ticks)")
    print(f"    BTN_3  jog DOWN (-{JOG_STEP_TICKS} ticks)")
    print(f"    BTN_4  save LIFT_CARRY_TICKS   (burger-in-hand travel height)")
    print(f"    BTN_6  save LIFT_PICKUP_TICKS  (lower to grab ingredient)")
    print(f"    BTN_7  save LIFT_DROPOFF_TICKS (lower to place at drop-off)")
    print(f"    BTN_5  return to origin and print results")
    print("=" * 62)

    while True:

        # ==================================================================
        # INIT — start firmware
        # ==================================================================
        if state == "INIT":
            start_robot(robot)
            led_idle(robot)
            print("\n[FSM] INIT → WAIT_START")
            print("  Press BTN_1 to begin homing and calibration.")
            print("  The lift will move to the origin before anything else.")
            state = "WAIT_START"

        # ==================================================================
        # WAIT_START — idle until the operator is ready
        # ==================================================================
        elif state == "WAIT_START":
            if robot.was_button_pressed(Button.BTN_1):
                led_moving(robot)
                print("\n[FSM] WAIT_START → HOMING")
                state = "HOMING"

        # ==================================================================
        # HOMING — drive to origin and zero encoder
        # ==================================================================
        elif state == "HOMING":
            ok = home_lift(robot)
            if ok:
                pos = get_lift_ticks(robot)
                print(f"[FSM] HOMING complete.  Origin set at {pos} ticks.")
                session_log.append(f"Homed: origin = {pos} ticks")
                print("\n[FSM] HOMING complete → JOG mode")
                print("  Jog the lift to each target height and press the matching save button.")
                print(f"  BTN_2  jog UP   (+{JOG_STEP_TICKS} ticks)")
                print(f"  BTN_3  jog DOWN (-{JOG_STEP_TICKS} ticks)")
                print(f"  BTN_4  save LIFT_CARRY_TICKS   (travel height with burger gripped)")
                print(f"  BTN_6  save LIFT_PICKUP_TICKS  (height to grab an ingredient)")
                print(f"  BTN_7  save LIFT_DROPOFF_TICKS (height to place at drop-off)")
                print(f"  BTN_5  return to origin and print results")
                led_jog(robot)
                state = "JOG"
            else:
                print("[FSM] HOMING failed.  Check wiring / HOME_DIRECTION and try again.")
                led_error(robot)
                print("  Press BTN_1 to retry homing.")
                state = "HOME_RETRY"

        # ==================================================================
        # HOME_RETRY — wait for retry after a failed home
        # ==================================================================
        elif state == "HOME_RETRY":
            if robot.was_button_pressed(Button.BTN_1):
                led_moving(robot)
                state = "HOMING"

        # ==================================================================
        # JOG — operator jogs the lift up/down to find the raised position
        # ==================================================================
        elif state == "JOG":
            current_ticks = get_lift_ticks(robot)

            # ── BTN_2 = jog UP ────────────────────────────────────────────
            if robot.was_button_pressed(Button.BTN_2):
                target = current_ticks + JOG_STEP_TICKS
                if target > SOFT_MAX_TICKS:
                    print(f"[JOG] Soft limit reached ({SOFT_MAX_TICKS} ticks) — cannot go higher")
                else:
                    print(f"[JOG] UP  → {target} ticks")
                    move_lift_to(robot, target)
                    new_pos = get_lift_ticks(robot)
                    print(f"[JOG] Position now: {new_pos} ticks")
                    session_log.append(f"Jog UP  → {new_pos} ticks")

            # ── BTN_3 = jog DOWN ──────────────────────────────────────────
            elif robot.was_button_pressed(Button.BTN_3):
                target = current_ticks - JOG_STEP_TICKS
                if target < 0:
                    print("[JOG] Already at or below origin — cannot go lower")
                else:
                    print(f"[JOG] DOWN → {target} ticks")
                    move_lift_to(robot, target)
                    new_pos = get_lift_ticks(robot)
                    print(f"[JOG] Position now: {new_pos} ticks")
                    session_log.append(f"Jog DOWN → {new_pos} ticks")

            # ── BTN_4 = save LIFT_CARRY_TICKS ────────────────────────────
            elif robot.was_button_pressed(Button.BTN_4):
                saved_carry_ticks = get_lift_ticks(robot)
                tag = "(updated)" if saved_carry_ticks is not None else ""
                print(f"\n[SAVE] LIFT_CARRY_TICKS = {saved_carry_ticks}  {tag}")
                print("         Carry height: lift holds this position while driving.")
                session_log.append(f"SAVED  LIFT_CARRY_TICKS   = {saved_carry_ticks}")
                _print_save_status(saved_carry_ticks, saved_pickup_ticks, saved_dropoff_ticks)

            # ── BTN_6 = save LIFT_PICKUP_TICKS ───────────────────────────
            elif robot.was_button_pressed(Button.BTN_6):
                saved_pickup_ticks = get_lift_ticks(robot)
                tag = "(updated)" if saved_pickup_ticks is not None else ""
                print(f"\n[SAVE] LIFT_PICKUP_TICKS = {saved_pickup_ticks}  {tag}")
                print("         Pickup height: claw lowers to this to grab an ingredient.")
                session_log.append(f"SAVED  LIFT_PICKUP_TICKS  = {saved_pickup_ticks}")
                _print_save_status(saved_carry_ticks, saved_pickup_ticks, saved_dropoff_ticks)

            # ── BTN_7 = save LIFT_DROPOFF_TICKS ──────────────────────────
            elif robot.was_button_pressed(Button.BTN_7):
                saved_dropoff_ticks = get_lift_ticks(robot)
                tag = "(updated)" if saved_dropoff_ticks is not None else ""
                print(f"\n[SAVE] LIFT_DROPOFF_TICKS = {saved_dropoff_ticks}  {tag}")
                print("         Drop-off height: claw lowers to this to place the burger.")
                session_log.append(f"SAVED  LIFT_DROPOFF_TICKS = {saved_dropoff_ticks}")
                _print_save_status(saved_carry_ticks, saved_pickup_ticks, saved_dropoff_ticks)

            # ── BTN_5 = return to origin and print results ────────────────
            elif robot.was_button_pressed(Button.BTN_5):
                print("\n[FSM] JOG → RETURN_ORIGIN (BTN_5 pressed)")
                state = "RETURN_ORIGIN"

        # ==================================================================
        # RETURN_ORIGIN — move lift back to 0 before shutdown
        # ==================================================================
        elif state == "RETURN_ORIGIN":
            return_to_origin(robot)
            state = "PRINT_RESULTS"

        # ==================================================================
        # PRINT_RESULTS — display the calibration block the user needs
        # ==================================================================
        elif state == "PRINT_RESULTS":
            led_done(robot)

            def _fmt(val: "int | None", btn: str) -> str:
                return str(val) if val is not None else f"NOT SET ({btn} was never pressed)"

            carry   = _fmt(saved_carry_ticks,   "BTN_4")
            pickup  = _fmt(saved_pickup_ticks,  "BTN_6")
            dropoff = _fmt(saved_dropoff_ticks, "BTN_7")

            print()
            print("=" * 62)
            print("  CALIBRATION COMPLETE")
            print("  Copy these values into main_full_mission.py")
            print("=" * 62)
            print()
            print(f"  LIFT_CARRY_TICKS   = {carry}")
            print(f"  LIFT_PICKUP_TICKS  = {pickup}")
            print(f"  LIFT_DROPOFF_TICKS = {dropoff}")
            print(f"  LIFT_DOWN_TICKS    = 0          # always 0 — the Sharpie-mark origin")
            print()
            all_set = all(v is not None for v in
                          (saved_carry_ticks, saved_pickup_ticks, saved_dropoff_ticks))
            if not all_set:
                print("  !! One or more values were NOT saved this session.")
                print("     Re-run calibration or set them manually before the mission.")
                print()
            print("  Full jog log for this session:")
            for entry in session_log:
                print(f"    {entry}")
            print()
            print("=" * 62)
            print("  SHUTDOWN RULE — READ BEFORE POWERING OFF")
            print("=" * 62)
            print("  The lift is now at the Sharpie-mark origin (encoder 0).")
            print("  The robot is SAFE to power off.")
            print()
            print("  The encoder resets to 0 at whatever physical position")
            print("  the lift is in when power is removed.  If the lift is")
            print("  raised at power-off, the saved tick values will be wrong")
            print("  on the next run — re-calibration will be required.")
            print()
            print("  main_full_mission.py calls lift_return_to_zero() at")
            print("  mission end and on E-stop to enforce this automatically.")
            print("=" * 62)
            print()
            print("  Press BTN_1 to re-home and run another calibration pass.")
            print("  Press BTN_5 to exit (lift stays at origin; safe to power off).")
            state = "DONE"

        # ==================================================================
        # DONE — offer to re-run or exit
        # ==================================================================
        elif state == "DONE":
            if robot.was_button_pressed(Button.BTN_1):
                # Reset all saved slots for a fresh calibration pass
                saved_carry_ticks   = None
                saved_pickup_ticks  = None
                saved_dropoff_ticks = None
                session_log         = []
                led_moving(robot)
                print("\n[FSM] DONE → HOMING (new calibration pass)")
                state = "HOMING"

            elif robot.was_button_pressed(Button.BTN_5):
                # Final safety check — make sure we are at origin before truly exiting
                current_ticks = get_lift_ticks(robot)
                if abs(current_ticks) > POSITION_TOLERANCE:
                    print(f"[warn] Lift is at {current_ticks} ticks — returning to origin first")
                    return_to_origin(robot)
                print("[FSM] Calibration tool exiting.  Safe to power off.")
                robot.stop()
                break

        # ── Tick-rate control ──────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
