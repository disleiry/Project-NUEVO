"""
main.py — student entry point
==============================
This is the only file you need to edit.

`run(robot)` is called by robot_node.py after ROS is set up and the Robot
instance is ready. Write your FSM here and call fsm.spin() at the end.

To run:
    ros2 run robot robot
"""

from robot.robot import Robot, FirmwareState
from robot.robot_fsm import RobotFSM


# ---------------------------------------------------------------------------
# LED identifiers for set_led()
#
# Usage:
#   self.robot.set_led(LED.GREEN, 255)   # full brightness
#   self.robot.set_led(LED.RED,   0)     # off
# ---------------------------------------------------------------------------

class LED:
    RED    = 0   # Discrete red LED    (PWM)
    GREEN  = 1   # Discrete green LED  (PWM)
    BLUE   = 2   # Discrete blue LED   (PWM)
    ORANGE = 3   # Discrete orange LED (PWM)
    PURPLE = 4   # Discrete purple LED (non-PWM — brightness is 0 or 255 only)

    OFF = 0      # Brightness: off
    ON  = 255    # Brightness: full on


class MyFSM(RobotFSM):
    """
    Three-state FSM: INIT → IDLE ↔ MOVING

    INIT  : one-time startup — starts the firmware, then immediately goes to IDLE
    IDLE  : waiting; RED LED on. Press button 1 to start moving.
    MOVING: driving forward; GREEN LED on. Press button 2 to stop.

    ┌──────┐  ready      ┌────────┐  to_moving  ┌────────┐
    │ INIT │ ──────────> │  IDLE  │ ──────────> │ MOVING │
    └──────┘             │        │ <────────── │        │
                         └────────┘   to_idle   └────────┘
    """

    # ------------------------------------------------------------------
    # Part 1 — State machine setup
    #
    # add_transition(from_state, event, to_state, action)
    #   from_state : which state this transition starts from
    #   event      : the name you pass to trigger() to fire it
    #   to_state   : which state to move to
    #   action     : method to call once, at the moment of transition
    #
    # The event name and action method share the same word so it is easy
    # to trace:  trigger("to_moving") → _on_to_moving()
    # ------------------------------------------------------------------

    def __init__(self, robot: Robot) -> None:
        super().__init__(robot, initial_state="INIT")

        self.add_transition("INIT",   "ready",     "IDLE",   action=self._on_ready)
        self.add_transition("IDLE",   "to_moving", "MOVING", action=self._on_to_moving)
        self.add_transition("MOVING", "to_idle",   "IDLE",   action=self._on_to_idle)

        # Tracks the previous button state for edge detection (see _button_pressed)
        self._btn_prev: dict[int, bool] = {}

    # ------------------------------------------------------------------
    # Part 2 — Continuous logic (runs every spin cycle, ~20 Hz)
    #
    # update() is called repeatedly by spin(). Use it to:
    #   - read sensors or buttons
    #   - send continuous commands (LEDs, velocity adjustments)
    #   - decide when to trigger a transition
    #
    # Calling self.robot.xxx() here talks directly to hardware but does
    # NOT change the FSM state — it just sends a command right now.
    # Calling self.trigger("event") is what actually changes state.
    # ------------------------------------------------------------------

    def update(self) -> None:
        state = self.get_state()

        if state == "INIT":
            # No output — transition happens automatically on the first cycle.
            # trigger() changes state to IDLE and calls _on_ready() once.
            self.trigger("ready")

        elif state == "IDLE":
            # Continuous output while IDLE — runs every cycle
            self.robot.set_led(LED.GREEN, LED.OFF)
            self.robot.set_led(LED.RED,   LED.ON)

            # _button_pressed() fires only on the rising edge (moment of press).
            # This prevents the trigger from firing repeatedly while held down.
            if self._button_pressed(1):
                self.trigger("to_moving")

        elif state == "MOVING":
            # Continuous output while MOVING — runs every cycle
            self.robot.set_led(LED.RED,   LED.OFF)
            self.robot.set_led(LED.GREEN, LED.ON)

            if self._button_pressed(2):
                self.trigger("to_idle")

    # ------------------------------------------------------------------
    # Part 3 — Transition actions (called once, at the moment of change)
    #
    # These run exactly once when the transition fires — not every cycle.
    # Use them for one-shot commands: enabling the firmware, setting an
    # initial velocity, or cleanly stopping motors.
    # ------------------------------------------------------------------

    def _on_ready(self) -> None:
        """Called once when leaving INIT. Starts the firmware."""
        self.robot.set_state(FirmwareState.RUNNING)

    def _on_to_moving(self) -> None:
        """Called once when entering MOVING."""
        self.robot.set_velocity(linear=100, angular=0.0)  # 100 mm/s forward

    def _on_to_idle(self) -> None:
        """Called once when entering IDLE from MOVING."""
        self.robot.stop()

    # ------------------------------------------------------------------
    # Helper — edge-detection for buttons
    #
    # get_button() returns True the entire time a button is held down.
    # In update() running at 20 Hz, one press would fire trigger() up to
    # 20 times per second. _button_pressed() detects only the rising edge
    # (the moment the button goes from not-pressed to pressed), so each
    # physical press fires the trigger exactly once.
    # ------------------------------------------------------------------

    def _button_pressed(self, button_id: int) -> bool:
        current = self.robot.get_button(button_id)
        prev = self._btn_prev.get(button_id, False)
        self._btn_prev[button_id] = current
        return current and not prev


def run(robot: Robot) -> None:
    fsm = MyFSM(robot)
    fsm.spin(hz=20)
