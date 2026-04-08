"""
main.py — student entry point
==============================
This is the only file you need to edit.

`run(robot)` is called by robot_node.py after ROS is set up and the Robot
instance is ready. Write your FSM here and call fsm.spin() at the end.

To run:
    ros2 run robot robot
"""

import math
import time

import numpy as np

from robot.robot import Robot, FirmwareState
from robot.robot_fsm import RobotFSM
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.path_planner import PurePursuitPlanner


# Drive-wheel mapping for this robot build.
# Change these if the left/right wheels are wired to different DC motor ports.
LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# Odometry / diff-drive geometry for this robot build.
# The path planner assumes pose is reported in millimetres and heading in radians.
# Set these once at startup so firmware odometry and robot-side body-motion
# mixing agree on the same wheel geometry and motor mapping.
WHEEL_DIAMETER_MM = 74.0
WHEEL_BASE_MM = 333.0
INITIAL_THETA_DEG = 90.0
LOOKAHEAD_MM = 100.0
MAX_LINEAR_SPEED_MM_S = 100.0
MAX_ANGULAR_SPEED_RAD_S = 1.0


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
        self.path = self._densify_polyline(
            np.array([
                [0.0, 0.0],
                [0.0, 500.0],
                [500.0, 500.0],
                [500.0, 0.0],
            ]),
            spacing_mm=25.0,
        )
        self.remaining_path = self.path.copy()
        self.planner = PurePursuitPlanner(
            lookahead_dist=LOOKAHEAD_MM,
            max_angular=MAX_ANGULAR_SPEED_RAD_S,
            goal_tolerance=20.0,
        )
        self._next_debug_log_time = 0.0

        self.add_transition("INIT",   "ready",     "IDLE",   action=self._on_ready)
        self.add_transition("IDLE",   "to_moving", "MOVING", action=self._on_to_moving)
        self.add_transition("MOVING", "to_idle",   "IDLE",   action=self._on_to_idle)

    # ------------------------------------------------------------------
    # Part 2 — Continuous logic (runs every spin cycle, ~50 Hz by default)
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
            self.trigger("ready")

        elif state == "IDLE":
            if self.robot.get_button(Button.BTN_1):
                self.trigger("to_moving")

        elif state == "MOVING":
            current_x, current_y, _ = self.robot.get_pose()
            current_theta_deg = self.robot.get_fused_orientation()
            current_theta_rad = math.radians(current_theta_deg)
            self._advance_remaining_path(current_x, current_y)
            current_pursuit_x, current_pursuit_y = self.planner._lookahead_point(
                current_x,
                current_y,
                waypoints=self.remaining_path,
            )
            linear_velocity_cmd, angular_velocity_cmd_rad_s = self.planner.compute_velocity(
                pose=(current_x, current_y, current_theta_rad),
                waypoints=self.remaining_path,
                max_linear=MAX_LINEAR_SPEED_MM_S,
            )
            self.robot.set_velocity(
                linear_velocity_cmd,
                math.degrees(angular_velocity_cmd_rad_s),
            )

            now = time.monotonic()
            if now >= self._next_debug_log_time:
                print(
                    "MOVING: pose=(%.2f, %.2f, %.2f deg / %.2f rad) "
                    "target=(%.2f, %.2f) lookahead=(%.2f, %.2f) command=(%.2f mm/s, %.2f rad/s)"
                    % (
                        current_x,
                        current_y,
                        current_theta_deg,
                        current_theta_rad,
                        self.remaining_path[0][0],
                        self.remaining_path[0][1],
                        current_pursuit_x,
                        current_pursuit_y,
                        linear_velocity_cmd,
                        angular_velocity_cmd_rad_s,
                    )
                )
                self._next_debug_log_time = now + 0.25

            if self.planner.TargetReached(current_x, current_y, self.remaining_path):
                print("MOVING: Target reached! Stopping.")
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
        self.robot.reset_odometry()
        self.robot.wait_for_pose_update(timeout=0.2)
        self.remaining_path = self.path[1:].copy() if len(self.path) > 1 else self.path.copy()
        self._show_idle_leds()
        print("[FSM] IDLE (odometry reset)")

    def _on_to_moving(self) -> None:
        """Called once when entering MOVING."""
        self._show_moving_leds()
        print("[FSM] MOVING")

    def _on_to_idle(self) -> None:
        """Called once when entering IDLE from MOVING."""
        self.robot.stop()
        self._show_idle_leds()
        print("[FSM] IDLE")

    def _show_idle_leds(self) -> None:
        self.robot.set_led(LED.GREEN, 0)
        self.robot.set_led(LED.RED, 255)

    def _show_moving_leds(self) -> None:
        self.robot.set_led(LED.RED, 0)
        self.robot.set_led(LED.GREEN, 255)

    def _advance_remaining_path(self, current_x: float, current_y: float) -> None:
        advance_radius = max(self.planner.goal_tolerance, self.planner._lookahead)
        while len(self.remaining_path) > 1:
            next_x, next_y = self.remaining_path[0]
            if math.hypot(next_x - current_x, next_y - current_y) > advance_radius:
                break
            self.remaining_path = self.remaining_path[1:]
            print(
                "[FSM] advancing to next waypoint: (%.2f, %.2f)"
                % (self.remaining_path[0][0], self.remaining_path[0][1])
            )

    @staticmethod
    def _densify_polyline(control_points: np.ndarray, spacing_mm: float) -> np.ndarray:
        dense_points: list[list[float]] = [[float(control_points[0][0]), float(control_points[0][1])]]
        for start, end in zip(control_points[:-1], control_points[1:]):
            dx = float(end[0] - start[0])
            dy = float(end[1] - start[1])
            segment_length = math.hypot(dx, dy)
            steps = max(1, int(math.ceil(segment_length / spacing_mm)))
            for step in range(1, steps + 1):
                ratio = step / steps
                dense_points.append([
                    float(start[0] + dx * ratio),
                    float(start[1] + dy * ratio),
                ])
        return np.array(dense_points)


def run(robot: Robot) -> None:
    robot.set_odometry_parameters(
        wheel_diameter_mm=WHEEL_DIAMETER_MM,
        wheel_base_mm=WHEEL_BASE_MM,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    fsm = MyFSM(robot)
    fsm.spin(hz=DEFAULT_FSM_HZ)
