"""
robot_impl/navigation.py — NavigationMixin

Odometry, sensor fusion callbacks, pose API, drive commands, and
higher-level path following.  Also defines Unit and MotionHandle since
they are tightly coupled to this layer.
All state lives in Robot.__init__; this mixin only defines methods.
"""
from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from enum import Enum

import numpy as np

from bridge_interfaces.msg import (
    FusedPose,
    SensorKinematics,
    SysOdomParamReq,
    SysOdomParamRsp,
    SysOdomParamSet,
    SysOdomReset,
)

from robot.hardware_map import DEFAULT_NAV_HZ
from robot.robot_impl.hardware import FirmwareState


# =============================================================================
# Public types
# =============================================================================

class Unit(Enum):
    MM   = 1.0
    INCH = 25.4
    AMERICAN = INCH
    REST_OF_THE_WORLD = MM


class MotionHandle:
    """
    Handle returned by all high-level base-motion commands.

    blocking=True waits before returning; the handle is already finished.
    blocking=False returns immediately; poll or wait as needed.

    Example:
        handle = robot.move_to(500, 0, 200, tolerance=15, blocking=False)
        # ... do other things ...
        success = handle.wait(timeout=10.0)
    """

    def __init__(self, finished_event: threading.Event, cancel_event: threading.Event) -> None:
        self._finished = finished_event
        self._cancel = cancel_event

    def wait(self, timeout: float = None) -> bool:
        """Block until motion finishes. Returns True if finished before timeout."""
        return self._finished.wait(timeout=timeout)

    def is_finished(self) -> bool:
        """Non-blocking poll. Returns True if motion has finished."""
        return self._finished.is_set()

    def is_done(self) -> bool:
        """Backward-compatible alias for is_finished()."""
        return self.is_finished()

    def cancel(self) -> None:
        """Abort this motion command. The robot will stop."""
        self._cancel.set()


# =============================================================================
# Module-level helpers (used by navigation internals)
# =============================================================================

def _dist2d(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def _wrap_angle(a: float) -> float:
    """Wrap angle to [−π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# =============================================================================
# NavigationMixin
# =============================================================================

class NavigationMixin:
    """Odometry config, pose, drive commands, and path following."""

    # =========================================================================
    # Subscription callbacks
    # =========================================================================

    def _on_kinematics(self, msg: SensorKinematics) -> None:
        import time as _time  # local import to avoid shadowing module-level time
        with self._lock:
            self._pose = (msg.x, msg.y, msg.theta)
            self._vel  = (msg.vx, msg.vy, msg.v_theta)

            _initial_theta_rad = math.radians(self._initial_theta_deg)
            if self._ahrs_heading is not None:
                if self._odom_reset_pending:
                    if abs(math.atan2(
                        math.sin(msg.theta - _initial_theta_rad),
                        math.cos(msg.theta - _initial_theta_rad),
                    )) < math.radians(5.0):
                        self._odom_reset_pending = False
                        self._odom_reset_event.set()
                    relative_ahrs = None
                else:
                    relative_ahrs = self._ahrs_heading
            else:
                if self._odom_reset_pending:
                    if abs(math.atan2(
                        math.sin(msg.theta - _initial_theta_rad),
                        math.cos(msg.theta - _initial_theta_rad),
                    )) < math.radians(5.0):
                        self._odom_reset_pending = False
                        self._odom_reset_event.set()
                relative_ahrs = None

            linear_vel  = math.hypot(float(msg.vx), float(msg.vy))
            angular_vel = float(msg.v_theta)
            self._fused_theta = self._orientation_fusion.update(
                odom_theta  = msg.theta,
                mag_heading = relative_ahrs,
                linear_vel  = linear_vel,
                angular_vel = angular_vel,
            )

            gps_fresh = (
                self._gps_last_time > 0.0
                and (_time.monotonic() - self._gps_last_time) < self._gps_timeout_s
            )
            gps_x = self._gps_x_mm if gps_fresh else None
            gps_y = self._gps_y_mm if gps_fresh else None
            self._fused_x_mm, self._fused_y_mm = self._pos_fusion.update(
                msg.x, msg.y, gps_x, gps_y
            )
            _raw_odom  = (float(msg.x), float(msg.y))
            _raw_fused = (self._fused_x_mm, self._fused_y_mm)

        self._odom_traj.append(_raw_odom)
        self._fused_traj.append(_raw_fused)
        self._node.get_logger().info(
            f"odom=({_raw_odom[0]:.1f}, {_raw_odom[1]:.1f}) mm  "
            f"fused=({_raw_fused[0]:.1f}, {_raw_fused[1]:.1f}) mm",
            throttle_duration_sec=0.5,
        )

        _fp = FusedPose()
        _fp.header.stamp = self._node.get_clock().now().to_msg()
        _fp.x     = float(_raw_fused[0])
        _fp.y     = float(_raw_fused[1])
        _fp.theta = float(self._fused_theta)
        _fp.gps_active = bool(gps_fresh)
        self._pub_fused_pose.publish(_fp)

        self._pose_event.set()
        self._pose_event.clear()

    def _on_odom_param_rsp(self, msg: SysOdomParamRsp) -> None:
        if not self._odom_user_configured:
            self._apply_odom_param_snapshot(
                float(msg.wheel_diameter_mm),
                float(msg.wheel_base_mm),
                float(msg.initial_theta_deg),
                int(msg.left_motor_number),
                bool(msg.left_motor_dir_inverted),
                int(msg.right_motor_number),
                bool(msg.right_motor_dir_inverted),
            )
            return

        with self._lock:
            in_sync = (
                abs(float(msg.wheel_diameter_mm) - self._wheel_diameter) < 1e-3
                and abs(float(msg.wheel_base_mm) - self._wheel_base) < 1e-3
                and abs(float(msg.initial_theta_deg) - self._initial_theta_deg) < 1e-3
                and int(msg.left_motor_number) == self._left_wheel_motor
                and bool(msg.left_motor_dir_inverted) == self._left_wheel_dir_inverted
                and int(msg.right_motor_number) == self._right_wheel_motor
                and bool(msg.right_motor_dir_inverted) == self._right_wheel_dir_inverted
            )
            if in_sync:
                snapshot = None
            else:
                snapshot = (
                    self._wheel_diameter,
                    self._wheel_base,
                    self._initial_theta_deg,
                    self._left_wheel_motor,
                    self._left_wheel_dir_inverted,
                    self._right_wheel_motor,
                    self._right_wheel_dir_inverted,
                )

        if in_sync:
            self._odom_confirm_event.set()
        elif snapshot is not None:
            self._publish_odom_params(snapshot)

    # =========================================================================
    # Odometry configuration
    # =========================================================================

    def reset_odometry(self) -> None:
        """Reset firmware odometry pose to (0, 0, initial_theta).

        Captures the AHRS zero reference on the first kinematics tick after the
        firmware confirms the reset, so both references share the same timestamp.
        """
        with self._lock:
            self._odom_reset_pending = True
            self._odom_reset_event.clear()
            self._pos_fusion.reset()
        msg = SysOdomReset()
        msg.flags = 0
        self._odom_pub.publish(msg)

    def set_wheel_diameter_mm(self, wheel_diameter_mm: float) -> None:
        """Update wheel diameter in explicit millimeters."""
        self._update_odometry_params(wheel_diameter_mm=wheel_diameter_mm)

    def set_wheel_base_mm(self, wheel_base_mm: float) -> None:
        """Update wheel base in explicit millimeters."""
        self._update_odometry_params(wheel_base_mm=wheel_base_mm)

    def set_initial_theta(self, theta_deg: float) -> None:
        """Update the heading used by future odometry resets."""
        self._update_odometry_params(initial_theta_deg=theta_deg)

    def set_odom_left_motor(self, motor_id: int) -> None:
        """Select which DC motor is treated as the left wheel."""
        self._update_odometry_params(left_wheel_motor=motor_id)

    def set_odom_right_motor(self, motor_id: int) -> None:
        """Select which DC motor is treated as the right wheel."""
        self._update_odometry_params(right_wheel_motor=motor_id)

    def set_odom_motors(self, left_motor_id: int, right_motor_id: int) -> None:
        """Atomically configure both odometry wheel motors."""
        self._update_odometry_params(
            left_wheel_motor=left_motor_id,
            right_wheel_motor=right_motor_id,
        )

    def set_odom_left_motor_dir_inverted(self, inverted: bool) -> None:
        """Configure whether positive left-wheel counts mean reverse motion."""
        self._update_odometry_params(left_wheel_dir_inverted=inverted)

    def set_odom_right_motor_dir_inverted(self, inverted: bool) -> None:
        """Configure whether positive right-wheel counts mean reverse motion."""
        self._update_odometry_params(right_wheel_dir_inverted=inverted)

    def set_odometry_parameters(
        self,
        *,
        wheel_diameter: float | None = None,
        wheel_base: float | None = None,
        initial_theta_deg: float | None = None,
        left_motor_id: int | None = None,
        left_motor_dir_inverted: bool | None = None,
        right_motor_id: int | None = None,
        right_motor_dir_inverted: bool | None = None,
        timeout: float = 1.0,
    ) -> bool:
        """
        Publish a full odometry-parameter snapshot to firmware and wait for confirmation.

        wheel_diameter and wheel_base are in the current user unit system.
        timeout — seconds to wait for a matching firmware echo. Pass 0 to return
        immediately after publishing.
        Returns True if the firmware confirmed the params within timeout.
        """
        scale = self._unit.value
        return self._update_odometry_params(
            wheel_diameter_mm=None if wheel_diameter is None else float(wheel_diameter) * scale,
            wheel_base_mm=None if wheel_base is None else float(wheel_base) * scale,
            initial_theta_deg=initial_theta_deg,
            left_wheel_motor=left_motor_id,
            left_wheel_dir_inverted=left_motor_dir_inverted,
            right_wheel_motor=right_motor_id,
            right_wheel_dir_inverted=right_motor_dir_inverted,
            timeout=timeout,
        )

    def request_odometry_parameters(self) -> None:
        """Request the current firmware odometry parameter snapshot."""
        msg = SysOdomParamReq()
        msg.target = 0xFF
        self._odom_param_req_pub.publish(msg)

    def get_odometry_parameters(self) -> dict[str, float | int | bool]:
        """Return the latest local odometry parameter snapshot in firmware-native millimeters."""
        with self._lock:
            return {
                "wheel_diameter_mm": self._wheel_diameter,
                "wheel_base_mm": self._wheel_base,
                "initial_theta_deg": self._initial_theta_deg,
                "left_motor_number": self._left_wheel_motor,
                "left_motor_dir_inverted": self._left_wheel_dir_inverted,
                "right_motor_number": self._right_wheel_motor,
                "right_motor_dir_inverted": self._right_wheel_dir_inverted,
            }

    # =========================================================================
    # Pose / odometry
    # =========================================================================

    def get_pose(self) -> tuple[float, float, float]:
        """
        Return the current pose as (x, y, theta_deg) in user units and degrees.

        x and y are GPS-fused when a recent GPS fix is available, otherwise
        raw wheel-odometry position.  theta is sensor-fused heading.
        """
        x_mm, y_mm, theta_rad = self._get_pose_mm()
        s = self._unit.value
        return x_mm / s, y_mm / s, math.degrees(theta_rad)

    def get_velocity(self) -> tuple[float, float, float]:
        """Return (vx, vy, v_theta_deg_s) in user units/s and degrees/s."""
        with self._lock:
            vx, vy, vt = self._vel
        s = self._unit.value
        return vx / s, vy / s, math.degrees(vt)

    def wait_for_pose_update(self, timeout: float = None) -> bool:
        """Block until the next /sensor_kinematics message arrives (~25 Hz)."""
        return self._pose_event.wait(timeout=timeout)

    def wait_for_odometry_reset(self, timeout: float = 2.0) -> bool:
        """Block until the firmware confirms the odometry reset (theta ≈ initial_theta).

        Returns True if the reset was confirmed within timeout, False on timeout.
        """
        return self._odom_reset_event.wait(timeout=timeout)

    def get_fused_pose(self) -> tuple[float, float, float]:
        """
        Return the fused (x, y, theta_deg) in user units and degrees.

        Position is a complementary-filter blend of GPS and odometry.
        Orientation is a complementary-filter blend of AHRS and odometry.
        """
        return self.get_pose()

    # =========================================================================
    # Obstacles
    # =========================================================================

    def set_obstacles(self, obstacles: list[tuple[float, float]]) -> None:
        """Cache APF obstacles in the robot frame using the current user length unit."""
        converted = np.float64([
            (
                self._require_finite_float("obstacle_x", obs_x) * self._unit.value,
                self._require_finite_float("obstacle_y", obs_y) * self._unit.value,
            )
            for obs_x, obs_y in obstacles
        ])
        with self._lock:
            self._obstacles_mm = converted

    def clear_obstacles(self) -> None:
        """Clear the cached APF obstacle list set by set_obstacles()."""
        with self._lock:
            self._obstacles_mm = np.float64([])

    def get_obstacles(self) -> list[tuple[float, float]]:
        """Return the current APF obstacles in the current user length unit."""
        scale = self._unit.value
        return [(x_mm / scale, y_mm / scale) for x_mm, y_mm in self._get_obstacles_mm()]

    def set_obstacle_provider(
        self,
        provider: Callable[[], list[tuple[float, float]]] | None,
    ) -> None:
        """Register a live APF obstacle callback that returns robot-frame positions in mm."""
        if provider is not None and not callable(provider):
            raise TypeError("provider must be callable or None")
        with self._lock:
            self._obstacle_provider = provider

    # =========================================================================
    # Differential drive — velocity commands
    # =========================================================================

    def set_velocity(self, linear: float, angular_deg_s: float) -> None:
        """
        Body-frame velocity command.
          linear        — forward speed in user units/s
          angular_deg_s — rotation rate in degrees/s (CCW positive)
        """
        linear_mm     = linear * self._unit.value
        angular_rad_s = math.radians(angular_deg_s)
        self._send_body_velocity_mm(linear_mm, angular_rad_s)

    def set_motor_velocity(self, motor_id: int, velocity: float) -> None:
        """Command a single DC motor by velocity in user units/s."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        self._send_motor_velocity_mm(motor_id, velocity * self._unit.value)

    def stop(self) -> None:
        """Zero velocity on both drive motors. Firmware stays in RUNNING state."""
        self._send_motor_velocity_mm(self._left_wheel_motor, 0.0)
        self._send_motor_velocity_mm(self._right_wheel_motor, 0.0)

    def disable_drive_motors(self) -> None:
        """Disable the currently configured drive motors."""
        with self._lock:
            drive_motors = tuple(dict.fromkeys((self._left_wheel_motor, self._right_wheel_motor)))
        for motor_id in drive_motors:
            self.disable_motor(motor_id)

    def shutdown(self) -> None:
        """Stop navigation and leave the drive motors disabled."""
        self.cancel_motion()
        self.stop()
        time.sleep(self._SHUTDOWN_SETTLE_S)
        self.disable_drive_motors()
        time.sleep(self._SHUTDOWN_SETTLE_S)
        if self.get_state() == FirmwareState.RUNNING:
            self.set_state(FirmwareState.IDLE, timeout=1.0)
        self.save_trajectory_image()

    def set_left_wheel(self, motor_id: int) -> None:
        """Alias for set_odom_left_motor()."""
        self.set_odom_left_motor(motor_id)

    def set_right_wheel(self, motor_id: int) -> None:
        """Alias for set_odom_right_motor()."""
        self.set_odom_right_motor(motor_id)

    def set_drive_wheels(self, left_motor_id: int, right_motor_id: int) -> None:
        """Alias for set_odom_motors()."""
        self.set_odom_motors(left_motor_id, right_motor_id)

    def get_left_wheel(self) -> int:
        return self._left_wheel_motor

    def get_right_wheel(self) -> int:
        return self._right_wheel_motor

    # =========================================================================
    # Navigation (higher-level, runs a background thread)
    # =========================================================================

    def move_to(
        self,
        x: float,
        y: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ) -> MotionHandle:
        """
        Navigate to (x, y) at the given speed using pure-pursuit steering.

        Always returns a MotionHandle.
        blocking=True waits before returning the handle.
        """
        x_mm   = x         * self._unit.value
        y_mm   = y         * self._unit.value
        vel_mm = velocity  * self._unit.value
        tol_mm = tolerance * self._unit.value
        lookahead_mm = max(tol_mm * 2.0, 100.0)

        def target():
            self._nav_follow_purepursuit_path([(x_mm, y_mm)], vel_mm, lookahead_mm, tol_mm, 2.0)

        return self._start_nav(target, blocking, timeout)

    def move_by(
        self,
        dx: float,
        dy: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ) -> MotionHandle:
        """Navigate by (dx, dy) relative to current pose."""
        cur_x, cur_y, _ = self.get_pose()
        return self.move_to(cur_x + dx, cur_y + dy, velocity,
                            blocking=blocking, tolerance=tolerance, timeout=timeout)

    def move_forward(
        self,
        distance: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ) -> MotionHandle:
        """Navigate forward by distance along the robot's current heading."""
        distance = self._require_positive_float("distance", distance)
        return self._move_along_heading(distance, velocity, tolerance=tolerance,
                                        blocking=blocking, timeout=timeout)

    def move_backward(
        self,
        distance: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ) -> MotionHandle:
        """Navigate backward by distance along the robot's current heading."""
        distance = self._require_positive_float("distance", distance)
        return self._move_along_heading(-distance, velocity, tolerance=tolerance,
                                        blocking=blocking, timeout=timeout)

    def turn_to(
        self,
        angle_deg: float,
        blocking: bool = True,
        tolerance_deg: float = 2.0,
        timeout: float = None,
    ) -> MotionHandle:
        """
        Rotate to an absolute heading. Firmware must be in RUNNING state.
        angle_deg is in degrees (CCW positive, same convention as the firmware).

        Always returns a MotionHandle.
        """
        target_rad = math.radians(angle_deg)
        tol_rad    = math.radians(tolerance_deg)

        def target():
            self._turn_to_heading(target_rad, tol_rad)

        return self._start_nav(target, blocking, timeout)

    def turn_by(
        self,
        delta_deg: float,
        blocking: bool = True,
        tolerance_deg: float = 2.0,
        timeout: float = None,
    ) -> MotionHandle:
        """Rotate by delta_deg relative to current heading."""
        _, _, cur_deg = self.get_pose()
        return self.turn_to(cur_deg + delta_deg, blocking=blocking,
                            tolerance_deg=tolerance_deg, timeout=timeout)

    def purepursuit_follow_path(
        self,
        waypoints: list[tuple[float, float]],
        velocity: float,
        lookahead: float,
        tolerance: float,
        blocking: bool = True,
        max_angular_rad_s: float = 1.0,
        timeout: float = None,
        *,
        advance_radius: float | None = None,
    ) -> MotionHandle:
        """
        Follow an ordered waypoint path with pure pursuit.

        waypoints           — [(x, y), ...] in the current user unit system
        velocity            — maximum forward speed in user units/s
        lookahead           — lookahead distance in user units
        tolerance           — goal tolerance in user units
        advance_radius      — optional intermediate waypoint acceptance radius
                              in user units; defaults to tolerance
        max_angular_rad_s   — angular-rate clamp in rad/s

        Always returns a MotionHandle.
        """
        if not waypoints:
            raise ValueError("waypoints must not be empty")

        path_mm = [(float(x) * self._unit.value, float(y) * self._unit.value) for x, y in waypoints]
        vel_mm       = float(velocity)   * self._unit.value
        lookahead_mm = float(lookahead)  * self._unit.value
        tolerance_mm = float(tolerance)  * self._unit.value
        advance_radius_mm = (
            tolerance_mm if advance_radius is None
            else float(advance_radius) * self._unit.value
        )
        max_angular = float(max_angular_rad_s)

        def target():
            self._nav_follow_purepursuit_path(
                path_mm, vel_mm, lookahead_mm, advance_radius_mm, tolerance_mm, max_angular
            )

        return self._start_nav(target, blocking, timeout)

    def apf_follow_path(
        self,
        waypoints: list[tuple[float, float]],
        velocity: float,
        lookahead: float,
        tolerance: float,
        repulsion_range: float,
        blocking: bool = True,
        max_angular_rad_s: float = 1.0,
        repulsion_gain: float = 500.0,
        timeout: float = None,
        *,
        advance_radius: float | None = None,
    ) -> MotionHandle:
        """
        Follow an ordered waypoint path with artificial potential fields.

        Obstacles come from set_obstacles() and/or set_obstacle_provider().
        """
        if not waypoints:
            raise ValueError("waypoints must not be empty")

        path_mm = [(float(x) * self._unit.value, float(y) * self._unit.value) for x, y in waypoints]
        vel_mm            = float(velocity)        * self._unit.value
        lookahead_mm      = float(lookahead)       * self._unit.value
        tolerance_mm      = float(tolerance)       * self._unit.value
        advance_radius_mm = (
            tolerance_mm if advance_radius is None
            else float(advance_radius) * self._unit.value
        )
        repulsion_range_mm = float(repulsion_range) * self._unit.value
        max_angular        = float(max_angular_rad_s)
        repulsion_gain     = float(repulsion_gain)

        def target():
            self._nav_follow_apf_path(
                path_mm, vel_mm, lookahead_mm, advance_radius_mm, tolerance_mm,
                repulsion_range_mm, max_angular, repulsion_gain,
            )

        return self._start_nav(target, blocking, timeout)

    def is_moving(self) -> bool:
        """True if a navigation command is active."""
        return self._nav_thread is not None and self._nav_thread.is_alive()

    def cancel_motion(self) -> None:
        """Abort any active navigation command. The robot stops."""
        self._nav_cancel.set()
        if self._nav_thread is not None:
            self._nav_thread.join(timeout=1.0)
            if self._nav_thread.is_alive():
                return
        self._nav_thread = None
        self._nav_cancel.clear()

    # =========================================================================
    # Trajectory
    # =========================================================================

    def save_trajectory_image(self, path: str = "trajectory.png") -> None:
        """Save a PNG comparing raw odometry vs fused trajectory. Requires matplotlib."""
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self._node.get_logger().error("matplotlib not installed; cannot save trajectory image")
            return

        odom  = list(self._odom_traj)
        fused = list(self._fused_traj)
        if not odom:
            self._node.get_logger().warn("No trajectory data to save")
            return

        fig, ax = plt.subplots(figsize=(8, 8))
        ox, oy = zip(*odom)
        ax.plot(ox, oy, label="odometry (raw)", color="steelblue", linewidth=1.5)
        if fused:
            fx, fy = zip(*fused)
            ax.plot(fx, fy, label="fused (GPS+odom)", color="darkorange", linewidth=1.5, linestyle="--")
        ax.scatter([ox[0]], [oy[0]], color="green", zorder=5, label="start")
        ax.scatter([ox[-1]], [oy[-1]], color="red",   zorder=5, label="end")
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_title("Trajectory: raw odometry vs fused")
        ax.legend()
        ax.set_aspect("equal")
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
        self._node.get_logger().info(f"Trajectory image saved to {path}")

    # =========================================================================
    # Units
    # =========================================================================

    def set_unit(self, unit: Unit) -> None:
        """Change the unit system. Does not affect already-sent commands."""
        self._unit = unit

    def get_unit(self) -> Unit:
        return self._unit

    # =========================================================================
    # Internal — navigation thread management
    # =========================================================================

    def _start_nav(self, target_fn, blocking: bool, timeout: float) -> MotionHandle:
        """Start a new navigation thread. Only one base-motion command may run at a time."""
        if self.is_moving():
            raise RuntimeError("Another motion is still running")

        self._nav_done.clear()
        self._nav_cancel.clear()

        def runner() -> None:
            try:
                target_fn()
            except Exception as exc:
                self._node.get_logger().error(f"motion thread failed: {exc}")
                try:
                    self.stop()
                except Exception:
                    pass
            finally:
                self._nav_done.set()

        self._nav_thread = threading.Thread(target=runner, daemon=True)
        self._nav_thread.start()
        handle = MotionHandle(self._nav_done, self._nav_cancel)
        if blocking:
            handle.wait(timeout=timeout)
        return handle

    def _move_along_heading(
        self,
        signed_distance: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ) -> MotionHandle:
        cur_x, cur_y, cur_theta_deg = self.get_pose()
        cur_theta_rad = math.radians(cur_theta_deg)
        dx = math.cos(cur_theta_rad) * signed_distance
        dy = math.sin(cur_theta_rad) * signed_distance
        return self.move_to(cur_x + dx, cur_y + dy, velocity,
                            tolerance=tolerance, blocking=blocking, timeout=timeout)

    def _nav_follow_purepursuit_path(
        self,
        waypoints_mm: list[tuple[float, float]],
        max_vel_mm: float,
        lookahead_mm: float,
        advance_radius_mm: float,
        tolerance_mm: float,
        max_angular_rad_s: float = 1.0,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Navigation thread body: pure-pursuit to an ordered list of waypoints (mm)."""
        from robot.path_planner import PurePursuitPlanner
        planner = PurePursuitPlanner(
            lookahead_dist=lookahead_mm,
            max_angular=max_angular_rad_s,
            goal_tolerance=tolerance_mm,
        )
        self._nav_follow_path(
            waypoints_mm, planner, max_vel_mm, advance_radius_mm, tolerance_mm,
            update_hz=update_hz,
        )

    def _nav_follow_apf_path(
        self,
        waypoints_mm: list[tuple[float, float]],
        max_vel_mm: float,
        lookahead_mm: float,
        advance_radius_mm: float,
        tolerance_mm: float,
        repulsion_range_mm: float,
        max_angular_rad_s: float,
        repulsion_gain: float,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Navigation thread body: APF path following with robot-frame obstacles."""
        from robot.path_planner import APFPlanner
        planner = APFPlanner(
            lookahead_dist=lookahead_mm,
            max_linear=max_vel_mm,
            max_angular=max_angular_rad_s,
            repulsion_gain=repulsion_gain,
            repulsion_range=repulsion_range_mm,
            goal_tolerance=tolerance_mm,
            obstacle_provider=self._get_obstacles_mm,
        )
        self._nav_follow_path(
            waypoints_mm, planner, max_vel_mm, advance_radius_mm, tolerance_mm,
            update_hz=update_hz,
        )

    def _nav_follow_path(
        self,
        waypoints_mm: list[tuple[float, float]],
        planner,
        max_vel_mm: float,
        advance_radius_mm: float,
        tolerance_mm: float,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Shared path-following loop for pure pursuit and APF planners."""
        remaining_path = list(waypoints_mm)
        dt = 1.0 / update_hz

        while not self._nav_cancel.is_set():
            x_mm, y_mm, theta_rad = self._get_pose_mm()
            remaining_path = self._advance_remaining_path(
                remaining_path, x_mm, y_mm, advance_radius_mm
            )

            goal_x_mm, goal_y_mm = remaining_path[0]
            if len(remaining_path) == 1 and _dist2d(x_mm, y_mm, goal_x_mm, goal_y_mm) <= tolerance_mm:
                self.stop()
                return

            linear_mm, angular_rad_s = planner.compute_velocity(
                (x_mm, y_mm, theta_rad), remaining_path, max_vel_mm
            )
            self._send_body_velocity_mm(linear_mm, angular_rad_s)
            if not self._sleep_with_cancel(dt):
                break

        self.stop()

    @staticmethod
    def _advance_remaining_path(
        remaining_path: list[tuple[float, float]],
        x_mm: float,
        y_mm: float,
        advance_radius_mm: float,
    ) -> list[tuple[float, float]]:
        """Drop intermediate waypoints once within advance_radius; never drop the last."""
        while len(remaining_path) > 1:
            next_x_mm, next_y_mm = remaining_path[0]
            if _dist2d(x_mm, y_mm, next_x_mm, next_y_mm) > advance_radius_mm:
                break
            remaining_path.pop(0)
        return remaining_path

    def _turn_to_heading(
        self,
        target_rad: float,
        tolerance_rad: float,
        max_angular_rad: float = 2.0,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Navigation thread body: rotate to target_rad in place."""
        dt = 1.0 / update_hz
        while not self._nav_cancel.is_set():
            _, _, theta_rad = self._get_pose_mm()
            error = _wrap_angle(target_rad - theta_rad)
            if abs(error) < tolerance_rad:
                self.stop()
                return
            angular = max(-max_angular_rad, min(max_angular_rad, error * 3.0))
            self._send_body_velocity_mm(0.0, angular)
            if not self._sleep_with_cancel(dt):
                break
        self.stop()

    def _sleep_with_cancel(self, seconds: float) -> bool:
        """Wait for up to seconds, waking early if motion is cancelled.
        Returns True on full interval, False on cancel."""
        return not self._nav_cancel.wait(timeout=seconds)

    # =========================================================================
    # Internal — low-level helpers
    # =========================================================================

    def _get_pose_mm(self) -> tuple[float, float, float]:
        """Return fused (x_mm, y_mm, theta_rad) without unit conversion."""
        with self._lock:
            return (self._fused_x_mm, self._fused_y_mm, self._fused_theta)

    def _get_obstacles_mm(self) -> list[tuple[float, float]]:
        """Return cached and provider-supplied APF obstacles in robot-frame millimeters."""
        with self._lock:
            cached = list(self._obstacles_mm)
            provider = self._obstacle_provider

        dynamic: list[tuple[float, float]] = []
        if provider is not None:
            try:
                provided = provider() or []
                dynamic = [
                    (
                        self._require_finite_float("obstacle_x_mm", obs_x),
                        self._require_finite_float("obstacle_y_mm", obs_y),
                    )
                    for obs_x, obs_y in provided
                ]
            except Exception as exc:
                self._node.get_logger().error(f"obstacle provider failed: {exc}")

        return cached + dynamic

    def _apply_odom_param_snapshot(
        self,
        wheel_diameter_mm: float,
        wheel_base_mm: float,
        initial_theta_deg: float,
        left_motor_number: int,
        left_motor_dir_inverted: bool,
        right_motor_number: int,
        right_motor_dir_inverted: bool,
    ) -> None:
        with self._lock:
            self._wheel_diameter = self._require_positive_float("wheel_diameter_mm", wheel_diameter_mm)
            self._wheel_base = self._require_positive_float("wheel_base_mm", wheel_base_mm)
            self._initial_theta_deg = self._require_finite_float("initial_theta_deg", initial_theta_deg)
            self._left_wheel_motor = self._require_id("left_motor_number", left_motor_number, 1, 4)
            self._right_wheel_motor = self._require_id("right_motor_number", right_motor_number, 1, 4)
            if self._left_wheel_motor == self._right_wheel_motor:
                raise ValueError("left and right odometry motors must be different")
            self._left_wheel_dir_inverted  = bool(left_motor_dir_inverted)
            self._right_wheel_dir_inverted = bool(right_motor_dir_inverted)
            self._ticks_per_mm = self._encoder_ppr / (math.pi * self._wheel_diameter)

    def _update_odometry_params(
        self,
        *,
        wheel_diameter_mm: float | None = None,
        wheel_base_mm: float | None = None,
        initial_theta_deg: float | None = None,
        left_wheel_motor: int | None = None,
        left_wheel_dir_inverted: bool | None = None,
        right_wheel_motor: int | None = None,
        right_wheel_dir_inverted: bool | None = None,
        timeout: float = 0.0,
    ) -> bool:
        with self._lock:
            next_wheel_diameter = self._wheel_diameter if wheel_diameter_mm is None else \
                self._require_positive_float("wheel_diameter_mm", wheel_diameter_mm)
            next_wheel_base = self._wheel_base if wheel_base_mm is None else \
                self._require_positive_float("wheel_base_mm", wheel_base_mm)
            next_initial_theta = self._initial_theta_deg if initial_theta_deg is None else \
                self._require_finite_float("initial_theta_deg", initial_theta_deg)
            next_left_motor = self._left_wheel_motor if left_wheel_motor is None else \
                self._require_id("left_motor_id", left_wheel_motor, 1, 4)
            next_right_motor = self._right_wheel_motor if right_wheel_motor is None else \
                self._require_id("right_motor_id", right_wheel_motor, 1, 4)
            if next_left_motor == next_right_motor:
                raise ValueError("left and right odometry motors must be different")
            next_left_inverted  = self._left_wheel_dir_inverted  if left_wheel_dir_inverted  is None else bool(left_wheel_dir_inverted)
            next_right_inverted = self._right_wheel_dir_inverted if right_wheel_dir_inverted is None else bool(right_wheel_dir_inverted)

        self._apply_odom_param_snapshot(
            next_wheel_diameter, next_wheel_base, next_initial_theta,
            next_left_motor, next_left_inverted, next_right_motor, next_right_inverted,
        )

        snapshot = (
            next_wheel_diameter, next_wheel_base, next_initial_theta,
            next_left_motor, next_left_inverted, next_right_motor, next_right_inverted,
        )

        self._odom_user_configured = True
        self._odom_confirm_event.clear()
        self._publish_odom_params(snapshot)
        if timeout <= 0.0:
            return True

        self.request_odometry_parameters()
        confirmed = self._odom_confirm_event.wait(timeout=timeout)
        if not confirmed:
            self._node.get_logger().warning(
                f"[robot] set_odometry_parameters: firmware did not confirm within "
                f"{timeout}s — bridge may not be connected yet or firmware rejected params"
            )
        return confirmed

    def _publish_odom_params(
        self,
        snapshot: tuple[float, float, float, int, bool, int, bool],
    ) -> None:
        msg = SysOdomParamSet()
        (
            msg.wheel_diameter_mm,
            msg.wheel_base_mm,
            msg.initial_theta_deg,
            msg.left_motor_number,
            msg.left_motor_dir_inverted,
            msg.right_motor_number,
            msg.right_motor_dir_inverted,
        ) = snapshot
        self._odom_param_pub.publish(msg)

    def _send_body_velocity_mm(self, linear_mm_s: float, angular_rad_s: float) -> None:
        """Diff-drive mixing and publish. All values in mm/s and rad/s."""
        with self._lock:
            half_wb    = self._wheel_base / 2.0
            left_motor = self._left_wheel_motor
            right_motor = self._right_wheel_motor
            left_dir_inverted  = self._left_wheel_dir_inverted
            right_dir_inverted = self._right_wheel_dir_inverted
        if linear_mm_s or angular_rad_s:
            self._ensure_drive_motors_enabled()
        left_velocity  = linear_mm_s - angular_rad_s * half_wb
        right_velocity = linear_mm_s + angular_rad_s * half_wb
        if left_dir_inverted:
            left_velocity  = -left_velocity
        if right_dir_inverted:
            right_velocity = -right_velocity
        self._send_motor_velocity_mm(left_motor,  left_velocity)
        self._send_motor_velocity_mm(right_motor, right_velocity)
