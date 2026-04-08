"""
sensor_fusion.py — pluggable heading-fusion strategies
======================================================
All strategies share the same interface::

    fused_theta_rad = strategy.update(odom_theta, mag_heading, linear_vel, angular_vel)

Arguments
---------
odom_theta   : heading from wheel odometry (radians)
mag_heading  : absolute heading from magnetometer (radians), or None when
               the magnetometer is not yet calibrated
linear_vel   : magnitude of linear body velocity (mm/s)
angular_vel  : signed angular velocity from odometry (rad/s)

Usage in robot.py / main.py
----------------------------
    from robot.sensor_fusion import AdaptiveComplementaryFilter
    robot.set_fusion_strategy(AdaptiveComplementaryFilter())
"""

from __future__ import annotations

import math


def _wrap(angle: float) -> float:
    """Wrap angle difference to [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# =============================================================================
# Abstract base
# =============================================================================

class SensorFusion:
    """Abstract base for heading fusion strategies."""

    def update(
        self,
        odom_theta: float,
        mag_heading: float | None,
        linear_vel: float,
        angular_vel: float,
    ) -> float:
        """Return fused heading estimate in radians."""
        raise NotImplementedError


# =============================================================================
# Complementary filter (fixed alpha)
# =============================================================================

class ComplementaryFilter(SensorFusion):
    """
    Fixed-weight complementary filter.

    Blends the magnetometer heading (absolute but noisy) with the odometry
    heading (smooth but drifts) using a constant weight::

        fused = odom_theta + alpha * wrap(mag_heading - odom_theta)

    Parameters
    ----------
    alpha : float, 0.0–1.0
        Magnetometer weight.  0 = pure odometry, 1 = pure magnetometer.
        Default 0.02 gives gentle long-term drift correction.
    """

    def __init__(self, alpha: float = 0.02) -> None:
        self.alpha = max(0.0, min(1.0, float(alpha)))

    def update(
        self,
        odom_theta: float,
        mag_heading: float | None,
        linear_vel: float,
        angular_vel: float,
    ) -> float:
        if mag_heading is None:
            return odom_theta
        return odom_theta + self.alpha * _wrap(mag_heading - odom_theta)


# =============================================================================
# Adaptive complementary filter (velocity-dependent alpha)
# =============================================================================

class AdaptiveComplementaryFilter(SensorFusion):
    """
    Velocity-adaptive complementary filter.

    The magnetometer weight decreases as the robot moves faster.  When the
    robot is nearly stationary, the magnetometer is very reliable so the
    weight rises toward ``alpha_max``.  At speed, odometry is more
    trustworthy short-term, so the weight drops toward ``alpha_min``::

        motion = |v| / linear_scale  +  |ω| / angular_scale
        alpha  = alpha_min + (alpha_max - alpha_min) * exp(-motion)

    Intuition
    ---------
    * Low linear *and* angular velocity  → alpha ≈ alpha_max  (trust mag)
    * High linear *or* angular velocity  → alpha ≈ alpha_min  (trust odom)
    * linear_scale / angular_scale set the velocity at which the extra mag
      weight decays to exp(−1) ≈ 37 % of its maximum.

    Parameters
    ----------
    alpha_min     : weight floor at high speed (default 0.005)
    alpha_max     : weight ceiling at rest (default 0.10)
    linear_scale  : linear speed (mm/s) where extra mag weight is ≈ 37 %
                    (default 50.0)
    angular_scale : angular speed (rad/s) where extra mag weight is ≈ 37 %
                    (default 0.3)
    """

    def __init__(
        self,
        alpha_min: float = 0.005,
        alpha_max: float = 0.10,
        linear_scale: float = 50.0,
        angular_scale: float = 0.3,
    ) -> None:
        self.alpha_min = float(alpha_min)
        self.alpha_max = float(alpha_max)
        self.linear_scale = max(float(linear_scale), 1e-6)
        self.angular_scale = max(float(angular_scale), 1e-6)

    def effective_alpha(self, linear_vel: float, angular_vel: float) -> float:
        """Return the alpha that will be applied at the given velocities."""
        motion = abs(linear_vel) / self.linear_scale + abs(angular_vel) / self.angular_scale
        return self.alpha_min + (self.alpha_max - self.alpha_min) * math.exp(-motion)

    def update(
        self,
        odom_theta: float,
        mag_heading: float | None,
        linear_vel: float,
        angular_vel: float,
    ) -> float:
        if mag_heading is None:
            return odom_theta
        alpha = self.effective_alpha(linear_vel, angular_vel)
        return odom_theta + alpha * _wrap(mag_heading - odom_theta)


# =============================================================================
# 1-D Kalman filter on heading
# =============================================================================

class HeadingKalmanFilter(SensorFusion):
    """
    1-D Kalman filter for robot heading.

    The odometry heading acts as the process prediction.  The magnetometer
    provides noisy absolute measurements to correct accumulated drift.

    Equations
    ---------
    Predict  :  θ̂⁻  = odom_theta
                P⁻   = P + Q

    Update   :  K    = P⁻ / (P⁻ + R)
                θ̂    = θ̂⁻ + K · wrap(mag_heading − θ̂⁻)
                P    = (1 − K) · P⁻

    When the magnetometer is not yet calibrated only the predict step runs,
    so the estimate tracks odometry exactly until the first mag fix.

    Parameters
    ----------
    process_noise     : Q — heading variance added each update (rad²/step).
                        Larger = trust odometry less and correct drift
                        faster, but the estimate becomes noisier.
                        Default 1e-4.
    measurement_noise : R — magnetometer heading variance (rad²).
                        Larger = trust mag less, smoother but slower to
                        correct drift.  Default 0.05.
    initial_variance  : P₀ — starting error covariance.  High default (1.0)
                        means the first magnetometer fix is trusted strongly.
    """

    def __init__(
        self,
        process_noise: float = 1e-4,
        measurement_noise: float = 0.05,
        initial_variance: float = 1.0,
    ) -> None:
        self.Q = float(process_noise)
        self.R = float(measurement_noise)
        self._P = float(initial_variance)
        self._theta_est: float | None = None  # initialized on first update

    @property
    def variance(self) -> float:
        """Current error covariance P (rad²). Converges toward Q*R/(Q+R) in steady state."""
        return self._P

    def update(
        self,
        odom_theta: float,
        mag_heading: float | None,
        linear_vel: float,
        angular_vel: float,
    ) -> float:
        if self._theta_est is None:
            self._theta_est = odom_theta

        # Predict: odometry is the process model
        theta_pred = odom_theta
        P_pred = self._P + self.Q

        if mag_heading is None:
            self._theta_est = theta_pred
            self._P = P_pred
            return theta_pred

        # Update: fuse with magnetometer measurement
        K = P_pred / (P_pred + self.R)
        self._theta_est = theta_pred + K * _wrap(mag_heading - theta_pred)
        self._P = (1.0 - K) * P_pred
        return self._theta_est
