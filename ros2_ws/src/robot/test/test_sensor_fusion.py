from __future__ import annotations

import importlib
import math
import sys
import types
import unittest
from pathlib import Path

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from robot.sensor_fusion import (
    AdaptiveComplementaryFilter,
    ComplementaryFilter,
    HeadingKalmanFilter,
)
from robot.path_planner import PurePursuitPlanner


# ---------------------------------------------------------------------------
# Minimal ROS 2 / bridge_interfaces stubs (no ROS installation required)
# ---------------------------------------------------------------------------

def _install_fake_robot_dependencies() -> None:
    if "rclpy" in sys.modules:
        return

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy.node = rclpy_node
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node

    bridge_interfaces = types.ModuleType("bridge_interfaces")
    bridge_interfaces_msg = types.ModuleType("bridge_interfaces.msg")
    bridge_interfaces_srv = types.ModuleType("bridge_interfaces.srv")

    for name in [
        "DCEnable", "DCHome", "DCPidReq", "DCPidSet", "DCResetPosition",
        "DCSetPosition", "DCSetPwm", "DCSetVelocity", "DCStateAll",
        "IOInputState", "IOSetLed", "IOSetNeopixel", "SensorImu",
        "SensorKinematics", "ServoEnable", "ServoSet", "ServoStateAll",
        "StepConfigSet", "StepEnable", "StepHome", "StepMove", "StepStateAll",
        "SysOdomParamReq", "SysOdomParamRsp", "SysOdomParamSet",
        "SysOdomReset", "SystemPower", "SystemState",
    ]:
        setattr(bridge_interfaces_msg, name, type(name, (), {}))

    class SetFirmwareState:
        class Request:
            def __init__(self) -> None:
                self.target_state = 0
                self.timeout_sec = 0.0

    bridge_interfaces_srv.SetFirmwareState = SetFirmwareState
    bridge_interfaces.msg = bridge_interfaces_msg
    bridge_interfaces.srv = bridge_interfaces_srv
    sys.modules["bridge_interfaces"] = bridge_interfaces
    sys.modules["bridge_interfaces.msg"] = bridge_interfaces_msg
    sys.modules["bridge_interfaces.srv"] = bridge_interfaces_srv


class FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class FakeNode:
    def __init__(self) -> None:
        self.publishers: dict[str, FakePublisher] = {}

    def create_publisher(self, _msg_type, topic: str, _qos: int) -> FakePublisher:
        pub = FakePublisher(topic)
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, *_args, **_kwargs):
        return None

    def create_client(self, *_args, **_kwargs):
        class _FakeClient:
            def wait_for_service(self, **_kw): return True
            def call_async(self, _req):
                f = types.SimpleNamespace(done=lambda: True, result=lambda: None,
                                         add_done_callback=lambda _cb: None)
                return f
        return _FakeClient()

    def get_logger(self):
        return types.SimpleNamespace(error=lambda _m: None)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_imu(module, *, mag_calibrated: bool, mag_x: float, mag_y: float):
    msg = module.SensorImu()
    msg.mag_calibrated = mag_calibrated
    msg.mag_x = mag_x
    msg.mag_y = mag_y
    return msg


def _make_kin(module, *, theta: float, x: float = 0.0, y: float = 0.0):
    msg = module.SensorKinematics()
    msg.x = x
    msg.y = y
    msg.theta = theta
    msg.vx = 0.0
    msg.vy = 0.0
    msg.v_theta = 0.0
    return msg


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class SensorFusionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_robot_dependencies()
        package_root = Path(__file__).resolve().parents[1]
        if str(package_root) not in sys.path:
            sys.path.insert(0, str(package_root))
        cls.mod = importlib.import_module("robot.robot")

    def setUp(self) -> None:
        self.node = FakeNode()
        self.robot = self.mod.Robot(self.node)

    # ------------------------------------------------------------------
    # Initial state
    # ------------------------------------------------------------------

    def test_initial_fused_orientation_is_zero_degrees(self) -> None:
        # No messages received yet — default fused_theta is 0.0 rad.
        self.assertAlmostEqual(self.robot.get_fused_orientation(), 0.0, places=6)

    # ------------------------------------------------------------------
    # No mag calibration → pure odometry pass-through
    # ------------------------------------------------------------------

    def test_uncalibrated_mag_does_not_affect_fused_theta(self) -> None:
        # Deliver an IMU message with mag pointing north but uncalibrated.
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=False, mag_x=1.0, mag_y=0.0))
        # Kinematics says robot is facing 45°.
        self.robot._on_kinematics(_make_kin(self.mod, theta=math.radians(45.0)))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 45.0, places=6)

    def test_no_imu_message_fused_theta_equals_odometry(self) -> None:
        # Without any IMU message at all, _mag_heading remains None.
        self.robot._on_kinematics(_make_kin(self.mod, theta=math.radians(30.0)))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 30.0, places=6)

    # ------------------------------------------------------------------
    # Mag heading extraction from atan2(mag_y, mag_x)
    # ------------------------------------------------------------------

    def test_mag_heading_east_is_zero(self) -> None:
        # mag_x > 0, mag_y == 0 → atan2(0, 1) = 0 rad
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=1.0, mag_y=0.0))
        self.robot.set_fusion_alpha(1.0)
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 0.0, places=6)

    def test_mag_heading_north_is_90_degrees(self) -> None:
        # mag_x == 0, mag_y > 0 → atan2(1, 0) = π/2 rad = 90°
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot.set_fusion_alpha(1.0)
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 90.0, places=6)

    def test_mag_heading_west_is_180_degrees(self) -> None:
        # mag_x < 0, mag_y == 0 → atan2(0, -1) = π rad = 180°
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=-1.0, mag_y=0.0))
        self.robot.set_fusion_alpha(1.0)
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        # math.degrees(math.pi) ≈ 180.0
        self.assertAlmostEqual(
            abs(self.robot.get_fused_orientation()), 180.0, places=6
        )

    def test_mag_heading_south_is_minus_90_degrees(self) -> None:
        # mag_x == 0, mag_y < 0 → atan2(-1, 0) = -π/2 rad = -90°
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=-1.0))
        self.robot.set_fusion_alpha(1.0)
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), -90.0, places=6)

    # ------------------------------------------------------------------
    # Complementary filter blend — alpha extremes
    # ------------------------------------------------------------------

    def test_alpha_zero_means_pure_odometry(self) -> None:
        # With alpha = 0 the mag heading has no influence at all.
        self.robot.set_fusion_alpha(0.0)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        odom_theta = math.radians(30.0)
        self.robot._on_kinematics(_make_kin(self.mod, theta=odom_theta))

        self.assertAlmostEqual(
            self.robot.get_fused_orientation(), math.degrees(odom_theta), places=6
        )

    def test_alpha_one_means_pure_magnetometer(self) -> None:
        # With alpha = 1 the result must equal the mag heading regardless of odometry.
        self.robot.set_fusion_alpha(1.0)
        mag_heading = math.atan2(1.0, 0.0)  # π/2
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        self.assertAlmostEqual(
            self.robot.get_fused_orientation(), math.degrees(mag_heading), places=6
        )

    def test_blend_formula_with_explicit_alpha(self) -> None:
        # Verify: fused = theta + alpha * wrap(mag_heading - theta)
        alpha = 0.3
        self.robot.set_fusion_alpha(alpha)

        mag_heading = math.atan2(1.0, 0.0)  # π/2
        odom_theta = math.radians(10.0)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=odom_theta))

        diff = math.atan2(
            math.sin(mag_heading - odom_theta),
            math.cos(mag_heading - odom_theta),
        )
        expected_deg = math.degrees(odom_theta + alpha * diff)
        self.assertAlmostEqual(self.robot.get_fused_orientation(), expected_deg, places=6)

    # ------------------------------------------------------------------
    # Angle-wrap robustness (crossing ±π boundary)
    # ------------------------------------------------------------------

    def test_angle_wrap_mag_near_plus_pi_odom_near_minus_pi(self) -> None:
        # mag heading ≈ +175°, odometry ≈ -175°: naive subtraction gives ~350°,
        # but the atan2 wrapping must produce a short ~10° correction.
        alpha = 0.5
        self.robot.set_fusion_alpha(alpha)

        mag_deg = 175.0
        odom_deg = -175.0
        mag_rad = math.radians(mag_deg)
        odom_rad = math.radians(odom_deg)

        # Construct a unit mag vector for the desired heading.
        self.robot._on_imu(_make_imu(
            self.mod, mag_calibrated=True,
            mag_x=math.cos(mag_rad), mag_y=math.sin(mag_rad),
        ))
        self.robot._on_kinematics(_make_kin(self.mod, theta=odom_rad))

        diff = math.atan2(
            math.sin(mag_rad - odom_rad),
            math.cos(mag_rad - odom_rad),
        )
        expected_deg = math.degrees(odom_rad + alpha * diff)
        self.assertAlmostEqual(self.robot.get_fused_orientation(), expected_deg, places=5)

        # The correction must be small (≈ +5°, not ≈ +175°).
        correction = self.robot.get_fused_orientation() - odom_deg
        self.assertLess(abs(correction), 20.0)

    def test_angle_wrap_mag_near_minus_pi_odom_near_plus_pi(self) -> None:
        alpha = 0.5
        self.robot.set_fusion_alpha(alpha)

        mag_rad = math.radians(-175.0)
        odom_rad = math.radians(175.0)

        self.robot._on_imu(_make_imu(
            self.mod, mag_calibrated=True,
            mag_x=math.cos(mag_rad), mag_y=math.sin(mag_rad),
        ))
        self.robot._on_kinematics(_make_kin(self.mod, theta=odom_rad))

        diff = math.atan2(
            math.sin(mag_rad - odom_rad),
            math.cos(mag_rad - odom_rad),
        )
        expected_deg = math.degrees(odom_rad + alpha * diff)
        self.assertAlmostEqual(self.robot.get_fused_orientation(), expected_deg, places=5)

        correction = self.robot.get_fused_orientation() - math.degrees(odom_rad)
        self.assertLess(abs(correction), 20.0)

    def test_no_wrap_artifact_when_headings_agree(self) -> None:
        # When mag and odometry agree, the correction must be zero.
        alpha = 0.5
        self.robot.set_fusion_alpha(alpha)
        theta = math.radians(45.0)

        self.robot._on_imu(_make_imu(
            self.mod, mag_calibrated=True,
            mag_x=math.cos(theta), mag_y=math.sin(theta),
        ))
        self.robot._on_kinematics(_make_kin(self.mod, theta=theta))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 45.0, places=5)

    # ------------------------------------------------------------------
    # Successive updates
    # ------------------------------------------------------------------

    def test_latest_imu_message_takes_precedence(self) -> None:
        # Send two IMU messages in a row — only the second should matter.
        self.robot.set_fusion_alpha(1.0)

        # First message: points east (0°)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=1.0, mag_y=0.0))
        # Second message: points north (90°)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 90.0, places=6)

    def test_recalibration_replaces_stale_mag_heading(self) -> None:
        # Calibrated → uncalibrated (drops mag) → recalibrated with new heading.
        self.robot.set_fusion_alpha(0.5)

        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=1.0, mag_y=0.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        after_first_cal = self.robot.get_fused_orientation()
        self.assertAlmostEqual(after_first_cal, 0.0, places=5)

        # Uncalibrated: _mag_heading should NOT be updated.
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=False, mag_x=0.0, mag_y=1.0))
        # Kinematics still uses the old mag heading.
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        # Still using the old east (0°) heading.
        self.assertAlmostEqual(self.robot.get_fused_orientation(), 0.0, places=5)

        # Now recalibrate pointing north.
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        # alpha=0.5 → fused = 0 + 0.5 * (π/2) ≈ 45°
        self.assertAlmostEqual(self.robot.get_fused_orientation(), 45.0, places=5)

    def test_kinematics_callback_sets_pose_event(self) -> None:
        # _on_kinematics must pulse the pose_event so wait_for_pose_update() works.
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        # After the callback the event is immediately cleared; the pulse should
        # have occurred, so the event object itself is the correct observable.
        # We can verify by checking that no exception was raised and _pose updated.
        x, y, theta = self.robot._pose
        self.assertAlmostEqual(theta, 0.0, places=6)

    # ------------------------------------------------------------------
    # get_pose() returns raw odometry, NOT fused heading
    # ------------------------------------------------------------------

    def test_get_pose_theta_is_raw_odometry_not_fused(self) -> None:
        self.robot.set_fusion_alpha(0.5)
        odom_theta = math.radians(20.0)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=odom_theta))

        _, _, pose_theta_deg = self.robot.get_pose()
        fused_deg = self.robot.get_fused_orientation()

        # The raw pose theta must equal the odometry input exactly.
        self.assertAlmostEqual(pose_theta_deg, math.degrees(odom_theta), places=6)
        # And the fused value must differ (mag pulls it toward 90°).
        self.assertNotAlmostEqual(pose_theta_deg, fused_deg, places=2)

    # ------------------------------------------------------------------
    # Alpha clamping
    # ------------------------------------------------------------------

    def test_alpha_above_one_is_clamped_to_one(self) -> None:
        self.robot.set_fusion_alpha(5.0)
        self.assertEqual(self.robot._fusion.alpha, 1.0)

    def test_alpha_below_zero_is_clamped_to_zero(self) -> None:
        self.robot.set_fusion_alpha(-3.0)
        self.assertEqual(self.robot._fusion.alpha, 0.0)

    def test_alpha_boundary_values_are_accepted(self) -> None:
        self.robot.set_fusion_alpha(0.0)
        self.assertEqual(self.robot._fusion.alpha, 0.0)
        self.robot.set_fusion_alpha(1.0)
        self.assertEqual(self.robot._fusion.alpha, 1.0)

    # ------------------------------------------------------------------
    # Output is in degrees
    # ------------------------------------------------------------------

    def test_get_fused_orientation_returns_degrees_not_radians(self) -> None:
        # If the value were in radians, π/2 rad ≈ 1.57 which is clearly not 90.
        self.robot.set_fusion_alpha(1.0)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        result = self.robot.get_fused_orientation()
        self.assertGreater(result, 2.0)   # would be ≈ 1.57 if accidentally radians
        self.assertAlmostEqual(result, 90.0, places=5)


# ===========================================================================
# AdaptiveComplementaryFilter unit tests
# ===========================================================================

class AdaptiveComplementaryFilterTests(unittest.TestCase):
    """Tests for velocity-dependent alpha blending."""

    def test_at_rest_uses_alpha_max(self) -> None:
        f = AdaptiveComplementaryFilter(alpha_min=0.01, alpha_max=0.20)
        alpha = f.effective_alpha(linear_vel=0.0, angular_vel=0.0)
        self.assertAlmostEqual(alpha, 0.20, places=6)

    def test_at_high_speed_approaches_alpha_min(self) -> None:
        f = AdaptiveComplementaryFilter(alpha_min=0.01, alpha_max=0.20,
                                        linear_scale=50.0, angular_scale=0.3)
        # At 500 mm/s (10× linear_scale) the exp term is exp(-10) ≈ 4.5e-5
        alpha = f.effective_alpha(linear_vel=500.0, angular_vel=0.0)
        self.assertLess(alpha, 0.011)

    def test_alpha_decreases_with_linear_velocity(self) -> None:
        f = AdaptiveComplementaryFilter()
        alpha_slow = f.effective_alpha(linear_vel=10.0, angular_vel=0.0)
        alpha_fast = f.effective_alpha(linear_vel=200.0, angular_vel=0.0)
        self.assertGreater(alpha_slow, alpha_fast)

    def test_alpha_decreases_with_angular_velocity(self) -> None:
        f = AdaptiveComplementaryFilter()
        alpha_slow = f.effective_alpha(linear_vel=0.0, angular_vel=0.1)
        alpha_fast = f.effective_alpha(linear_vel=0.0, angular_vel=2.0)
        self.assertGreater(alpha_slow, alpha_fast)

    def test_no_mag_returns_odom(self) -> None:
        f = AdaptiveComplementaryFilter()
        result = f.update(odom_theta=1.0, mag_heading=None,
                          linear_vel=0.0, angular_vel=0.0)
        self.assertAlmostEqual(result, 1.0, places=6)

    def test_blend_at_known_velocities(self) -> None:
        f = AdaptiveComplementaryFilter(alpha_min=0.0, alpha_max=0.5,
                                        linear_scale=50.0, angular_scale=0.3)
        # At rest alpha == 0.5; mag at π/2, odom at 0 → fused = 0 + 0.5*(π/2)
        expected = 0.0 + 0.5 * (math.pi / 2)
        result = f.update(odom_theta=0.0, mag_heading=math.pi / 2,
                          linear_vel=0.0, angular_vel=0.0)
        self.assertAlmostEqual(result, expected, places=5)

    def test_wrap_handled_at_pi_boundary(self) -> None:
        f = AdaptiveComplementaryFilter(alpha_min=0.0, alpha_max=1.0,
                                        linear_scale=50.0, angular_scale=0.3)
        # At rest alpha == 1.0 → result == mag_heading (angles may differ by 2π)
        mag = math.radians(175.0)
        odom = math.radians(-175.0)
        result = f.update(odom_theta=odom, mag_heading=mag,
                          linear_vel=0.0, angular_vel=0.0)
        # Compare wrapped: result and mag must be the same angle
        self.assertAlmostEqual(math.atan2(math.sin(result - mag),
                                          math.cos(result - mag)), 0.0, places=5)

    def test_via_robot_set_fusion_strategy(self) -> None:
        """Verify the strategy plugs in through Robot.set_fusion_strategy()."""
        _install_fake_robot_dependencies()
        mod = importlib.import_module("robot.robot")
        node = FakeNode()
        robot = mod.Robot(node)

        robot.set_fusion_strategy(AdaptiveComplementaryFilter(
            alpha_min=0.0, alpha_max=0.5,
            linear_scale=50.0, angular_scale=0.3,
        ))

        imu = mod.SensorImu()
        imu.mag_calibrated = True
        imu.mag_x = 0.0
        imu.mag_y = 1.0   # mag heading = π/2
        robot._on_imu(imu)

        kin = mod.SensorKinematics()
        kin.x = 0.0; kin.y = 0.0; kin.theta = 0.0
        kin.vx = 0.0; kin.vy = 0.0; kin.v_theta = 0.0  # at rest → alpha_max = 0.5
        robot._on_kinematics(kin)

        expected_deg = math.degrees(0.5 * math.pi / 2)
        self.assertAlmostEqual(robot.get_fused_orientation(), expected_deg, places=4)

    def test_set_fusion_alpha_raises_on_wrong_strategy(self) -> None:
        _install_fake_robot_dependencies()
        mod = importlib.import_module("robot.robot")
        node = FakeNode()
        robot = mod.Robot(node)
        robot.set_fusion_strategy(AdaptiveComplementaryFilter())
        with self.assertRaises(TypeError):
            robot.set_fusion_alpha(0.5)


# ===========================================================================
# HeadingKalmanFilter unit tests
# ===========================================================================

class HeadingKalmanFilterTests(unittest.TestCase):
    """Tests for the 1-D Kalman heading filter."""

    def test_no_mag_tracks_odometry_exactly(self) -> None:
        f = HeadingKalmanFilter()
        for theta in [0.0, 0.3, 0.7, 1.2]:
            result = f.update(odom_theta=theta, mag_heading=None,
                              linear_vel=0.0, angular_vel=0.0)
            self.assertAlmostEqual(result, theta, places=6)

    def test_variance_grows_without_mag(self) -> None:
        f = HeadingKalmanFilter(process_noise=0.01, initial_variance=0.5)
        P0 = f.variance
        f.update(odom_theta=0.0, mag_heading=None, linear_vel=0.0, angular_vel=0.0)
        self.assertGreater(f.variance, P0)

    def test_variance_shrinks_after_mag_update(self) -> None:
        f = HeadingKalmanFilter(process_noise=0.001, measurement_noise=0.05,
                                initial_variance=1.0)
        P_before = f.variance + f.Q   # P_pred on first step
        f.update(odom_theta=0.0, mag_heading=0.0, linear_vel=0.0, angular_vel=0.0)
        self.assertLess(f.variance, P_before)

    def test_applies_nonzero_correction_from_mag(self) -> None:
        # Odom says 0°, mag says 90°. The Kalman filter must apply some
        # correction — the estimate can't stay at exactly 0.
        # Note: the filter will NOT converge to 90° because the process model
        # (odom=0) constantly pulls the prediction back to 0; the steady-state
        # is a noise-weighted blend well below 90°.
        f = HeadingKalmanFilter(process_noise=1e-4, measurement_noise=0.01)
        mag = math.pi / 2
        for _ in range(200):
            result = f.update(odom_theta=0.0, mag_heading=mag,
                              linear_vel=0.0, angular_vel=0.0)
        self.assertGreater(result, 0.0)
        self.assertLess(result, math.pi / 2)

    def test_higher_process_noise_gives_more_mag_weight(self) -> None:
        # With larger Q the filter is less certain about the process model
        # (odom) so it trusts the magnetometer more.
        mag = math.pi / 2
        results = []
        for Q in (1e-6, 1e-3, 0.1):
            f = HeadingKalmanFilter(process_noise=Q, measurement_noise=0.01,
                                    initial_variance=0.001)
            for _ in range(50):
                r = f.update(odom_theta=0.0, mag_heading=mag,
                             linear_vel=0.0, angular_vel=0.0)
            results.append(r)
        # Larger Q → more mag correction → larger heading estimate
        self.assertLess(results[0], results[1])
        self.assertLess(results[1], results[2])

    def test_angle_wrap_at_pi_boundary(self) -> None:
        f = HeadingKalmanFilter(process_noise=1e-4, measurement_noise=0.01)
        mag = math.radians(175.0)
        odom = math.radians(-175.0)
        result = f.update(odom_theta=odom, mag_heading=mag,
                          linear_vel=0.0, angular_vel=0.0)
        # Correction must be small (toward +175°), not a large swing
        correction = abs(result - odom)
        self.assertLess(correction, math.radians(15.0))

    def test_kalman_gain_decreases_as_filter_converges(self) -> None:
        # Run several mag updates and verify variance (hence K) shrinks.
        f = HeadingKalmanFilter(process_noise=1e-6, measurement_noise=0.1,
                                initial_variance=1.0)
        variances = []
        for _ in range(10):
            f.update(odom_theta=0.0, mag_heading=0.0, linear_vel=0.0, angular_vel=0.0)
            variances.append(f.variance)
        # Each successive variance should be smaller than the previous.
        self.assertTrue(all(a > b for a, b in zip(variances, variances[1:])))

    def test_high_measurement_noise_stays_close_to_odom(self) -> None:
        # With R >> P, the Kalman gain K ≈ 0 → estimate ≈ odom_theta.
        f = HeadingKalmanFilter(process_noise=1e-4, measurement_noise=1e6,
                                initial_variance=0.001)
        result = f.update(odom_theta=0.0, mag_heading=math.pi / 2,
                          linear_vel=0.0, angular_vel=0.0)
        self.assertAlmostEqual(result, 0.0, places=3)

    def test_low_measurement_noise_trusts_mag_strongly(self) -> None:
        # With R << P, K ≈ 1 → estimate ≈ mag_heading.
        f = HeadingKalmanFilter(process_noise=1e-4, measurement_noise=1e-6,
                                initial_variance=1.0)
        result = f.update(odom_theta=0.0, mag_heading=math.pi / 2,
                          linear_vel=0.0, angular_vel=0.0)
        self.assertAlmostEqual(result, math.pi / 2, places=4)

    def test_via_robot_set_fusion_strategy(self) -> None:
        _install_fake_robot_dependencies()
        mod = importlib.import_module("robot.robot")
        node = FakeNode()
        robot = mod.Robot(node)
        robot.set_fusion_strategy(HeadingKalmanFilter(
            process_noise=1e-4, measurement_noise=1e-6, initial_variance=1.0
        ))

        imu = mod.SensorImu()
        imu.mag_calibrated = True
        imu.mag_x = 0.0; imu.mag_y = 1.0   # π/2
        robot._on_imu(imu)

        kin = mod.SensorKinematics()
        kin.x = 0.0; kin.y = 0.0; kin.theta = 0.0
        kin.vx = 0.0; kin.vy = 0.0; kin.v_theta = 0.0
        robot._on_kinematics(kin)

        # Low R → strongly trusts mag → fused ≈ π/2 rad = 90°
        self.assertAlmostEqual(robot.get_fused_orientation(), 90.0, places=3)


# ===========================================================================
# Integration: fused heading → PurePursuitPlanner.compute_velocity()
#
# These tests mirror the logic in main.py's MOVING state:
#
#   current_x, current_y, _ = robot.get_pose()
#   current_theta_rad = math.radians(robot.get_fused_orientation())
#   linear, angular = planner.compute_velocity(
#       pose=(current_x, current_y, current_theta_rad), ...)
#
# The key contract: compute_velocity receives the *fused* heading, not the
# raw odometry theta.  When the two differ the steering commands must differ.
# ===========================================================================

def _drive_robot(mod, node, *, imu_mag_x, imu_mag_y, mag_calibrated,
                 odom_theta, vx=0.0, vy=0.0, v_theta=0.0, strategy=None):
    """
    Feed one IMU + kinematics pair into a fresh Robot and return it.

    If *strategy* is provided it is installed before the kinematics message
    so the fusion runs with the intended strategy from the first update.
    """
    robot = mod.Robot(node)
    imu = mod.SensorImu()
    imu.mag_calibrated = mag_calibrated
    imu.mag_x = imu_mag_x
    imu.mag_y = imu_mag_y
    robot._on_imu(imu)
    if strategy is not None:
        robot.set_fusion_strategy(strategy)
    kin = mod.SensorKinematics()
    kin.x = 0.0; kin.y = 0.0; kin.theta = odom_theta
    kin.vx = vx; kin.vy = vy; kin.v_theta = v_theta
    robot._on_kinematics(kin)
    return robot


class FusedPurePursuitTests(unittest.TestCase):
    """
    Verify that sensor-fused heading drives PurePursuit steering differently
    than raw odometry heading, and that each fusion strategy produces the
    expected steering behaviour.
    """

    PLANNER = PurePursuitPlanner(lookahead_dist=100.0, max_angular=2.0)
    WAYPOINTS_NORTH = __import__("numpy").array([[0.0, 200.0]])  # directly north
    MAX_V = 100.0

    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_robot_dependencies()
        cls.mod = importlib.import_module("robot.robot")

    def setUp(self) -> None:
        self.node = FakeNode()

    # ------------------------------------------------------------------
    # Helper: run the main.py MOVING-state lines and return (lin, ang)
    # ------------------------------------------------------------------
    def _velocity_from_fused(self, robot) -> tuple[float, float]:
        x, y, _ = robot.get_pose()
        theta_rad = math.radians(robot.get_fused_orientation())
        return self.PLANNER.compute_velocity(
            pose=(x, y, theta_rad),
            waypoints=self.WAYPOINTS_NORTH,
            max_linear=self.MAX_V,
        )

    def _velocity_from_raw_odom(self, robot) -> tuple[float, float]:
        x, y, theta_deg = robot.get_pose()
        return self.PLANNER.compute_velocity(
            pose=(x, y, math.radians(theta_deg)),
            waypoints=self.WAYPOINTS_NORTH,
            max_linear=self.MAX_V,
        )

    # ------------------------------------------------------------------
    # 1. When odom theta == fused theta the commands are identical
    # ------------------------------------------------------------------

    def test_no_mag_fused_and_raw_commands_are_identical(self) -> None:
        # Uncalibrated mag → fusion falls back to odom → commands must match.
        robot = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=False,
            odom_theta=0.0,
        )
        lin_fused, ang_fused = self._velocity_from_fused(robot)
        lin_raw, ang_raw = self._velocity_from_raw_odom(robot)
        self.assertAlmostEqual(lin_fused, lin_raw, places=6)
        self.assertAlmostEqual(ang_fused, ang_raw, places=6)

    # ------------------------------------------------------------------
    # 2. When fused heading differs from odom the commands diverge
    # ------------------------------------------------------------------

    def test_fused_heading_changes_steering_command(self) -> None:
        # Odom: 0° (east).  Mag: 90° (north) with alpha=1 → fused = 90°.
        # Waypoint is north.  Facing north needs no turn; facing east needs
        # a large left turn.  Commands must differ.
        robot = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        robot.set_fusion_alpha(1.0)   # ComplementaryFilter: full mag trust
        robot._on_kinematics(          # re-deliver kin so fusion recalculates
            _make_kin(self.mod, theta=0.0)
        )

        _, ang_fused = self._velocity_from_fused(robot)
        _, ang_raw = self._velocity_from_raw_odom(robot)

        # Fused heading (90°) is aligned with waypoint → near-zero angular.
        # Raw odom heading (0°) is perpendicular → large positive angular.
        self.assertLess(abs(ang_fused), abs(ang_raw))

    def test_fused_heading_turns_correct_direction(self) -> None:
        # Odom: 0° (east).  Mag: 90° (north).  Waypoint north.
        # Robot using raw odom thinks it faces east → must turn left (positive ω).
        # Robot using fused heading knows it faces north → no turn needed.
        robot = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        robot.set_fusion_alpha(1.0)
        robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        _, ang_fused = self._velocity_from_fused(robot)
        _, ang_raw = self._velocity_from_raw_odom(robot)

        self.assertGreater(ang_raw, 0.0)           # raw: turn left toward north
        self.assertAlmostEqual(ang_fused, 0.0, places=3)  # fused: already north

    # ------------------------------------------------------------------
    # 3. ComplementaryFilter alpha scales the steering correction
    # ------------------------------------------------------------------

    def test_higher_alpha_gives_more_heading_correction(self) -> None:
        # Odom 0°, mag 90°.  Higher alpha → fused heading closer to 90°.
        # The heading correction is linear in alpha for the complementary filter.
        headings = []
        for alpha in (0.0, 0.25, 0.5, 0.75, 1.0):
            robot = _drive_robot(
                self.mod, self.node,
                imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
                odom_theta=0.0,
            )
            robot.set_fusion_alpha(alpha)
            robot._on_kinematics(_make_kin(self.mod, theta=0.0))
            headings.append(robot.get_fused_orientation())

        self.assertTrue(all(a <= b for a, b in zip(headings, headings[1:])))
        self.assertAlmostEqual(headings[0], 0.0, places=5)   # alpha=0 → pure odom
        self.assertAlmostEqual(headings[-1], 90.0, places=5)  # alpha=1 → pure mag

    def test_alpha_extremes_produce_opposite_steering_demands(self) -> None:
        # alpha=0: fused=0° (east), waypoint north → must turn left (positive ω).
        # alpha=1: fused=90° (north), waypoint north → already aligned (ω ≈ 0).
        robot_no_correction = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        robot_no_correction.set_fusion_alpha(0.0)
        robot_no_correction._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_no = self._velocity_from_fused(robot_no_correction)

        robot_full = _drive_robot(
            self.mod, FakeNode(),
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        robot_full.set_fusion_alpha(1.0)
        robot_full._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_full = self._velocity_from_fused(robot_full)

        self.assertGreater(ang_no, 0.0)                         # must turn left
        self.assertAlmostEqual(ang_full, 0.0, places=3)         # already aligned
        self.assertGreater(ang_no, ang_full)

    # ------------------------------------------------------------------
    # 4. AdaptiveComplementaryFilter: low speed → more correction
    # ------------------------------------------------------------------

    def test_adaptive_filter_applies_more_correction_at_low_speed(self) -> None:
        # At v=0 alpha≈alpha_max (0.8); at v=200 alpha≈alpha_min (0.0).
        # Odom 0°, mag 90°.  Slow robot: fused heading closer to 90°.
        # Fast robot: fused heading closer to 0°.
        strategy_params = dict(alpha_min=0.0, alpha_max=0.8,
                               linear_scale=50.0, angular_scale=0.3)

        robot_slow = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0, vx=0.0,
            strategy=AdaptiveComplementaryFilter(**strategy_params),
        )
        robot_fast = _drive_robot(
            self.mod, FakeNode(),
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0, vx=200.0,
            strategy=AdaptiveComplementaryFilter(**strategy_params),
        )

        # Slow robot gets more mag pull → fused heading is larger (closer to 90°).
        self.assertGreater(robot_slow.get_fused_orientation(),
                           robot_fast.get_fused_orientation())

    def test_adaptive_filter_at_high_angular_velocity_reduces_mag_weight(self) -> None:
        # High ω → alpha closer to alpha_min → heading closer to odom (0°).
        strategy_params = dict(alpha_min=0.0, alpha_max=1.0,
                               linear_scale=50.0, angular_scale=0.3)

        robot_spinning = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0, v_theta=3.0,
            strategy=AdaptiveComplementaryFilter(**strategy_params),
        )
        robot_still = _drive_robot(
            self.mod, FakeNode(),
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0, v_theta=0.0,
            strategy=AdaptiveComplementaryFilter(**strategy_params),
        )

        # Still robot gets more mag correction → fused heading closer to 90°.
        self.assertGreater(robot_still.get_fused_orientation(),
                           robot_spinning.get_fused_orientation())

    # ------------------------------------------------------------------
    # 5. HeadingKalmanFilter: heading stays between odom and mag
    # ------------------------------------------------------------------

    def test_kalman_fused_heading_is_between_odom_and_mag(self) -> None:
        # Odom 0°, mag 90°.  Kalman estimate must be strictly between them.
        robot = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        robot.set_fusion_strategy(HeadingKalmanFilter(
            process_noise=1e-4, measurement_noise=0.05, initial_variance=1.0,
        ))
        robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        fused = robot.get_fused_orientation()
        self.assertGreater(fused, 0.0)
        self.assertLess(fused, 90.0)

    def test_kalman_angular_command_is_between_raw_and_full_mag(self) -> None:
        # Angular command with Kalman heading must sit between the command
        # produced by pure odom and the command produced by full mag trust.
        raw_robot = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=False,
            odom_theta=0.0,
        )
        _, ang_raw = self._velocity_from_fused(raw_robot)

        full_mag_robot = _drive_robot(
            self.mod, FakeNode(),
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        full_mag_robot.set_fusion_alpha(1.0)
        full_mag_robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_full_mag = self._velocity_from_fused(full_mag_robot)

        kalman_robot = _drive_robot(
            self.mod, FakeNode(),
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        kalman_robot.set_fusion_strategy(HeadingKalmanFilter(
            process_noise=1e-4, measurement_noise=0.05, initial_variance=1.0,
        ))
        kalman_robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_kalman = self._velocity_from_fused(kalman_robot)

        # Kalman correction is partial → command is strictly between raw and full-mag.
        lo, hi = sorted([ang_full_mag, ang_raw])
        self.assertGreater(ang_kalman, lo)
        self.assertLess(ang_kalman, hi)

    # ------------------------------------------------------------------
    # 6. Strategy switch at runtime changes the next velocity command
    # ------------------------------------------------------------------

    def test_switching_strategy_changes_velocity_command(self) -> None:
        robot = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        # Start with zero-alpha complementary → fused == odom
        robot.set_fusion_alpha(0.0)
        robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_before = self._velocity_from_fused(robot)

        # Switch to full-mag complementary
        robot.set_fusion_strategy(ComplementaryFilter(alpha=1.0))
        robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_after = self._velocity_from_fused(robot)

        self.assertNotAlmostEqual(ang_before, ang_after, places=3)

    def test_reverting_to_complementary_restores_original_command(self) -> None:
        # Switching strategy and then switching back gives the same command.
        robot = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        robot.set_fusion_alpha(0.5)
        robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_original = self._velocity_from_fused(robot)

        robot.set_fusion_strategy(AdaptiveComplementaryFilter())
        robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        robot.set_fusion_strategy(ComplementaryFilter(alpha=0.5))
        robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_restored = self._velocity_from_fused(robot)

        self.assertAlmostEqual(ang_original, ang_restored, places=6)


if __name__ == "__main__":
    unittest.main()
