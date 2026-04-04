/**
 * @file RobotKinematics.h
 * @brief Robot kinematics model for odometry and velocity computation
 *
 * The default odometry parameters are seeded from config.h at boot and can be
 * updated at runtime via SYS_ODOM_PARAM_SET. To implement a different drive
 * model (e.g., Ackermann steering, mecanum wheels), replace the
 * RobotKinematics::update() implementation in RobotKinematics.cpp while
 * keeping this interface unchanged.
 *
 * Current model: Differential drive (two-wheeled)
 *
 * Usage:
 *   // In setup():
 *   RobotKinematics::reset(0, 0);
 *
 *   // In the sensor / odometry task (100 Hz by default):
 *   RobotKinematics::update(
 *       dcMotors[RobotKinematics::getLeftMotorId()].getPosition(),
 *       dcMotors[RobotKinematics::getRightMotorId()].getPosition(),
 *       dcMotors[RobotKinematics::getLeftMotorId()].getVelocity(),
 *       dcMotors[RobotKinematics::getRightMotorId()].getVelocity());
 */

#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <stdint.h>

// ============================================================================
// KINEMATICS CLASS
// ============================================================================

/**
 * @brief Differential-drive odometry and instantaneous velocity estimator.
 *
 * Integrates encoder deltas each cycle to maintain pose (x, y, theta) and
 * reports instantaneous body-frame velocities (vx, vy, vTheta).
 *
 * All positions are in millimetres and radians. Positive theta is CCW.
 * Heading is initialized from the current runtime initial-theta setting.
 */
class RobotKinematics {
public:
    /**
     * @brief Update runtime odometry parameters and reseed the encoder baseline.
     *
     * Pose is preserved. The baseline is reseeded so the next update() call
     * interprets the new motor selection against the current encoder counts
     * instead of creating an artificial jump.
     *
     * @return True when parameters are valid and were applied.
     */
    static bool setParameters(float wheelDiameterMm, float wheelBaseMm, float initialThetaDeg,
                              uint8_t leftMotorId, bool leftMotorDirInverted,
                              uint8_t rightMotorId, bool rightMotorDirInverted,
                              int32_t leftTicks, int32_t rightTicks);

    /**
     * @brief Reset pose to (0, 0, initial theta) and reseed the tick baseline.
     *
     * Call once in setup() and again whenever SYS_ODOM_RESET is handled
     * is received. Pass the current encoder counts so the first update()
     * call produces a zero delta.
     *
     * @param leftTicks  Current left encoder count
     * @param rightTicks Current right encoder count
     */
    static void reset(int32_t leftTicks, int32_t rightTicks);

    /**
     * @brief Preserve pose but reseed the encoder baseline.
     *
     * Use this when an encoder count is externally re-zeroed so the next
     * update() call does not interpret that count jump as robot motion.
     */
    static void reseed(int32_t leftTicks, int32_t rightTicks);

    /**
     * @brief Update odometry and instantaneous velocity.
     *
     * Call once per odometry integration cycle after refreshing motor state.
     * In the current firmware this runs from the 100 Hz sensor task.
     *
     * @param leftTicks   Absolute left encoder tick count
     * @param rightTicks  Absolute right encoder tick count
     * @param leftVelTps  Left wheel velocity  (ticks/sec, from DCMotor::getVelocity())
     * @param rightVelTps Right wheel velocity (ticks/sec, from DCMotor::getVelocity())
     */
    static void update(int32_t leftTicks, int32_t rightTicks,
                       float leftVelTps, float rightVelTps);

    /** @brief Current left odometry motor index (0-based). */
    static uint8_t getLeftMotorId() { return leftMotorId_; }

    /** @brief Current right odometry motor index (0-based). */
    static uint8_t getRightMotorId() { return rightMotorId_; }

    /** @brief Current wheel diameter used for odometry (mm). */
    static float getWheelDiameterMm() { return wheelDiameterMm_; }

    /** @brief Current wheel base used for odometry (mm). */
    static float getWheelBaseMm() { return wheelBaseMm_; }

    /** @brief Current initial heading used by future resets (degrees). */
    static float getInitialThetaDeg() { return initialThetaRad_ * 180.0f / 3.14159265358979323846f; }

    /** @brief Whether left odometry wheel direction is inverted. */
    static bool isLeftMotorDirInverted() { return leftMotorDirInverted_; }

    /** @brief Whether right odometry wheel direction is inverted. */
    static bool isRightMotorDirInverted() { return rightMotorDirInverted_; }

    /** @brief X position from start (mm) */
    static float getX()      { return x_; }

    /** @brief Y position from start (mm) */
    static float getY()      { return y_; }

    /** @brief Heading from start (radians, CCW positive) */
    static float getTheta()  { return theta_; }

    /** @brief Forward velocity in robot frame (mm/s) */
    static float getVx()     { return vx_; }

    /** @brief Lateral velocity in robot frame (mm/s, always 0 for diff drive) */
    static float getVy()     { return vy_; }

    /** @brief Angular velocity (rad/s, CCW positive) */
    static float getVTheta() { return vTheta_; }

private:
    static void refreshDerivedParameters();

    static float   x_;
    static float   y_;
    static float   theta_;
    static float   vx_;
    static float   vy_;
    static float   vTheta_;
    static float   wheelDiameterMm_;
    static float   wheelBaseMm_;
    static float   initialThetaRad_;
    static uint8_t leftMotorId_;
    static uint8_t rightMotorId_;
    static bool    leftMotorDirInverted_;
    static bool    rightMotorDirInverted_;
    static float   leftMmPerTick_;
    static float   rightMmPerTick_;
    static float   invWheelBase_;
    static int32_t prevLeftTicks_;
    static int32_t prevRightTicks_;
};

#endif // ROBOT_KINEMATICS_H
