/**
 * @file RobotKinematics.cpp
 * @brief Differential-drive odometry implementation
 *
 * To adapt for a different drive model (e.g., Ackermann steering):
 *   1. Update the runtime odometry parameters to match your geometry.
 *   2. Replace the update() body below with your kinematics equations.
 *      - reset() and the static members do not need to change.
 *      - The interface (getX, getY, getTheta, getVx, getVy, getVTheta) stays the same.
 */

#include "RobotKinematics.h"
#include "../config.h"
#include <math.h>

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float countsPerRevForMode(uint8_t mode)
{
    return (mode == ENCODER_2X) ? ((float)ENCODER_PPR * 0.5f) : (float)ENCODER_PPR;
}
constexpr float kTicksPerRev[4] = {
    countsPerRevForMode(ENCODER_1_MODE),
    countsPerRevForMode(ENCODER_2_MODE),
    countsPerRevForMode(ENCODER_3_MODE),
    countsPerRevForMode(ENCODER_4_MODE)
};
} // namespace

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

float   RobotKinematics::x_             = 0.0f;
float   RobotKinematics::y_             = 0.0f;
float   RobotKinematics::theta_         = INITIAL_THETA * kPi / 180.0f;
float   RobotKinematics::vx_            = 0.0f;
float   RobotKinematics::vy_            = 0.0f;
float   RobotKinematics::vTheta_        = 0.0f;
float   RobotKinematics::wheelDiameterMm_ = WHEEL_DIAMETER_MM;
float   RobotKinematics::wheelBaseMm_     = WHEEL_BASE_MM;
float   RobotKinematics::initialThetaRad_ = INITIAL_THETA * kPi / 180.0f;
uint8_t RobotKinematics::leftMotorId_     = ODOM_LEFT_MOTOR;
uint8_t RobotKinematics::rightMotorId_    = ODOM_RIGHT_MOTOR;
bool    RobotKinematics::leftMotorDirInverted_ = (ODOM_LEFT_MOTOR_DIR_INVERTED != 0);
bool    RobotKinematics::rightMotorDirInverted_ = (ODOM_RIGHT_MOTOR_DIR_INVERTED != 0);
float   RobotKinematics::leftMmPerTick_   = (kPi * WHEEL_DIAMETER_MM) / kTicksPerRev[ODOM_LEFT_MOTOR];
float   RobotKinematics::rightMmPerTick_  = (kPi * WHEEL_DIAMETER_MM) / kTicksPerRev[ODOM_RIGHT_MOTOR];
float   RobotKinematics::invWheelBase_    = 1.0f / WHEEL_BASE_MM;
int32_t RobotKinematics::prevLeftTicks_ = 0;
int32_t RobotKinematics::prevRightTicks_= 0;

// ============================================================================
// COMPILE-TIME VALIDATION
// ============================================================================

static_assert(ODOM_LEFT_MOTOR  < NUM_DC_MOTORS, "ODOM_LEFT_MOTOR must be less than NUM_DC_MOTORS");
static_assert(ODOM_RIGHT_MOTOR < NUM_DC_MOTORS, "ODOM_RIGHT_MOTOR must be less than NUM_DC_MOTORS");
static_assert(ODOM_LEFT_MOTOR  != ODOM_RIGHT_MOTOR, "ODOM_LEFT_MOTOR and ODOM_RIGHT_MOTOR must be different");
static_assert(NUM_DC_MOTORS == 4, "RobotKinematics assumes four DC motor channels");

// ============================================================================
// PUBLIC API
// ============================================================================

bool RobotKinematics::setParameters(float wheelDiameterMm, float wheelBaseMm, float initialThetaDeg,
                                    uint8_t leftMotorId, bool leftMotorDirInverted,
                                    uint8_t rightMotorId, bool rightMotorDirInverted,
                                    int32_t leftTicks, int32_t rightTicks) {
    if (!isfinite(wheelDiameterMm) || !isfinite(wheelBaseMm) || !isfinite(initialThetaDeg) ||
        wheelDiameterMm <= 0.0f || wheelBaseMm <= 0.0f) {
        return false;
    }
    if (leftMotorId >= NUM_DC_MOTORS || rightMotorId >= NUM_DC_MOTORS || leftMotorId == rightMotorId) {
        return false;
    }

    wheelDiameterMm_ = wheelDiameterMm;
    wheelBaseMm_ = wheelBaseMm;
    initialThetaRad_ = initialThetaDeg * kPi / 180.0f;
    leftMotorId_ = leftMotorId;
    rightMotorId_ = rightMotorId;
    leftMotorDirInverted_ = leftMotorDirInverted;
    rightMotorDirInverted_ = rightMotorDirInverted;
    refreshDerivedParameters();
    reseed(leftTicks, rightTicks);
    return true;
}

void RobotKinematics::reset(int32_t leftTicks, int32_t rightTicks) {
    x_              = 0.0f;
    y_              = 0.0f;
    theta_          = initialThetaRad_;
    vx_             = 0.0f;
    vy_             = 0.0f;
    vTheta_         = 0.0f;
    prevLeftTicks_  = leftTicks;
    prevRightTicks_ = rightTicks;
}

void RobotKinematics::reseed(int32_t leftTicks, int32_t rightTicks) {
    vx_ = 0.0f;
    vy_ = 0.0f;
    vTheta_ = 0.0f;
    prevLeftTicks_ = leftTicks;
    prevRightTicks_ = rightTicks;
}

void RobotKinematics::update(int32_t leftTicks, int32_t rightTicks,
                              float leftVelTps, float rightVelTps)
{
    // ---- Odometry integration ----
    int32_t dL = leftTicks  - prevLeftTicks_;
    if (leftMotorDirInverted_) {
        dL = -dL;
    }

    int32_t dR = rightTicks - prevRightTicks_;
    if (rightMotorDirInverted_) {
        dR = -dR;
    }

    prevLeftTicks_  = leftTicks;
    prevRightTicks_ = rightTicks;

    if (dL != 0 || dR != 0) {
        float dLeft   = (float)dL * leftMmPerTick_;
        float dRight  = (float)dR * rightMmPerTick_;
        float dCenter = (dLeft + dRight) * 0.5f;
        float dTheta  = (dRight - dLeft) * invWheelBase_;

        // Midpoint heading integration (reduces discretization error)
        float headingMid = theta_ + dTheta * 0.5f;
        x_     += dCenter * cosf(headingMid);
        y_     += dCenter * sinf(headingMid);
        theta_ = theta_ + dTheta; // We intentionally NOT mod theta to 2 PI here.
    }

    // ---- Instantaneous body-frame velocities ----
    if (leftMotorDirInverted_) {
        leftVelTps = -leftVelTps;
    }
    if (rightMotorDirInverted_) {
        rightVelTps = -rightVelTps;
    }

    float vLeft  = leftVelTps  * leftMmPerTick_;
    float vRight = rightVelTps * rightMmPerTick_;

    vx_     = (vLeft + vRight) * 0.5f;
    vy_     = 0.0f;  // Always 0 for differential drive
    vTheta_ = (vRight - vLeft) * invWheelBase_;
}

void RobotKinematics::refreshDerivedParameters()
{
    const float wheelCircumferenceMm = kPi * wheelDiameterMm_;
    leftMmPerTick_ = wheelCircumferenceMm / kTicksPerRev[leftMotorId_];
    rightMmPerTick_ = wheelCircumferenceMm / kTicksPerRev[rightMotorId_];
    invWheelBase_ = 1.0f / wheelBaseMm_;
}
