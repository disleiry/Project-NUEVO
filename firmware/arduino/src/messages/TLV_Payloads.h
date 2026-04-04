/**
 * @file TLV_Payloads.h
 * @brief Packed payload definitions for the NUEVO TLV protocol
 *
 * Source of truth:
 *   - docs/COMMUNICATION_PROTOCOL.md
 *   - tlv_protocol/TLV_Payloads.md
 *   - tlv_protocol/TLV_TypeDefs.json
 *
 * IMPORTANT:
 *   All structs in this file are wire-format payloads and must remain tightly
 *   packed and byte-exact across firmware and Python.
 */

#ifndef TLV_PAYLOADS_H
#define TLV_PAYLOADS_H

#include <stdint.h>

#pragma pack(push, 1)

// ============================================================================
// COMMON LIMITS
// ============================================================================

constexpr uint8_t TLV_MAX_DC_MOTORS = 4U;
constexpr uint8_t TLV_MAX_STEPPERS = 4U;
constexpr uint8_t TLV_MAX_SERVO_CHANNELS = 16U;
constexpr uint8_t TLV_MAX_ULTRASONICS = 4U;
constexpr uint8_t TLV_MAX_USER_LEDS = 5U;

// ============================================================================
// SYSTEM PAYLOADS
// ============================================================================

struct PayloadHeartbeat {
    uint32_t timestamp;   // RPi milliseconds since boot
    uint8_t  flags;       // Reserved
};
// 5 bytes

enum SystemState : uint8_t {
    SYS_STATE_INIT    = 0,
    SYS_STATE_IDLE    = 1,
    SYS_STATE_RUNNING = 2,
    SYS_STATE_ERROR   = 3,
    SYS_STATE_ESTOP   = 4,
};

enum SystemWarningFlags : uint8_t {
    WARN_NONE           = 0x00,
    WARN_LIVENESS_LOST  = 0x01,
    WARN_LOOP_OVERRUN   = 0x02,
    WARN_NO_BATTERY     = 0x04,
    WARN_CONTROL_MISS   = 0x08,
};

enum SystemErrorFlags : uint8_t {
    ERR_NONE           = 0x00,
    ERR_UNDERVOLTAGE   = 0x01,
    ERR_OVERVOLTAGE    = 0x02,
    ERR_ENCODER_FAIL   = 0x04,
    ERR_I2C_ERROR      = 0x08,
    ERR_IMU_ERROR      = 0x10,
};

// Backward-compatible symbolic aliases used in existing firmware logic.
constexpr uint8_t ERR_LIVENESS_LOST = WARN_LIVENESS_LOST;
constexpr uint8_t ERR_LOOP_OVERRUN = WARN_LOOP_OVERRUN;

enum SystemRuntimeFlags : uint8_t {
    RTFLAG_LINK_OK         = 0x01,
    RTFLAG_I2C_OK          = 0x02,
    RTFLAG_SERVO_READY     = 0x04,
    RTFLAG_BATTERY_PRESENT = 0x08,
    RTFLAG_IMU_READY       = 0x10,
};

enum SystemFeatureFlags : uint8_t {
    FEATURE_DC_MOTORS = 0x01,
    FEATURE_STEPPERS  = 0x02,
    FEATURE_SERVOS    = 0x04,
    FEATURE_IMU       = 0x08,
    FEATURE_ULTRASONIC= 0x10,
    FEATURE_USER_LED  = 0x20,
    FEATURE_NEOPIXEL  = 0x40,
};

enum SensorConfigMask : uint8_t {
    SENSORCFG_IMU         = 0x01,
    SENSORCFG_ULTRASONIC  = 0x02,
};

struct PayloadSysState {
    uint8_t  state;         // SystemState
    uint8_t  warningFlags;  // SystemWarningFlags
    uint8_t  errorFlags;    // SystemErrorFlags
    uint8_t  runtimeFlags;  // SystemRuntimeFlags
    uint32_t uptimeMs;      // Arduino uptime
    uint16_t lastRxMs;      // ms since last TLV received
    uint16_t lastCmdMs;     // ms since last non-heartbeat command
};
// 12 bytes

enum SysCmdType : uint8_t {
    SYS_CMD_START  = 1,
    SYS_CMD_STOP   = 2,
    SYS_CMD_RESET  = 3,
    SYS_CMD_ESTOP  = 4,
};

struct PayloadSysCmd {
    uint8_t command;      // SysCmdType
    uint8_t reserved[3];
};
// 4 bytes

struct PayloadSysInfoReq {
    uint8_t target;       // Reserved for future use; send 0xFF
    uint8_t reserved[3];
};
// 4 bytes

struct PayloadSysInfoRsp {
    uint8_t  firmwareMajor;
    uint8_t  firmwareMinor;
    uint8_t  firmwarePatch;
    uint8_t  protocolMajor;
    uint8_t  protocolMinor;
    uint8_t  boardRevision;
    uint8_t  featureMask;          // SystemFeatureFlags
    uint8_t  sensorCapabilityMask; // SensorConfigMask
    uint8_t  dcMotorCount;
    uint8_t  stepperCount;
    uint8_t  servoChannelCount;
    uint8_t  ultrasonicMaxCount;
    uint8_t  userLedCount;
    uint8_t  maxNeoPixelCount;
    uint16_t limitSwitchMask;
    uint8_t  stepperHomeLimitGpio[TLV_MAX_STEPPERS];
    uint8_t  dcHomeLimitGpio[TLV_MAX_DC_MOTORS];
};
// 24 bytes

struct PayloadSysConfigReq {
    uint8_t target;       // Reserved for future use; send 0xFF
    uint8_t reserved[3];
};
// 4 bytes

struct PayloadSysConfigRsp {
    uint8_t  motorDirMask;          // Runtime direction inversion mask
    uint8_t  configuredSensorMask;  // SensorConfigMask
    uint8_t  neoPixelCount;         // Active NeoPixel count
    uint8_t  reserved;
    uint16_t heartbeatTimeoutMs;
    uint16_t reserved2;
};
// 8 bytes

struct PayloadSysConfigSet {
    uint8_t  motorDirMask;          // Direction inversion bits to apply
    uint8_t  motorDirChangeMask;    // Which motors to modify
    uint8_t  neoPixelCount;         // 0 = no change
    uint8_t  configuredSensorMask;  // 0xFF = no change
    uint16_t heartbeatTimeoutMs;    // 0 = no change
    uint16_t reserved;
};
// 8 bytes

struct PayloadSysPower {
    uint16_t batteryMv;
    uint16_t rail5vMv;
    uint16_t servoRailMv;
    uint8_t  batteryType;  // BATTERY_TYPE from config.h
    uint8_t  reserved;
    uint32_t timestamp;
};
// 12 bytes

struct PayloadSysDiagReq {
    uint8_t target;       // Reserved for future use; send 0xFF
    uint8_t reserved[3];
};
// 4 bytes

struct PayloadSysDiagRsp {
    uint16_t freeSram;
    uint16_t loopTimeAvgUs;
    uint16_t loopTimeMaxUs;
    uint16_t uartRxErrors;
    uint16_t crcErrors;
    uint16_t frameErrors;
    uint16_t tlvErrors;
    uint16_t oversizeErrors;
    uint16_t txPendingBytes;
    uint16_t reserved;
    uint32_t txDroppedFrames;
};
// 24 bytes

struct PayloadSysOdomReset {
    uint8_t flags;        // Reserved for future use; send 0
    uint8_t reserved[3];
};
// 4 bytes

struct PayloadSysOdomParamReq {
    uint8_t target;       // Reserved for future use; send 0xFF
    uint8_t reserved[3];
};
// 4 bytes

struct PayloadSysOdomParamSet {
    float wheelDiameterMm;
    float wheelBaseMm;
    float initialThetaDeg;
    uint8_t leftMotorId;            // 0-based
    uint8_t leftMotorDirInverted;   // 0 or 1
    uint8_t rightMotorId;           // 0-based
    uint8_t rightMotorDirInverted;  // 0 or 1
};
// 16 bytes

struct PayloadSysOdomParamRsp {
    float wheelDiameterMm;
    float wheelBaseMm;
    float initialThetaDeg;
    uint8_t leftMotorId;            // 0-based
    uint8_t leftMotorDirInverted;   // 0 or 1
    uint8_t rightMotorId;           // 0-based
    uint8_t rightMotorDirInverted;  // 0 or 1
};
// 16 bytes

// ============================================================================
// DC MOTOR PAYLOADS
// ============================================================================

enum DCMotorMode : uint8_t {
    DC_MODE_DISABLED = 0,
    DC_MODE_POSITION = 1,
    DC_MODE_VELOCITY = 2,
    DC_MODE_PWM      = 3,
    DC_MODE_HOMING   = 4,
};

enum DCPidLoopType : uint8_t {
    DC_PID_LOOP_POSITION = 0,
    DC_PID_LOOP_VELOCITY = 1,
};

struct PayloadDCEnable {
    uint8_t motorId;
    uint8_t mode;         // DCMotorMode
    uint8_t reserved[2];
};
// 4 bytes

struct PayloadDCSetPosition {
    uint8_t motorId;
    uint8_t reserved[3];
    int32_t targetTicks;
    int32_t maxVelTicks;
};
// 12 bytes

struct PayloadDCSetVelocity {
    uint8_t motorId;
    uint8_t reserved[3];
    int32_t targetTicks;
};
// 8 bytes

struct PayloadDCSetPWM {
    uint8_t motorId;
    uint8_t reserved;
    int16_t pwm;
};
// 4 bytes

struct PayloadDCResetPosition {
    uint8_t motorId;
    uint8_t reserved[3];
};
// 4 bytes

struct PayloadDCHome {
    uint8_t motorId;
    int8_t  direction;     // +1 or -1
    uint8_t reserved[2];
    int32_t homeVelocity;  // ticks/sec magnitude; 0 = use firmware default
};
// 8 bytes

struct DCMotorState {
    uint8_t mode;         // DCMotorMode
    uint8_t faultFlags;
    int32_t position;
    int32_t velocity;
    int32_t targetPos;
    int32_t targetVel;
    int16_t pwmOutput;
    int16_t currentMa;
};
// 22 bytes

struct PayloadDCStateAll {
    DCMotorState motors[TLV_MAX_DC_MOTORS];
    uint32_t     timestamp;
};
// 92 bytes

struct PayloadDCPidReq {
    uint8_t motorId;
    uint8_t loopType;     // DCPidLoopType
    uint8_t reserved[2];
};
// 4 bytes

struct PayloadDCPidRsp {
    uint8_t motorId;
    uint8_t loopType;     // DCPidLoopType
    uint8_t reserved[2];
    float   kp;
    float   ki;
    float   kd;
    float   maxOutput;
    float   maxIntegral;
};
// 24 bytes

using PayloadDCPidSet = PayloadDCPidRsp;

// ============================================================================
// STEPPER PAYLOADS
// ============================================================================

struct PayloadStepEnable {
    uint8_t stepperId;
    uint8_t enable;
    uint8_t reserved[2];
};
// 4 bytes

enum StepMoveType : uint8_t {
    STEP_MOVE_ABSOLUTE = 0,
    STEP_MOVE_RELATIVE = 1,
};

struct PayloadStepMove {
    uint8_t stepperId;
    uint8_t moveType;     // StepMoveType
    uint8_t reserved[2];
    int32_t target;
};
// 8 bytes

struct PayloadStepHome {
    uint8_t stepperId;
    int8_t  direction;
    uint8_t reserved[2];
    uint32_t homeVelocity;
    int32_t backoffSteps;
};
// 12 bytes

struct PayloadStepConfigReq {
    uint8_t stepperId;    // 0xFF = all (future use); current default is one stepper
    uint8_t reserved[3];
};
// 4 bytes

struct PayloadStepConfigRsp {
    uint8_t  stepperId;
    uint8_t  reserved[3];
    uint32_t maxVelocity;
    uint32_t acceleration;
};
// 12 bytes

using PayloadStepConfigSet = PayloadStepConfigRsp;

enum StepperMotionState : uint8_t {
    STEPPER_IDLE   = 0,
    STEPPER_ACCEL  = 1,
    STEPPER_CRUISE = 2,
    STEPPER_DECEL  = 3,
    STEPPER_HOMING = 4,
};

struct StepperChannelState {
    uint8_t enabled;
    uint8_t motionState;
    uint8_t limitFlags;
    int8_t  reserved;
    int32_t count;        // Open-loop commanded count
    int32_t targetCount;
    int32_t currentSpeed; // Signed steps/sec
};
// 16 bytes

struct PayloadStepStateAll {
    StepperChannelState steppers[TLV_MAX_STEPPERS];
    uint32_t     timestamp;
};
// 68 bytes

// ============================================================================
// SERVO PAYLOADS
// ============================================================================

struct PayloadServoEnable {
    uint8_t channel;      // 0xFF = all
    uint8_t enable;
    uint8_t reserved[2];
};
// 4 bytes

struct PayloadServoSetSingle {
    uint8_t  channel;
    uint8_t  count;       // Must be 1
    uint16_t pulseUs[1];
};
// 4 bytes

struct PayloadServoSetBulk {
    uint8_t  startChannel;
    uint8_t  count;
    uint16_t pulseUs[TLV_MAX_SERVO_CHANNELS];
};
// 34 bytes

struct PayloadServoStateAll {
    uint8_t  pca9685Connected;
    uint8_t  pca9685Error;
    uint16_t enabledMask;
    uint16_t pulseUs[TLV_MAX_SERVO_CHANNELS];
    uint32_t timestamp;
};
// 40 bytes

// ============================================================================
// SENSOR PAYLOADS
// ============================================================================

struct PayloadSensorIMU {
    float    quatW;
    float    quatX;
    float    quatY;
    float    quatZ;
    float    earthAccX;
    float    earthAccY;
    float    earthAccZ;
    int16_t  rawAccX;
    int16_t  rawAccY;
    int16_t  rawAccZ;
    int16_t  rawGyroX;
    int16_t  rawGyroY;
    int16_t  rawGyroZ;
    int16_t  magX;
    int16_t  magY;
    int16_t  magZ;
    uint8_t  magCalibrated;
    uint8_t  reserved;
    uint32_t timestamp;
};
// 52 bytes

struct PayloadSensorKinematics {
    float    x;
    float    y;
    float    theta;
    float    vx;
    float    vy;
    float    vTheta;
    uint32_t timestamp;
};
// 28 bytes

enum UltrasonicStatus : uint8_t {
    ULTRA_STATUS_VALID         = 0,
    ULTRA_STATUS_OUT_OF_RANGE  = 1,
    ULTRA_STATUS_SENSOR_ERROR  = 2,
    ULTRA_STATUS_NOT_INSTALLED = 3,
};

struct UltrasonicState {
    uint8_t  status;       // UltrasonicStatus
    uint8_t  reserved;
    uint16_t distanceMm;
};
// 4 bytes

struct PayloadSensorUltrasonicAll {
    uint8_t         configuredCount;
    uint8_t         reserved[3];
    UltrasonicState sensors[TLV_MAX_ULTRASONICS];
    uint32_t        timestamp;
};
// 24 bytes

enum MagCalCmdType : uint8_t {
    MAG_CAL_START = 1,
    MAG_CAL_STOP  = 2,
    MAG_CAL_SAVE  = 3,   // Legacy hard-iron-only save
    MAG_CAL_APPLY = 4,
    MAG_CAL_CLEAR = 5,
};

struct PayloadMagCalCmd {
    uint8_t command;            // MagCalCmdType
    uint8_t reserved[3];
    float   offsetX;
    float   offsetY;
    float   offsetZ;
    float   softIronMatrix[9];  // Row-major 3x3
};
// 52 bytes

enum MagCalState : uint8_t {
    MAG_CAL_STATE_IDLE     = 0,
    MAG_CAL_STATE_SAMPLING = 1,
    MAG_CAL_STATE_COMPLETE = 2,
    MAG_CAL_STATE_SAVED    = 3,
    MAG_CAL_STATE_ERROR    = 4,
};

struct PayloadMagCalStatus {
    uint8_t state;              // MagCalState
    uint16_t sampleCount;
    uint8_t reserved;
    float   minX;
    float   maxX;
    float   minY;
    float   maxY;
    float   minZ;
    float   maxZ;
    float   offsetX;
    float   offsetY;
    float   offsetZ;
    uint8_t savedToEeprom;
    uint8_t reserved2[3];
};
// 44 bytes

// ============================================================================
// USER I/O PAYLOADS
// ============================================================================

enum LEDMode : uint8_t {
    LED_OFF     = 0,
    LED_ON      = 1,
    LED_BLINK   = 2,
    LED_BREATHE = 3,
    LED_PWM     = 4,
};

struct PayloadSetLED {
    uint8_t  ledId;
    uint8_t  mode;          // LEDMode
    uint8_t  brightness;
    uint8_t  reserved;
    uint16_t periodMs;
    uint16_t dutyCycle;
};
// 8 bytes

struct PayloadSetNeoPixel {
    uint8_t index;          // 0xFF = all
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};
// 4 bytes

struct PayloadIOInputState {
    uint16_t buttonMask;
    uint16_t limitMask;
    uint32_t timestamp;
};
// 8 bytes

struct PayloadIOOutputState {
    uint8_t  ledBrightness[TLV_MAX_USER_LEDS];
    uint8_t  neoPixelCount;
    uint8_t  reserved;
    uint32_t timestamp;
    // uint8_t neoPixelRgb[neoPixelCount * 3]; appended after fixed portion
};
// 11 bytes fixed + 3 * neoPixelCount bytes

#pragma pack(pop)

// ============================================================================
// SIZE VALIDATION
// ============================================================================

#define STATIC_ASSERT_SIZE(type, expected) \
    static_assert(sizeof(type) == expected, #type " size mismatch")

STATIC_ASSERT_SIZE(PayloadHeartbeat, 5);
STATIC_ASSERT_SIZE(PayloadSysState, 12);
STATIC_ASSERT_SIZE(PayloadSysCmd, 4);
STATIC_ASSERT_SIZE(PayloadSysInfoReq, 4);
STATIC_ASSERT_SIZE(PayloadSysInfoRsp, 24);
STATIC_ASSERT_SIZE(PayloadSysConfigReq, 4);
STATIC_ASSERT_SIZE(PayloadSysConfigRsp, 8);
STATIC_ASSERT_SIZE(PayloadSysConfigSet, 8);
STATIC_ASSERT_SIZE(PayloadSysPower, 12);
STATIC_ASSERT_SIZE(PayloadSysDiagReq, 4);
STATIC_ASSERT_SIZE(PayloadSysDiagRsp, 24);
STATIC_ASSERT_SIZE(PayloadSysOdomReset, 4);
STATIC_ASSERT_SIZE(PayloadSysOdomParamReq, 4);
STATIC_ASSERT_SIZE(PayloadSysOdomParamSet, 16);
STATIC_ASSERT_SIZE(PayloadSysOdomParamRsp, 16);

STATIC_ASSERT_SIZE(PayloadDCEnable, 4);
STATIC_ASSERT_SIZE(PayloadDCSetPosition, 12);
STATIC_ASSERT_SIZE(PayloadDCSetVelocity, 8);
STATIC_ASSERT_SIZE(PayloadDCSetPWM, 4);
STATIC_ASSERT_SIZE(PayloadDCResetPosition, 4);
STATIC_ASSERT_SIZE(PayloadDCHome, 8);
STATIC_ASSERT_SIZE(DCMotorState, 22);
STATIC_ASSERT_SIZE(PayloadDCStateAll, 92);
STATIC_ASSERT_SIZE(PayloadDCPidReq, 4);
STATIC_ASSERT_SIZE(PayloadDCPidRsp, 24);
STATIC_ASSERT_SIZE(PayloadDCPidSet, 24);

STATIC_ASSERT_SIZE(PayloadStepEnable, 4);
STATIC_ASSERT_SIZE(PayloadStepMove, 8);
STATIC_ASSERT_SIZE(PayloadStepHome, 12);
STATIC_ASSERT_SIZE(PayloadStepConfigReq, 4);
STATIC_ASSERT_SIZE(PayloadStepConfigRsp, 12);
STATIC_ASSERT_SIZE(PayloadStepConfigSet, 12);
STATIC_ASSERT_SIZE(StepperChannelState, 16);
STATIC_ASSERT_SIZE(PayloadStepStateAll, 68);

STATIC_ASSERT_SIZE(PayloadServoEnable, 4);
STATIC_ASSERT_SIZE(PayloadServoSetSingle, 4);
STATIC_ASSERT_SIZE(PayloadServoSetBulk, 34);
STATIC_ASSERT_SIZE(PayloadServoStateAll, 40);

STATIC_ASSERT_SIZE(PayloadSensorIMU, 52);
STATIC_ASSERT_SIZE(PayloadSensorKinematics, 28);
STATIC_ASSERT_SIZE(UltrasonicState, 4);
STATIC_ASSERT_SIZE(PayloadSensorUltrasonicAll, 24);
STATIC_ASSERT_SIZE(PayloadMagCalCmd, 52);
STATIC_ASSERT_SIZE(PayloadMagCalStatus, 44);

STATIC_ASSERT_SIZE(PayloadSetLED, 8);
STATIC_ASSERT_SIZE(PayloadSetNeoPixel, 4);
STATIC_ASSERT_SIZE(PayloadIOInputState, 8);
STATIC_ASSERT_SIZE(PayloadIOOutputState, 11);

#endif // TLV_PAYLOADS_H
