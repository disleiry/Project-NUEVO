/**
 * @file MessageCenter.cpp
 * @brief Implementation of central message routing (TLV protocol v2.0)
 *
 * Telemetry batching:
 *   sendTelemetry() calls beginFrame() once, appends each due TLV with
 *   addTlvPacket(), then calls sendFrame() — transmitting a single
 *   multi-TLV frame per 100 Hz tick instead of N separate frames.
 *
 * Incoming:
 *   processIncoming() feeds raw bytes into the TLV decoder; decodeCallback()
 *   is invoked synchronously for each complete frame and routes every TLV in
 *   that frame to routeMessage().
 */

#include "MessageCenter.h"
#include "RobotKinematics.h"
#include "SensorManager.h"
#include "UserIO.h"
#include "../SystemManager.h"
#include "../drivers/DCMotor.h"
#include "../drivers/StepperMotor.h"
#include "../drivers/ServoController.h"
#include "StepperManager.h"
#include <string.h>
#include <math.h>

// External references to motor arrays (defined in arduino.ino)
extern DCMotor dcMotors[NUM_DC_MOTORS];

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

struct TlvEncodeDescriptor MessageCenter::encodeDesc_;
struct TlvDecodeDescriptor MessageCenter::decodeDesc_;

uint8_t *MessageCenter::txBuffer_;
uint8_t MessageCenter::rxBuffer_[RX_BUFFER_SIZE];
uint16_t MessageCenter::rxFill_ = 0;

uint32_t MessageCenter::lastHeartbeatMs_ = 0;
uint32_t MessageCenter::lastCmdMs_ = 0;
bool MessageCenter::heartbeatValid_ = true;   // true → 2s grace period on boot before timeout fires
uint16_t MessageCenter::heartbeatTimeoutMs_ = HEARTBEAT_TIMEOUT_MS;

uint8_t MessageCenter::motorDirMask_ = 0;
uint8_t MessageCenter::neoPixelCount_ = NEOPIXEL_COUNT;

uint16_t MessageCenter::servoEnabledMask_ = 0;
uint16_t MessageCenter::loopTimeAvgUs_ = 0;
uint16_t MessageCenter::loopTimeMaxUs_ = 0;
uint16_t MessageCenter::uartRxErrors_ = 0;

uint32_t MessageCenter::lastDCStatusSendMs_ = 0;
uint32_t MessageCenter::lastStepStatusSendMs_ = 0;
uint32_t MessageCenter::lastServoStatusSendMs_ = 0;
uint32_t MessageCenter::lastIMUSendMs_ = 0;
uint32_t MessageCenter::lastKinematicsSendMs_ = 0;
uint32_t MessageCenter::lastVoltageSendMs_ = 0;
uint32_t MessageCenter::lastIOStatusSendMs_ = 0;
uint32_t MessageCenter::lastStatusSendMs_ = 0;
uint32_t MessageCenter::lastMagCalSendMs_ = 0;
uint32_t MessageCenter::lastLidarSendMs_ = 0;
uint32_t MessageCenter::lastUltrasonicSendMs_ = 0;

bool MessageCenter::pendingMagCal_ = false;
bool MessageCenter::initialized_ = false;
volatile uint8_t MessageCenter::faultLatchFlags_ = 0;

// ============================================================================
// INITIALIZATION
// ============================================================================

void MessageCenter::init()
{
    if (initialized_)
        return;

    // Open Serial2 for RPi communication
    RPI_SERIAL.begin(RPI_BAUD_RATE);

    // Initialise TX encoder
    initEncodeDescriptor(&encodeDesc_, TX_BUFFER_SIZE, DEVICE_ID, ENABLE_CRC_CHECK);
    txBuffer_ = encodeDesc_.buffer;

    // Initialise RX decoder with callback
    initDecodeDescriptor(&decodeDesc_, RX_BUFFER_SIZE, ENABLE_CRC_CHECK, decodeCallback);

    // Stagger telemetry timers so large packets don't land on the same tick.
    // Offsets chosen so DC (20ms), stepper (100ms), and servo (40ms) never
    // all fire in the same taskUART call.
    uint32_t now = millis();
    lastDCStatusSendMs_    = now;        // fires at now+20ms
    lastStepStatusSendMs_  = now - 50;  // fires at now+50ms (midway through DC cycle)
    lastServoStatusSendMs_ = now - 10;  // fires at now+30ms (between DC ticks)
    lastKinematicsSendMs_  = now;
    lastIMUSendMs_         = now;
    lastIOStatusSendMs_    = now;
    lastVoltageSendMs_     = now;
    lastStatusSendMs_      = now;
    lastMagCalSendMs_      = now;
    lastLidarSendMs_       = now;
    lastUltrasonicSendMs_  = now;

    lastHeartbeatMs_ = now;
    lastCmdMs_ = now;
    // Start valid so SafetyManager doesn't fire during the boot grace period.
    // checkHeartbeatTimeout() will clear this after HEARTBEAT_TIMEOUT_MS if no
    // TLV arrives. routeMessage() re-sets it on every received packet.
    heartbeatValid_ = true;
    pendingMagCal_ = false;
    RobotKinematics::reset(0, 0);
    SystemManager::requestTransition(SYS_STATE_IDLE);

    initialized_ = true;

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.println(F("[MessageCenter] Initialized (v2.0, batched TX)"));
#endif
}

// ============================================================================
// FRAME HELPERS
// ============================================================================

void MessageCenter::beginFrame()
{
    resetDescriptor(&encodeDesc_);
}

void MessageCenter::sendFrame()
{
    // Only send if at least one TLV was appended
    if (encodeDesc_.frameHeader.numTlvs == 0)
        return;

    int n = wrapupBuffer(&encodeDesc_);
    if (n > 0)
    {
        RPI_SERIAL.write(txBuffer_, (size_t)n);
    }

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[TX] Frame: "));
    DEBUG_SERIAL.print(encodeDesc_.frameHeader.numTlvs);
    DEBUG_SERIAL.print(F(" TLVs, "));
    DEBUG_SERIAL.print(n);
    DEBUG_SERIAL.println(F(" bytes"));
#endif
}

// ============================================================================
// LOOP TIMING
// ============================================================================

void MessageCenter::recordLoopTime(uint32_t elapsedUs) {
    // Exponential moving average (alpha = 1/8) for display in SYS_STATUS.
    // NOTE: elapsedUs includes ISR preemption time — see header for details.
    uint32_t clamped = (elapsedUs > 0xFFFF) ? 0xFFFF : elapsedUs;
    if (loopTimeAvgUs_ == 0)
        loopTimeAvgUs_ = (uint16_t)clamped;
    else
        loopTimeAvgUs_ = (uint16_t)(((uint32_t)loopTimeAvgUs_ * 7 + clamped) >> 3);

    // Per-window max: reset every 200 ticks (2 s at 100 Hz)
    if (elapsedUs > loopTimeMaxUs_)
        loopTimeMaxUs_ = (uint16_t)clamped;

    static uint8_t windowTick = 0;
    if (++windowTick >= 200) {
        windowTick    = 0;
        loopTimeMaxUs_ = 0;
    }
}

// ============================================================================
// MESSAGE PROCESSING
// ============================================================================

// ----------------------------------------------------------------------------
// drainUart — called from loop() on EVERY iteration
// ----------------------------------------------------------------------------
void MessageCenter::drainUart()
{
    // Rapidly move bytes from the 64-byte HW ring buffer into the 1024-byte
    // software buffer.  At loop() rate (~67 kHz) the HW ring buffer never
    // accumulates more than ~1 byte between drain calls, preventing overflow
    // when the RPi sends back-to-back frames (heartbeat + command ≥ 96 bytes).
    int b;
    while (rxFill_ < RX_BUFFER_SIZE && (b = RPI_SERIAL.read()) >= 0)
        rxBuffer_[rxFill_++] = (uint8_t)b;
}

// ----------------------------------------------------------------------------
// processIncoming — called from taskUART at 100 Hz
// ----------------------------------------------------------------------------
void MessageCenter::processIncoming()
{
    // Final drain: pick up any bytes that arrived since the last drainUart() call.
    drainUart();

    // Decode all accumulated bytes, then reset the fill counter so the next
    // drainUart() call starts filling from the beginning of rxBuffer_.
    if (rxFill_ > 0)
    {
        decode(&decodeDesc_, rxBuffer_, rxFill_);
        rxFill_ = 0;
    }

    checkHeartbeatTimeout();
}

void MessageCenter::sendTelemetry()
{
    uint32_t currentMs = millis();

    SystemState state = SystemManager::getState();
    bool running = (state == SYS_STATE_RUNNING);
    bool runningOrError = (state == SYS_STATE_RUNNING ||
                           state == SYS_STATE_ERROR);

    // Open a new frame; all send*() calls below append TLVs to it.
    beginFrame();

    // ---- 100 Hz telemetry (RUNNING only) ----
    if (running)
    {
        if (currentMs - lastDCStatusSendMs_ >= 20)    // 50 Hz (184 B) — every 2nd tick
        {
            lastDCStatusSendMs_ = currentMs;
            sendDCStatusAll();
        }

        if (currentMs - lastStepStatusSendMs_ >= 100) // 10 Hz (96 B) — every 10th tick
        {
            lastStepStatusSendMs_ = currentMs;
            sendStepStatusAll();
        }

        if (currentMs - lastKinematicsSendMs_ >= 10)
        {
            lastKinematicsSendMs_ = currentMs;
            sendSensorKinematics();
        }

        if (currentMs - lastIOStatusSendMs_ >= 10)
        {
            lastIOStatusSendMs_ = currentMs;
            sendIOStatus();
        }

        if (SensorManager::isIMUAvailable() &&
            currentMs - lastIMUSendMs_ >= 10)
        {
            lastIMUSendMs_ = currentMs;
            sendSensorIMU();
        }

        // Lidar range at 50 Hz (each configured slot, found or not)
        if (SensorManager::getLidarConfiguredCount() > 0 &&
            currentMs - lastLidarSendMs_ >= 20)
        {
            lastLidarSendMs_ = currentMs;
            sendSensorRange(true);
        }

        // ---- 25 Hz telemetry — staggered from DC (40ms vs 20ms: lands between DC ticks)
        if (currentMs - lastServoStatusSendMs_ >= 40)
        {
            lastServoStatusSendMs_ = currentMs;
            sendServoStatusAll();
        }
    }

    // Ultrasonic range at 10 Hz (each configured slot, found or not)
    if (running && SensorManager::getUltrasonicConfiguredCount() > 0 &&
        currentMs - lastUltrasonicSendMs_ >= 100)
    {
        lastUltrasonicSendMs_ = currentMs;
        sendSensorRange(false);
    }

    // ---- 10 Hz voltage (RUNNING or ERROR) ----
    if (runningOrError && currentMs - lastVoltageSendMs_ >= 100)
    {
        lastVoltageSendMs_ = currentMs;
        sendVoltageData();
    }

    // ---- System status: 10 Hz (RUNNING/ERROR), 1 Hz otherwise ----
    uint32_t statusInterval = runningOrError ? 100UL : 1000UL;
    if (currentMs - lastStatusSendMs_ >= statusInterval)
    {
        lastStatusSendMs_ = currentMs;
        sendSystemStatus();
    }

    // ---- Mag cal status at 10 Hz while sampling ----
    if (SensorManager::getMagCalData().state == MAG_CAL_SAMPLING)
    {
        if (currentMs - lastMagCalSendMs_ >= 100)
        {
            lastMagCalSendMs_ = currentMs;
            sendMagCalStatus();
        }
    }

    // ---- Queued immediate mag cal response (STOP / SAVE / APPLY / CLEAR) ----
    if (pendingMagCal_)
    {
        pendingMagCal_ = false;
        sendMagCalStatus();
    }

    // Transmit the completed frame (no-ops if nothing was appended)
    sendFrame();
}

// ============================================================================
// LIVENESS MONITORING
// ============================================================================

bool MessageCenter::isHeartbeatValid()
{
    return heartbeatValid_;
}

uint32_t MessageCenter::getTimeSinceHeartbeat()
{
    return millis() - lastHeartbeatMs_;
}

void MessageCenter::checkHeartbeatTimeout()
{
    // Only mark heartbeat invalid — SafetyManager::check() responds on next 100 Hz tick
    if (getTimeSinceHeartbeat() > (uint32_t)heartbeatTimeoutMs_)
    {
        // DEBUG_SERIAL.print("getTimeSinceHeartbeat(): ");
        // DEBUG_SERIAL.println(getTimeSinceHeartbeat());
        heartbeatValid_ = false;
    }
}

// ============================================================================
// DECODE CALLBACK
// ============================================================================

void MessageCenter::decodeCallback(enum DecodeErrorCode *error,
                                   const struct FrameHeader *frameHeader,
                                   struct TlvHeader *tlvHeaders,
                                   uint8_t **tlvData)
{
    if (*error != NoError)
    {
        uartRxErrors_++;
#ifdef DEBUG_TLV_PACKETS
        DEBUG_SERIAL.print(F("[RX] Decode error: "));
        DEBUG_SERIAL.println(*error);
#endif
        return;
    }

    // Route every TLV in the frame — supports multi-TLV incoming frames
    for (uint32_t i = 0; i < frameHeader->numTlvs; i++)
    {
        uint32_t length = tlvHeaders[i].tlvLen;
        if (length > MAX_TLV_PAYLOAD_SIZE)
        {
            uartRxErrors_++;
#ifdef DEBUG_TLV_PACKETS
            DEBUG_SERIAL.print(F("[RX] Oversized payload, type="));
            DEBUG_SERIAL.println(tlvHeaders[i].tlvType);
#endif
            continue;
        }

        routeMessage(tlvHeaders[i].tlvType, tlvData[i], length);

#ifdef DEBUG_TLV_PACKETS
        DEBUG_SERIAL.print(F("[RX] Type: "));
        DEBUG_SERIAL.print(tlvHeaders[i].tlvType);
        DEBUG_SERIAL.print(F(", Len: "));
        DEBUG_SERIAL.println(length);
#endif
    }
}

// ============================================================================
// MESSAGE ROUTING
// ============================================================================

void MessageCenter::routeMessage(uint32_t type, const uint8_t *payload, uint32_t length)
{
    // Any received TLV resets the liveness timer
    lastHeartbeatMs_ = millis();
    heartbeatValid_ = true;
    // Non-heartbeat commands update the command timer
    if (type != SYS_HEARTBEAT)
    {
        lastCmdMs_ = millis();
    }

    // ---- State-based command gating ----
    SystemState state = SystemManager::getState();
    // Motor commands only accepted in RUNNING state
    bool allowMotorCmds = (state == SYS_STATE_RUNNING);
    // Config only accepted in IDLE state
    bool allowConfig = (state == SYS_STATE_IDLE);
    // SYS_CMD and SYS_HEARTBEAT accepted in any state
    // SYS_SET_PID accepted in IDLE or RUNNING
    bool allowPID = (state == SYS_STATE_IDLE || state == SYS_STATE_RUNNING);

    switch (type)
    {
    // ---- System messages (always accepted) ----
    case SYS_HEARTBEAT:
        if (length == sizeof(PayloadHeartbeat))
            handleHeartbeat((const PayloadHeartbeat *)payload);
        break;

    case SYS_CMD:
        if (length == sizeof(PayloadSysCmd))
            handleSysCmd((const PayloadSysCmd *)payload);
        break;

    case SYS_CONFIG:
        if (allowConfig && length == sizeof(PayloadSysConfig))
            handleSysConfig((const PayloadSysConfig *)payload);
        break;

    case SYS_SET_PID:
        if (allowPID && length == sizeof(PayloadSetPID))
            handleSetPID((const PayloadSetPID *)payload);
        break;

    // ---- DC motor commands (RUNNING only) ----
    case DC_ENABLE:
        if (length == sizeof(PayloadDCEnable))
            handleDCEnable((const PayloadDCEnable *)payload);
        break;

    case DC_SET_POSITION:
        if (allowMotorCmds && length == sizeof(PayloadDCSetPosition))
            handleDCSetPosition((const PayloadDCSetPosition *)payload);
        break;

    case DC_SET_VELOCITY:
        if (allowMotorCmds && length == sizeof(PayloadDCSetVelocity))
            handleDCSetVelocity((const PayloadDCSetVelocity *)payload);
        break;

    case DC_SET_PWM:
        if (allowMotorCmds && length == sizeof(PayloadDCSetPWM))
            handleDCSetPWM((const PayloadDCSetPWM *)payload);
        break;

    // ---- Stepper commands (RUNNING only) ----
    case STEP_ENABLE:
        if (length == sizeof(PayloadStepEnable))
            handleStepEnable((const PayloadStepEnable *)payload);
        break;

    case STEP_SET_PARAMS:
        if (allowMotorCmds && length == sizeof(PayloadStepSetParams))
            handleStepSetParams((const PayloadStepSetParams *)payload);
        break;

    case STEP_MOVE:
        if (allowMotorCmds && length == sizeof(PayloadStepMove))
            handleStepMove((const PayloadStepMove *)payload);
        break;

    case STEP_HOME:
        if (allowMotorCmds && length == sizeof(PayloadStepHome))
            handleStepHome((const PayloadStepHome *)payload);
        break;

    // ---- Servo commands ----
    case SERVO_ENABLE:
        if (length == sizeof(PayloadServoEnable))
            handleServoEnable((const PayloadServoEnable *)payload);
        break;

    case SERVO_SET:
        if (length == sizeof(PayloadServoSetSingle))
        {
            handleServoSet((const PayloadServoSetSingle *)payload);
        }
        else if (length == sizeof(PayloadServoSetBulk))
        {
            // Bulk servo update
            const PayloadServoSetBulk *b = (const PayloadServoSetBulk *)payload;
            if ((uint16_t)b->startChannel + b->count <= NUM_SERVO_CHANNELS)
            {
                ServoController::setMultiplePositionsUs(
                    b->startChannel, b->count, b->pulseUs);
            }
        }
        break;

    // ---- User I/O commands ----
    case IO_SET_LED:
        if (length == sizeof(PayloadSetLED))
            handleSetLED((const PayloadSetLED *)payload);
        break;

    case IO_SET_NEOPIXEL:
        if (length == sizeof(PayloadSetNeoPixel))
            handleSetNeoPixel((const PayloadSetNeoPixel *)payload);
        break;

    // ---- Magnetometer calibration (IDLE only) ----
    case SENSOR_MAG_CAL_CMD:
        if (allowConfig && length == sizeof(PayloadMagCalCmd))
            handleMagCalCmd((const PayloadMagCalCmd *)payload);
        break;

    default:
#ifdef DEBUG_TLV_PACKETS
        DEBUG_SERIAL.print(F("[RX] Unknown type: "));
        DEBUG_SERIAL.println(type);
#endif
        break;
    }
}

// ============================================================================
// SYSTEM MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleHeartbeat(const PayloadHeartbeat *payload)
{
    // Liveness timer already updated in routeMessage()
    (void)payload;
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[RX] Heartbeat ts="));
    DEBUG_SERIAL.println(payload->timestamp);
#endif
}

void MessageCenter::handleSysCmd(const PayloadSysCmd *payload)
{
    switch ((SysCmdType)payload->command)
    {
    case SYS_CMD_START:
        if (SystemManager::requestTransition(SYS_STATE_RUNNING))
        {
            DEBUG_SERIAL.println(F("[SYS] CMD_START → RUNNING OK"));
        }
        else
        {
            DEBUG_SERIAL.print(F("[SYS] CMD_START REJECTED — state="));
            DEBUG_SERIAL.println(static_cast<uint8_t>(SystemManager::getState()));
        }
        break;

    case SYS_CMD_STOP:
        if (SystemManager::requestTransition(SYS_STATE_IDLE))
        {
            disableAllActuators();
            DEBUG_SERIAL.println(F("[SYS] CMD_STOP → IDLE"));
        }
        break;

    case SYS_CMD_RESET:
        if (SystemManager::requestTransition(SYS_STATE_IDLE))
        {
            disableAllActuators();
            faultLatchFlags_ = 0;   // clear after successful reset so UI shows clean state
            DEBUG_SERIAL.println(F("[SYS] CMD_RESET → IDLE"));
        }
        else
        {
            DEBUG_SERIAL.print(F("[SYS] CMD_RESET REJECTED — state="));
            DEBUG_SERIAL.println(static_cast<uint8_t>(SystemManager::getState()));
        }
        break;

    case SYS_CMD_ESTOP:
        disableAllActuators();
        SystemManager::requestTransition(SYS_STATE_ESTOP);
        DEBUG_SERIAL.println(F("[SYS] CMD_ESTOP → ESTOP"));
        break;

    default:
        DEBUG_SERIAL.print(F("[SYS] unknown command="));
        DEBUG_SERIAL.println(payload->command);
        break;
    }
}

void MessageCenter::handleSysConfig(const PayloadSysConfig *payload)
{
    // IDLE state only — enforced by routeMessage()
    if (payload->neoPixelCount != 0)
        neoPixelCount_ = payload->neoPixelCount;

    if (payload->heartbeatTimeoutMs != 0)
        heartbeatTimeoutMs_ = payload->heartbeatTimeoutMs;

    // Apply direction mask changes (store; motors read this in sendSystemStatus)
    if (payload->motorDirChangeMask != 0)
    {
        motorDirMask_ = (motorDirMask_ & ~payload->motorDirChangeMask) |
                        (payload->motorDirMask & payload->motorDirChangeMask);
    }

    if (payload->resetOdometry)
    {
        RobotKinematics::reset(
            dcMotors[ODOM_LEFT_MOTOR].getPosition(),
            dcMotors[ODOM_RIGHT_MOTOR].getPosition());
    }
}

void MessageCenter::handleSetPID(const PayloadSetPID *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;

    DCMotor &motor = dcMotors[payload->motorId];

    if (payload->loopType == 0)
    {
        motor.setPositionPID(payload->kp, payload->ki, payload->kd);
    }
    else if (payload->loopType == 1)
    {
        motor.setVelocityPID(payload->kp, payload->ki, payload->kd);
    }
}

// ============================================================================
// DC MOTOR MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleDCEnable(const PayloadDCEnable *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;

    DCMotor &motor = dcMotors[payload->motorId];

    if (payload->mode == DC_MODE_DISABLED)
    {
        motor.disable();   // disable always allowed regardless of state
    }
    else
    {
        // Enable only permitted in IDLE or RUNNING — block in ERROR / ESTOP / INIT
        SystemState s = SystemManager::getState();
        if (s != SYS_STATE_IDLE && s != SYS_STATE_RUNNING) return;
        // Silently block when no battery present — see VBAT_MIN_PRESENT_V in config.h
        if (!SensorManager::isBatteryPresent()) return;
        motor.enable((DCMotorMode)payload->mode);
    }
}

void MessageCenter::handleDCSetPosition(const PayloadDCSetPosition *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;
    if (!SensorManager::isBatteryPresent()) return;

    DCMotor &motor = dcMotors[payload->motorId];
    // Auto-switch to position mode. Only call enable() on mode change so
    // the PID is not reset unnecessarily when updating an in-flight target.
    if (motor.getMode() != DC_MODE_POSITION)
        motor.enable(DC_MODE_POSITION);

    motor.setTargetPosition(payload->targetTicks);
}

void MessageCenter::handleDCSetVelocity(const PayloadDCSetVelocity *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;
    if (!SensorManager::isBatteryPresent()) return;

    DCMotor &motor = dcMotors[payload->motorId];
    if (motor.getMode() != DC_MODE_VELOCITY)
        motor.enable(DC_MODE_VELOCITY);

    motor.setTargetVelocity((float)payload->targetTicks);
}

void MessageCenter::handleDCSetPWM(const PayloadDCSetPWM *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;
    if (!SensorManager::isBatteryPresent()) return;

    DCMotor &motor = dcMotors[payload->motorId];
    if (motor.getMode() != DC_MODE_PWM)
        motor.enable(DC_MODE_PWM);

    motor.setDirectPWM(payload->pwm);
}

// ============================================================================
// STEPPER MOTOR MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleStepEnable(const PayloadStepEnable *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    if (payload->enable)
    {
        // Enable only permitted in IDLE or RUNNING
        SystemState st = SystemManager::getState();
        if (st != SYS_STATE_IDLE && st != SYS_STATE_RUNNING) return;
        if (!SensorManager::isBatteryPresent()) return;
        s->enable();
    }
    else
    {
        s->disable();   // disable always allowed
    }
}

void MessageCenter::handleStepSetParams(const PayloadStepSetParams *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    s->setMaxVelocity((uint16_t)payload->maxVelocity);
    s->setAcceleration((uint16_t)payload->acceleration);
}

void MessageCenter::handleStepMove(const PayloadStepMove *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    if (payload->moveType == STEP_MOVE_ABSOLUTE)
        s->moveToPosition(payload->target);
    else if (payload->moveType == STEP_MOVE_RELATIVE)
        s->moveSteps(payload->target);
}

void MessageCenter::handleStepHome(const PayloadStepHome *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    if (payload->homeVelocity > 0)
        s->setMaxVelocity((uint16_t)payload->homeVelocity);

    s->home(payload->direction);
}

// ============================================================================
// SERVO MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleServoEnable(const PayloadServoEnable *payload)
{
    // Enable only permitted in IDLE or RUNNING; disable always allowed
    SystemState st = SystemManager::getState();
    bool canEnable = (st == SYS_STATE_IDLE || st == SYS_STATE_RUNNING) &&
                     SensorManager::isBatteryPresent();

    if (payload->channel == 0xFF)
    {
        // All channels
        if (payload->enable)
        {
            if (!canEnable) return;
            servoEnabledMask_ = 0xFFFF;
            ServoController::enable();
        }
        else
        {
            servoEnabledMask_ = 0;
            ServoController::disable();
        }
    }
    else if (payload->channel < NUM_SERVO_CHANNELS)
    {
        if (payload->enable)
        {
            if (!canEnable) return;
            servoEnabledMask_ |= (uint16_t)(1u << payload->channel);
            if (!ServoController::isEnabled())
                ServoController::enable();
        }
        else
        {
            servoEnabledMask_ &= (uint16_t)~(1u << payload->channel);
            ServoController::setChannelOff(payload->channel);
        }
    }
}

void MessageCenter::handleServoSet(const PayloadServoSetSingle *payload)
{
    if (payload->channel >= NUM_SERVO_CHANNELS)
        return;

    ServoController::setPositionUs(payload->channel, payload->pulseUs);
}

// ============================================================================
// USER I/O MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleSetLED(const PayloadSetLED *payload)
{
    if (payload->ledId >= LED_COUNT)
        return;

    UserIO::setLED((LEDId)payload->ledId,
                   (LEDMode)payload->mode,
                   payload->brightness,
                   payload->periodMs);
}

void MessageCenter::handleSetNeoPixel(const PayloadSetNeoPixel *payload)
{
    if (payload->index == 0xFF)
    {
        // Set all pixels
        UserIO::setNeoPixelColor(payload->red, payload->green, payload->blue);
    }
    else if (payload->index < neoPixelCount_)
    {
        UserIO::setNeoPixelColor(payload->red, payload->green, payload->blue);
    }
}

// ============================================================================
// MAGNETOMETER CALIBRATION HANDLER
// ============================================================================

void MessageCenter::handleMagCalCmd(const PayloadMagCalCmd *payload)
{
    // IDLE state only — enforced by routeMessage()
    switch ((MagCalCmdType)payload->command)
    {
    case MAG_CAL_START:
        SensorManager::startMagCalibration();
        lastMagCalSendMs_ = 0; // Force immediate first status send
        break;

    case MAG_CAL_STOP:
        SensorManager::cancelMagCalibration();
        pendingMagCal_ = true; // Queue final status for next telemetry frame
        break;

    case MAG_CAL_SAVE:
        SensorManager::saveMagCalibration();
        pendingMagCal_ = true;
        break;

    case MAG_CAL_APPLY:
        SensorManager::applyMagCalibration(
            payload->offsetX, payload->offsetY, payload->offsetZ);
        pendingMagCal_ = true;
        break;

    case MAG_CAL_CLEAR:
        SensorManager::clearMagCalibration();
        pendingMagCal_ = true;
        break;

    default:
        break;
    }
}

// ============================================================================
// TELEMETRY APPENDERS
// ============================================================================
// Each function appends one TLV to the current frame via addTlvPacket().
// Must be called after beginFrame() and before sendFrame().

void MessageCenter::sendDCStatusAll()
{
    PayloadDCStatusAll payload;

    for (uint8_t i = 0; i < 4; i++)
    {
        DCMotorStatus &s = payload.motors[i];
        s.mode = (uint8_t)dcMotors[i].getMode();
        s.faultFlags = dcMotors[i].isEncoderFailed() ? 0x02 : 0;
        s.position = dcMotors[i].getPosition();
        s.velocity = (int32_t)dcMotors[i].getVelocity();
        s.targetPos = dcMotors[i].getTargetPosition();
        s.targetVel = (int32_t)dcMotors[i].getTargetVelocity();
        s.pwmOutput = dcMotors[i].getPWMOutput();
        s.currentMa = dcMotors[i].getCurrent();
        s.posKp = dcMotors[i].getPosKp();
        s.posKi = dcMotors[i].getPosKi();
        s.posKd = dcMotors[i].getPosKd();
        s.velKp = dcMotors[i].getVelKp();
        s.velKi = dcMotors[i].getVelKi();
        s.velKd = dcMotors[i].getVelKd();
    }

    addTlvPacket(&encodeDesc_, DC_STATUS_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendStepStatusAll()
{
    PayloadStepStatusAll payload;

    for (uint8_t i = 0; i < 4; i++)
    {
        StepperStatus &s = payload.steppers[i];
        const StepperMotor *sm = StepperManager::getStepper(i);
        if (!sm) continue;
        s.enabled = sm->isEnabled() ? 1 : 0;
        s.motionState = (uint8_t)sm->getState();
        s.limitHit = 0;
        s.reserved = 0;
        s.commandedCount = sm->getPosition();
        s.targetCount = sm->getTargetPosition();
        s.currentSpeed = sm->getCurrentSpeed();
        s.maxSpeed = sm->getMaxVelocity();
        s.acceleration = sm->getAcceleration();
    }

    addTlvPacket(&encodeDesc_, STEP_STATUS_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendServoStatusAll()
{
    PayloadServoStatusAll payload;

    payload.pca9685Connected = ServoController::isInitialized() ? 1 : 0;
    payload.pca9685Error = ServoController::getLastI2CError();
    payload.enabledMask = servoEnabledMask_;

    for (uint8_t i = 0; i < 16; i++)
    {
        if (servoEnabledMask_ & (1u << i))
        {
            payload.pulseUs[i] = ServoController::getPositionUs(i);
        }
        else
        {
            payload.pulseUs[i] = 0;
        }
    }

    addTlvPacket(&encodeDesc_, SERVO_STATUS_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendSensorIMU()
{
    if (!SensorManager::isIMUAvailable())
        return;

    PayloadSensorIMU payload;

    SensorManager::getQuaternion(
        payload.quatW, payload.quatX, payload.quatY, payload.quatZ);
    SensorManager::getEarthAcceleration(
        payload.earthAccX, payload.earthAccY, payload.earthAccZ);

    payload.rawAccX = SensorManager::getRawAccX();
    payload.rawAccY = SensorManager::getRawAccY();
    payload.rawAccZ = SensorManager::getRawAccZ();
    payload.rawGyroX = SensorManager::getRawGyrX();
    payload.rawGyroY = SensorManager::getRawGyrY();
    payload.rawGyroZ = SensorManager::getRawGyrZ();
    payload.magX = SensorManager::getRawMagX();
    payload.magY = SensorManager::getRawMagY();
    payload.magZ = SensorManager::getRawMagZ();

    payload.magCalibrated = SensorManager::isMagCalibrated() ? 1 : 0;
    payload.reserved = 0;
    payload.timestamp = micros();

    addTlvPacket(&encodeDesc_, SENSOR_IMU, sizeof(payload), &payload);
}

void MessageCenter::sendSensorRange(bool lidarOnly)
{
    if (lidarOnly)
    {
        // Lidar sensors (50 Hz path)
        for (uint8_t i = 0; i < SensorManager::getLidarConfiguredCount(); i++)
        {
            PayloadSensorRange p;
            p.sensorId = i;
            p.sensorType = 1; // lidar
            p.reserved = 0;
            p.reserved2 = 0;
            p.timestamp = micros();
            if (SensorManager::isLidarFound(i))
            {
                p.status = 0; // valid
                p.distanceMm = SensorManager::getLidarDistanceMm(i);
            }
            else
            {
                p.status = 3; // not installed
                p.distanceMm = 0;
            }
            addTlvPacket(&encodeDesc_, SENSOR_RANGE, sizeof(p), &p);
        }
    }
    else
    {
        // Ultrasonic sensors (10 Hz path)
        for (uint8_t i = 0; i < SensorManager::getUltrasonicConfiguredCount(); i++)
        {
            PayloadSensorRange p;
            p.sensorId = i;
            p.sensorType = 0; // ultrasonic
            p.reserved = 0;
            p.reserved2 = 0;
            p.timestamp = micros();
            if (SensorManager::isUltrasonicFound(i))
            {
                p.status = 0; // valid
                p.distanceMm = SensorManager::getUltrasonicDistanceMm(i);
            }
            else
            {
                p.status = 3; // not installed
                p.distanceMm = 0;
            }
            addTlvPacket(&encodeDesc_, SENSOR_RANGE, sizeof(p), &p);
        }
    }
}

void MessageCenter::sendSensorKinematics()
{
    RobotKinematics::update(
        dcMotors[ODOM_LEFT_MOTOR].getPosition(),
        dcMotors[ODOM_RIGHT_MOTOR].getPosition(),
        dcMotors[ODOM_LEFT_MOTOR].getVelocity(),
        dcMotors[ODOM_RIGHT_MOTOR].getVelocity());

    PayloadSensorKinematics payload;
    payload.x = RobotKinematics::getX();
    payload.y = RobotKinematics::getY();
    payload.theta = RobotKinematics::getTheta();
    payload.vx = RobotKinematics::getVx();
    payload.vy = RobotKinematics::getVy();
    payload.vTheta = RobotKinematics::getVTheta();
    payload.timestamp = micros();

    addTlvPacket(&encodeDesc_, SENSOR_KINEMATICS, sizeof(payload), &payload);
}

void MessageCenter::sendVoltageData()
{
    PayloadSensorVoltage payload;

    payload.batteryMv = (uint16_t)(SensorManager::getBatteryVoltage() * 1000.0f);
    payload.rail5vMv = (uint16_t)(SensorManager::get5VRailVoltage() * 1000.0f);
    payload.servoRailMv = (uint16_t)(SensorManager::getServoVoltage() * 1000.0f);
    payload.reserved = 0;

    addTlvPacket(&encodeDesc_, SENSOR_VOLTAGE, sizeof(payload), &payload);
}

void MessageCenter::sendIOStatus()
{
    // Variable-length payload: 12 bytes fixed + 3 bytes per NeoPixel
    uint8_t buf[12 + 3 * NEOPIXEL_COUNT];
    memset(buf, 0, sizeof(buf));

    PayloadIOStatus *p = (PayloadIOStatus *)buf;
    p->buttonMask = UserIO::getButtonStates();
    for (uint8_t i = 0; i < 5; i++) {
        p->ledBrightness[i] = UserIO::getLEDBrightness(i);
    }
    p->reserved = 0;
    p->timestamp = millis();
    // NeoPixel RGB bytes appended after fixed struct

    uint8_t sendLen = 12 + (uint8_t)(neoPixelCount_ * 3);
    if (sendLen > sizeof(buf))
        sendLen = sizeof(buf);

    addTlvPacket(&encodeDesc_, IO_STATUS, sendLen, buf);
}

void MessageCenter::sendSystemStatus()
{
    PayloadSystemStatus payload;
    memset(&payload, 0, sizeof(payload));

    payload.firmwareMajor = (FIRMWARE_VERSION >> 24) & 0xFF;
    payload.firmwareMinor = (FIRMWARE_VERSION >> 16) & 0xFF;
    payload.firmwarePatch = (FIRMWARE_VERSION >> 8) & 0xFF;
    payload.state = (uint8_t)SystemManager::getState();
    payload.uptimeMs = millis();
    payload.lastRxMs = getTimeSinceHeartbeat();
    payload.lastCmdMs = millis() - lastCmdMs_;
    payload.batteryMv = (uint16_t)(SensorManager::getBatteryVoltage() * 1000.0f);
    payload.rail5vMv = (uint16_t)(SensorManager::get5VRailVoltage() * 1000.0f);

    // Error flags — live conditions OR latched fault cause
    payload.errorFlags = faultLatchFlags_;   // always show what triggered ERROR
    if (SensorManager::isBatteryLow())
        payload.errorFlags |= ERR_UNDERVOLTAGE;
    if (SensorManager::isBatteryOvervoltage())
        payload.errorFlags |= ERR_OVERVOLTAGE;
    // LIVENESS_LOST: live check only valid in RUNNING; latch covers the ERROR case
    if (!heartbeatValid_ && SystemManager::getState() == SYS_STATE_RUNNING)
        payload.errorFlags |= ERR_LIVENESS_LOST;
    if (ServoController::hasI2CError())
        payload.errorFlags |= ERR_I2C_ERROR;
#if IMU_ENABLED
    if (!SensorManager::isIMUAvailable())
        payload.errorFlags |= ERR_IMU_ERROR;
#endif
    // Per-motor encoder stall: any failed motor sets the shared flag
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
        if (dcMotors[i].isEncoderFailed()) {
            payload.errorFlags |= ERR_ENCODER_FAIL;
            break;
        }
    }

    // Attached sensors bitmask: bit0=IMU, bit1=Lidar, bit2=Ultrasonic
    if (SensorManager::isIMUAvailable())
        payload.attachedSensors |= 0x01;
    if (SensorManager::getLidarCount() > 0)
        payload.attachedSensors |= 0x02;
    if (SensorManager::getUltrasonicCount() > 0)
        payload.attachedSensors |= 0x04;

    payload.freeSram = getFreeRAM();
    payload.loopTimeAvgUs = loopTimeAvgUs_;
    payload.loopTimeMaxUs = loopTimeMaxUs_;
    payload.uartRxErrors = uartRxErrors_;
    payload.motorDirMask = motorDirMask_;
    payload.neoPixelCount = neoPixelCount_;
    payload.heartbeatTimeoutMs = heartbeatTimeoutMs_;
    payload.limitSwitchMask = 0;

    // 0xFF = no home limit GPIO configured for this stepper
    memset(payload.stepperHomeLimitGpio, 0xFF, sizeof(payload.stepperHomeLimitGpio));

    addTlvPacket(&encodeDesc_, SYS_STATUS, sizeof(payload), &payload);
}

void MessageCenter::sendMagCalStatus()
{
    const MagCalData &cal = SensorManager::getMagCalData();

    PayloadMagCalStatus payload;
    payload.state = (uint8_t)cal.state;
    payload.sampleCount = cal.sampleCount;
    payload.reserved = 0;
    payload.minX = cal.minX;
    payload.maxX = cal.maxX;
    payload.minY = cal.minY;
    payload.maxY = cal.maxY;
    payload.minZ = cal.minZ;
    payload.maxZ = cal.maxZ;
    payload.offsetX = cal.offsetX;
    payload.offsetY = cal.offsetY;
    payload.offsetZ = cal.offsetZ;
    payload.savedToEeprom = cal.savedToEeprom ? 1 : 0;
    memset(payload.reserved2, 0, sizeof(payload.reserved2));

    addTlvPacket(&encodeDesc_, SENSOR_MAG_CAL_STATUS, sizeof(payload), &payload);
}

// ============================================================================
// HELPERS
// ============================================================================

void MessageCenter::disableAllActuators()
{
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++)
    {
        if (dcMotors[i].isEnabled())
            dcMotors[i].disable();
    }
    StepperManager::emergencyStopAll();
    ServoController::disable();
    servoEnabledMask_ = 0;
}

void MessageCenter::latchFaultFlags(uint8_t flags)
{
    // OR-in the new flags so multiple faults accumulate.
    // Called from SafetyManager ISR — volatile write is atomic on AVR for uint8_t.
    faultLatchFlags_ |= flags;
}

uint16_t MessageCenter::getFreeRAM()
{
    extern int __heap_start, *__brkval;
    uint8_t v;
    const uint8_t *stackPtr = &v;
    const uint8_t *heapEnd = (__brkval == 0)
                                 ? (const uint8_t *)&__heap_start
                                 : (const uint8_t *)__brkval;
    if (stackPtr > heapEnd)
    {
        return (uint16_t)(stackPtr - heapEnd);
    }
    return 0;
}
