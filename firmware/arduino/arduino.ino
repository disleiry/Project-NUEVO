/**
 * @file arduino.ino
 * @brief Main firmware for Arduino Mega 2560 educational robotics platform
 * @version 0.8.0
 *
 * Educational robotics platform firmware for MAE 162 course.
 * Provides real-time motor control, sensor integration, and
 * communication with Raspberry Pi 5 via TLV protocol over UART.
 *
 * Two-tier scheduling architecture:
 *
 *   Hard real-time (ISR-driven — unaffected by loop() blocking):
 *     TIMER1_OVF_vect  200 Hz  DC motor PID always
 *                      100 Hz  button reads + safety check (every 2nd tick)
 *     TIMER3_OVF_vect  10 kHz  Stepper pulse generation (StepperManager)
 *     TIMER4_OVF_vect  100 Hz  IMU/Lidar/Voltage dispatch (SensorManager)
 *
 *   Soft real-time (millis-based, runs in loop() with interrupts enabled):
 *     taskUART         100 Hz  UART RX/TX — loop() keeps USART2_RX_vect alive;
 *                              sei() inside the ISR would allow TIMER4 to
 *                              preempt and deadlock on the shared I2C bus
 *     taskUserIO        20 Hz  LED animations, NeoPixel status
 *
 * Initialization Order (setup):
 *  1. Debug serial (Serial0)
 *  2. Scheduler (millis-based soft scheduler)
 *  3. MessageCenter (Serial2 + TLV codec)
 *  4. SensorManager (I2C, ADC)
 *  5. UserIO (GPIO, NeoPixel)
 *  6. ServoController (PCA9685 via I2C)
 *  7. StepperManager (Timer3 — also starts stepper ISR)
 *  8. DC Motors (PWM pins, encoder counters)
 *  9. Attach encoder ISRs
 * 10. Register soft-scheduler tasks
 * 11. ISRScheduler::init() — LAST: starts Timer1 and Timer4 ISRs
 *
 * Main Loop:
 * - Scheduler::tick() executes highest-priority ready soft task
 */

// ============================================================================
// INCLUDES
// ============================================================================

// Core configuration
#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"
#include "src/ISRScheduler.h"
#include "src/SystemManager.h"

// Communication
#include "src/modules/MessageCenter.h"
#include "src/modules/SafetyManager.h"
#include "src/messages/TLV_Payloads.h"

// DC motor control
#include "src/modules/EncoderCounter.h"
#include "src/modules/VelocityEstimator.h"
#include "src/drivers/DCMotor.h"

// Stepper and servo control
#include "src/modules/StepperManager.h"
#include "src/drivers/StepperMotor.h"
#include "src/drivers/ServoController.h"

// Sensors and user I/O
#include "src/modules/SensorManager.h"
#include "src/modules/UserIO.h"
#include "src/drivers/IMUDriver.h"
#include "src/drivers/NeoPixelDriver.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Encoder instances (2x or 4x mode per config.h)
#if ENCODER_1_MODE == ENCODER_2X
EncoderCounter2x encoder1;
#else
EncoderCounter4x encoder1;
#endif

#if ENCODER_2_MODE == ENCODER_2X
EncoderCounter2x encoder2;
#else
EncoderCounter4x encoder2;
#endif

#if ENCODER_3_MODE == ENCODER_2X
EncoderCounter2x encoder3;
#else
EncoderCounter4x encoder3;
#endif

#if ENCODER_4_MODE == ENCODER_2X
EncoderCounter2x encoder4;
#else
EncoderCounter4x encoder4;
#endif

// Velocity estimators (edge-time algorithm)
EdgeTimeVelocityEstimator velocityEst1;
EdgeTimeVelocityEstimator velocityEst2;
EdgeTimeVelocityEstimator velocityEst3;
EdgeTimeVelocityEstimator velocityEst4;

// Motor controller arrays
DCMotor dcMotors[NUM_DC_MOTORS];

// ============================================================================
// HARD REAL-TIME ISR — TIMER1_OVF_vect (200 Hz / 100 Hz)
// ============================================================================

/**
 * @brief Timer1 Overflow ISR — DC Motor PID (200 Hz) + safety check (100 Hz)
 *
 * Fires at 200 Hz (5 ms period).
 *   - Every tick:       DC motor PID update for all enabled motors.
 *   - Every other tick: button reads + hard real-time safety check.
 *
 * WHY UART IS NOT HERE
 * ─────────────────────
 * AVR disables global interrupts on ISR entry. At 1 Mbps a byte arrives every
 * 10 µs; processIncoming() + sendTelemetry() take 0.5–2 ms — long enough to
 * overflow the 2-byte hardware UART receive register and corrupt TLV frames.
 *
 * Using sei() to re-enable interrupts inside the ISR is NOT safe: it allows
 * TIMER4_OVF_vect to preempt us. Both TIMER1 (ServoController) and TIMER4
 * (SensorManager IMU) use the I2C / Wire bus. Concurrent Wire access deadlocks
 * inside Wire's "while(twi_state != TWI_READY)" spin, freezing the firmware.
 *
 * Solution: UART comms run in taskUART() via the soft scheduler in loop().
 * loop() executes with global interrupts ENABLED, so USART2_RX_vect fills the
 * 64-byte HardwareSerial ring buffer normally. As long as no loop() task
 * disables interrupts for > 640 µs, no bytes are lost. The longest known
 * interrupt-disable window is NeoPixel show() at ~38 µs — 16× safe margin.
 *
 * Timer1 is configured in Fast PWM mode 14 by ISRScheduler::init().
 * OC1A (pin 11) simultaneously drives LED_RED hardware PWM in Rev A.
 */
ISR(TIMER1_OVF_vect) {
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_PID_LOOP, HIGH);
#endif

  // ── 200 Hz: DC motor PID ────────────────────────────────────────────────
  for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
    dcMotors[i].update();
  }

  // ── 200 Hz: Green LED heartbeat (1 Hz toggle = 2 s blink period) ────────
  // Toggles every 200 PID ticks, giving a visible "scheduler alive" indicator.
  static uint8_t greenCounter = 0;
  static bool    greenState   = false;
  if (++greenCounter >= 200) {
    greenCounter = 0;
    greenState   = !greenState;
    digitalWrite(PIN_LED_GREEN, greenState ? HIGH : LOW);
  }

  // ── 100 Hz: buttons + safety check ──────────────────────────────────────
  static bool safetyTick = false;
  safetyTick = !safetyTick;
  if (safetyTick) {
    // Button reads at 100 Hz — fast GPIO only, no millis/delay
    UserIO::readButtons();

    // Centralised hard RT safety check (heartbeat + battery faults → ERROR)
    SafetyManager::check();
  }

#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_PID_LOOP, LOW);
#endif
}

// ============================================================================
// SOFT TASK — taskUART (100 Hz, millis-based)
// ============================================================================

/**
 * @brief UART RX/TX task — runs in loop() so USART2_RX_vect stays enabled.
 *
 * Do NOT move into TIMER1_OVF_vect (see ISR comment above for explanation).
 * Registered at prior0 (highest) so it preempts all other soft tasks.
 */ 
void taskUART() {
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_TASK, HIGH);
#endif

  uint32_t t0 = micros();

#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_RX, HIGH);
#endif
  MessageCenter::processIncoming();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_RX, LOW);
  digitalWrite(DEBUG_PIN_UART_TX, HIGH);
#endif
  MessageCenter::sendTelemetry();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_TX, LOW);
#endif

  uint32_t elapsed = micros() - t0;
  MessageCenter::recordLoopTime(elapsed);

#if DEBUG_PINS_ENABLED
  // A9 (UART_LATE): pulse when wall-clock > 10 ms. This fires due to ISR
  // preemption inflation, NOT actual UART slowness — the PID loop is unaffected.
  if (elapsed > UART_TASK_BUDGET_US) {
    digitalWrite(DEBUG_PIN_UART_LATE, HIGH);
    digitalWrite(DEBUG_PIN_UART_LATE, LOW);
  }
  digitalWrite(DEBUG_PIN_UART_TASK, LOW);
#endif
}

// ============================================================================
// SOFT TASK — taskUserIO (20 Hz, millis-based)
// ============================================================================

/**
 * @brief User I/O updates (20 Hz, soft scheduler)
 *
 * Runs LED animations (blink, breathe, RED battery warnings).
 * Reads limit switches.
 * Drives NeoPixel state animations via UserIO::updateNeoPixelAnimation()
 * (INIT=yellow, IDLE=breathing emerald, RUNNING=rainbow, ERROR=flashing red).
 * Button reads are handled at 100 Hz inside TIMER1_OVF_vect via readButtons().
 */
void taskUserIO() {
  // LED animations + limit switches + NeoPixel state animation
  UserIO::update();

  // RED LED battery warnings (independent of NeoPixel system-state animation)
  if (SensorManager::isBatteryOvervoltage()) {
    UserIO::setLED(LED_RED, LED_BLINK, 255, 250);   // Fast blink — wrong battery/charger
  } else if (SensorManager::isBatteryCritical()) {
    UserIO::setLED(LED_RED, LED_BLINK, 255, 250);   // Fast blink — shutdown imminent
  } else if (SensorManager::isBatteryLow()) {
    UserIO::setLED(LED_RED, LED_BLINK, 255, 1000);  // Slow blink — low warning
  }
}


void systemInfo() {
  // ── System state ────────────────────────────────────────────────────────────
  SystemState state = SystemManager::getState();
  DEBUG_SERIAL.print(F("[state] "));
  switch (state) {
    case SystemState::SYS_STATE_INIT:    DEBUG_SERIAL.println(F("INIT"));    break;
    case SystemState::SYS_STATE_IDLE:    DEBUG_SERIAL.println(F("IDLE"));    break;
    case SystemState::SYS_STATE_RUNNING: DEBUG_SERIAL.println(F("RUNNING")); break;
    case SystemState::SYS_STATE_ERROR:   DEBUG_SERIAL.println(F("ERROR"));   break;
    case SystemState::SYS_STATE_ESTOP:   DEBUG_SERIAL.println(F("E-STOP"));  break;
    default:
      DEBUG_SERIAL.print(F("UNKNOWN("));
      DEBUG_SERIAL.print(static_cast<uint8_t>(state));
      DEBUG_SERIAL.println(')');
      break;
  }

  // ── Power / sensors ─────────────────────────────────────────────────────────
  float vbat  = SensorManager::getBatteryVoltage();
  float v5    = SensorManager::get5VRailVoltage();
  float vserv = SensorManager::getServoVoltage();
  DEBUG_SERIAL.print(F("[power]  VBAT="));
  if (vbat < 0.1f) {
    DEBUG_SERIAL.print(F("--.- (no battery)"));
  } else {
    DEBUG_SERIAL.print(vbat, 2);
    DEBUG_SERIAL.print('V');
    if      (SensorManager::isBatteryOvervoltage()) DEBUG_SERIAL.print(F(" OVERVOLTAGE!"));
    else if (SensorManager::isBatteryCritical())    DEBUG_SERIAL.print(F(" CRITICAL!"));
    else if (SensorManager::isBatteryLow())         DEBUG_SERIAL.print(F(" LOW"));
  }
  DEBUG_SERIAL.print(F("  5V="));
  DEBUG_SERIAL.print(v5, 2);
  DEBUG_SERIAL.print(F("V  SRV="));
  DEBUG_SERIAL.print(vserv, 2);
  DEBUG_SERIAL.println('V');

  // ── Warnings / errors (always shown, regardless of system state) ─────────
  // Build the same flags as sendSystemStatus() so they match the UI
  uint8_t flags = ERR_NONE;
  if (SensorManager::isBatteryLow())         flags |= ERR_UNDERVOLTAGE;
  if (SensorManager::isBatteryOvervoltage()) flags |= ERR_OVERVOLTAGE;
  // LIVENESS_LOST only meaningful in RUNNING — in IDLE, no host connection expected
  if (!MessageCenter::isHeartbeatValid() && state == SystemState::SYS_STATE_RUNNING)
                                             flags |= ERR_LIVENESS_LOST;
  if (ServoController::hasI2CError())        flags |= ERR_I2C_ERROR;
#if IMU_ENABLED
  if (!SensorManager::isIMUAvailable())      flags |= ERR_IMU_ERROR;
#endif
  for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
    if (dcMotors[i].isEncoderFailed()) { flags |= ERR_ENCODER_FAIL; break; }
  }

  if (flags == ERR_NONE) {
    DEBUG_SERIAL.println(F("[flags]  OK"));
  } else {
    DEBUG_SERIAL.print(F("[flags]  "));
    bool first = true;
    auto pf = [&](const __FlashStringHelper* s) {
      if (!first) DEBUG_SERIAL.print(F(", "));
      DEBUG_SERIAL.print(s);
      first = false;
    };
    if (flags & ERR_UNDERVOLTAGE)  pf(F("UNDERVOLTAGE"));
    if (flags & ERR_OVERVOLTAGE)   pf(F("OVERVOLTAGE"));
    if (flags & ERR_ENCODER_FAIL)  pf(F("ENCODER_FAIL"));
    if (flags & ERR_I2C_ERROR)     pf(F("I2C_ERROR"));
    if (flags & ERR_IMU_ERROR)     pf(F("IMU_ERROR"));
    if (flags & ERR_LIVENESS_LOST) pf(F("LIVENESS_LOST"));
    DEBUG_SERIAL.println();
  }

  // ── Heartbeat / liveness ─────────────────────────────────────────────────
  DEBUG_SERIAL.print(F("[hbeat]  valid="));
  DEBUG_SERIAL.print(MessageCenter::isHeartbeatValid() ? F("YES") : F("NO"));
  DEBUG_SERIAL.print(F("  age="));
  DEBUG_SERIAL.print(MessageCenter::getTimeSinceHeartbeat());
  DEBUG_SERIAL.println(F("ms"));

  // ── Loop timing + SRAM + UART ────────────────────────────────────────────
  DEBUG_SERIAL.print(F("[perf]   uart avg="));
  DEBUG_SERIAL.print(MessageCenter::getLoopTimeAvgUs());
  DEBUG_SERIAL.print(F("us max="));
  DEBUG_SERIAL.print(MessageCenter::getLoopTimeMaxUs());
  DEBUG_SERIAL.print(F("us(incl.ISR)  uart_errs="));
  DEBUG_SERIAL.print(MessageCenter::getUartRxErrors());
  DEBUG_SERIAL.print(F("  sram="));
  DEBUG_SERIAL.print(MessageCenter::getFreeRAM());
  DEBUG_SERIAL.println(F("B"));

  // ── DC motors (all 4) ───────────────────────────────────────────────────
  DEBUG_SERIAL.print(F("[motors] "));
  for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
    DEBUG_SERIAL.print('M');
    DEBUG_SERIAL.print(i + 1);
    DEBUG_SERIAL.print(dcMotors[i].isEnabled() ? F(":ON") : F(":off"));
    DEBUG_SERIAL.print(F(" pos="));
    DEBUG_SERIAL.print(dcMotors[i].getPosition());
    DEBUG_SERIAL.print(F(" vel="));
    DEBUG_SERIAL.print((int16_t)dcMotors[i].getVelocity());
    DEBUG_SERIAL.print(F(" pwm="));
    DEBUG_SERIAL.print(dcMotors[i].getPWMOutput());
    if (dcMotors[i].isEncoderFailed()) DEBUG_SERIAL.print(F(" ENC_FAIL!"));
    if (i < NUM_DC_MOTORS - 1) DEBUG_SERIAL.print(F("  "));
  }
  DEBUG_SERIAL.println();
}

// ============================================================================
// ENCODER ISR TRAMPOLINES
// ============================================================================

/**
 * @brief Encoder ISR wrappers
 *
 * These are minimal ISR wrappers that forward calls to encoder objects.
 * Encoder ISRs must be global functions (not class methods) to use with
 * attachInterrupt().
 */

void encoderISR_M1_A() {
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, HIGH);
#endif
  encoder1.onInterruptA();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
#endif
}

void encoderISR_M1_B() {
  encoder1.onInterruptB();
}

void encoderISR_M2_A() {
  encoder2.onInterruptA();
}

void encoderISR_M2_B() {
  encoder2.onInterruptB();
}

// M3/M4 encoder pins (A14, A15, 11, 12) are on PCINT buses, not hardware
// INT pins. attachInterrupt() does not work for them — they require PCINT
// ISR setup (ISR(PCINT1_vect) / ISR(PCINT0_vect)), which is not yet
// implemented. For now these are declared but unused.
void encoderISR_M3() {
  encoder3.onInterruptA();
}

void encoderISR_M4() {
  encoder4.onInterruptA();
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  // ------------------------------------------------------------------------
  // Initialize SystemManager (sets INIT state before any module starts)
  // ------------------------------------------------------------------------
  SystemManager::init();

  // ------------------------------------------------------------------------
  // Initialize Debug Serial (USB)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
  while (!DEBUG_SERIAL && millis() < 2000) {
    ; // Wait for serial port to connect (up to 2 seconds)
  }

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F("  MAE 162 Robot Firmware v0.8.0"));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println();

  // ------------------------------------------------------------------------
  // Initialize Soft Scheduler (millis-based)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing soft scheduler..."));
  Scheduler::init();

#if DEBUG_PINS_ENABLED
  // Configure debug pins for oscilloscope timing measurement
  pinMode(DEBUG_PIN_ENCODER_ISR, OUTPUT);  // A7
  pinMode(DEBUG_PIN_STEPPER_ISR, OUTPUT);  // A8
  pinMode(DEBUG_PIN_UART_LATE,   OUTPUT);  // A9
  pinMode(DEBUG_PIN_PID_LOOP,    OUTPUT);  // A10
  pinMode(DEBUG_PIN_UART_TASK,   OUTPUT);  // A11
  pinMode(DEBUG_PIN_UART_RX,     OUTPUT);  // A12
  pinMode(DEBUG_PIN_UART_TX,     OUTPUT);  // A13
  digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
  digitalWrite(DEBUG_PIN_STEPPER_ISR, LOW);
  digitalWrite(DEBUG_PIN_UART_LATE,   LOW);
  digitalWrite(DEBUG_PIN_PID_LOOP,    LOW);
  digitalWrite(DEBUG_PIN_UART_TASK,   LOW);
  digitalWrite(DEBUG_PIN_UART_RX,     LOW);
  digitalWrite(DEBUG_PIN_UART_TX,     LOW);
  DEBUG_SERIAL.println(F("[Setup] Debug pins configured (A7-A13)"));
#endif

  // ------------------------------------------------------------------------
  // Initialize Communication (UART + TLV Protocol)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing UART communication..."));
  MessageCenter::init();

  // ------------------------------------------------------------------------
  // Initialize Sensors (I2C, ADC)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing sensors..."));
  SensorManager::init();

  // ------------------------------------------------------------------------
  // Initialize User I/O (LEDs, Buttons, NeoPixels)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing user I/O..."));
  UserIO::init();

  // ------------------------------------------------------------------------
  // Initialize Servo Controller (PCA9685)
  // ------------------------------------------------------------------------
#if SERVO_CONTROLLER_ENABLED
  DEBUG_SERIAL.println(F("[Setup] Initializing servo controller..."));
  ServoController::init();
  DEBUG_SERIAL.println(F("  - PCA9685 initialized (50Hz PWM)"));
#endif

  // ------------------------------------------------------------------------
  // Initialize Stepper Motors (Timer3 @ 10kHz)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing stepper motors..."));
  StepperManager::init();

  // ------------------------------------------------------------------------
  // Initialize DC Motors and Encoders
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing DC motors and encoders..."));

  // Calculate counts per revolution based on encoder mode
  uint16_t countsPerRev = ENCODER_PPR * encoder1.getResolutionMultiplier();
  DEBUG_SERIAL.print(F("  - Encoder resolution: "));
  DEBUG_SERIAL.print(countsPerRev);
  DEBUG_SERIAL.println(F(" counts/rev"));

  // Motor 1
  encoder1.init(PIN_M1_ENC_A, PIN_M1_ENC_B, ENCODER_1_DIR_INVERTED);
  velocityEst1.init(countsPerRev);
  velocityEst1.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst1.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[0].init(0, &encoder1, &velocityEst1, DC_MOTOR_1_DIR_INVERTED);
  dcMotors[0].setPins(PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2);
  dcMotors[0].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[0].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 1 initialized"));

  // Motor 2
  encoder2.init(PIN_M2_ENC_A, PIN_M2_ENC_B, ENCODER_2_DIR_INVERTED);
  velocityEst2.init(countsPerRev);
  velocityEst2.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst2.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[1].init(1, &encoder2, &velocityEst2, DC_MOTOR_2_DIR_INVERTED);
  dcMotors[1].setPins(PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2);
  dcMotors[1].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[1].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 2 initialized"));

  // Motor 3
  encoder3.init(PIN_M3_ENC_A, PIN_M3_ENC_B, ENCODER_3_DIR_INVERTED);
  velocityEst3.init(countsPerRev);
  velocityEst3.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst3.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[2].init(2, &encoder3, &velocityEst3, DC_MOTOR_3_DIR_INVERTED);
  dcMotors[2].setPins(PIN_M3_EN, PIN_M3_IN1, PIN_M3_IN2);
  dcMotors[2].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[2].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 3 initialized"));

  // Motor 4
  encoder4.init(PIN_M4_ENC_A, PIN_M4_ENC_B, ENCODER_4_DIR_INVERTED);
  velocityEst4.init(countsPerRev);
  velocityEst4.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst4.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[3].init(3, &encoder4, &velocityEst4, DC_MOTOR_4_DIR_INVERTED);
  dcMotors[3].setPins(PIN_M4_EN, PIN_M4_IN1, PIN_M4_IN2);
  dcMotors[3].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[3].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 4 initialized"));

  // ------------------------------------------------------------------------
  // Attach Encoder Interrupts
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Attaching encoder interrupts..."));

  // M1: pins 2 (INT0) and 3 (INT1) — both on hardware INT, full 4x
  attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_B), encoderISR_M1_B, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 1 encoder ISRs attached (4x: A+B)"));

  // M2: pins 18 (INT5) and 19 (INT4) — both on hardware INT, full 4x
  attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), encoderISR_M2_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_B), encoderISR_M2_B, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 2 encoder ISRs attached (4x: A+B)"));

  // M3/M4: encoder pins are on PCINT buses — attachInterrupt() silently
  // ignores non-INT pins on Mega. PCINT ISRs not yet implemented; these
  // motors have no encoder counting until that is added.
  DEBUG_SERIAL.println(F("  - Motor 3/4 encoders: PCINT not implemented (no counting)"));

  // ------------------------------------------------------------------------
  // Register Soft Scheduler Tasks
  // DC Motor PID and sensors are hard real-time ISR tasks (Timer1 @ 200/100 Hz,
  // Timer4 @ 100 Hz). UART comms must run in loop() to keep USART2_RX_vect
  // alive — see taskUART and TIMER1_OVF_vect comments for explanation.
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Registering soft scheduler tasks..."));

  // UART comms: priority 0 (highest) — must preempt all other soft tasks
  int8_t taskId = Scheduler::registerTask(taskUART, 10, 0);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - UART: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.println(F(" @ 10ms (100Hz)"));
  }

  taskId = Scheduler::registerTask(taskUserIO, 1000 / USER_IO_FREQ_HZ, 1);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - User I/O: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / USER_IO_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms"));
  }

  taskId = Scheduler::registerTask(systemInfo, 1000, 2);  // 1 Hz system info
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - System Info: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.println(F(" @ 1000ms"));
  }
  // ------------------------------------------------------------------------
  // Transition to IDLE (all hardware initialized, ready to receive commands)
  // ------------------------------------------------------------------------
  SystemManager::requestTransition(SYS_STATE_IDLE);
  DEBUG_SERIAL.println(F("[Setup] System state → IDLE"));

  // ------------------------------------------------------------------------
  // Start Hard Real-Time ISRs (MUST be last — ISRs fire immediately)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Starting hard real-time ISRs (Timer1 + Timer4)..."));
  ISRScheduler::init();

  // ------------------------------------------------------------------------
  // Setup Complete
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("[Setup] Initialization complete!"));
  DEBUG_SERIAL.println(F("  Hard RT: PID@200Hz, Safety@100Hz, Sensors@100Hz (Timer1/3/4)"));
  DEBUG_SERIAL.println(F("  Soft RT: UART@100Hz, UserIO@20Hz, SystemInfo@1Hz (millis-based)"));
  DEBUG_SERIAL.println(F("[Setup] Entered main loop."));
  DEBUG_SERIAL.println();
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

void loop() {
  // Drain HW UART ring buffer (64 bytes) into software buffer (1024 bytes) on
  // every loop() iteration (~67 kHz).  This prevents ring-buffer overflow when
  // the RPi sends back-to-back frames (heartbeat + command ≥ 96 bytes at 1 Mbps).
  // processIncoming() (100 Hz) then decodes from the software buffer.
  MessageCenter::drainUart();

  // Execute highest-priority ready task
  Scheduler::tick();
}
