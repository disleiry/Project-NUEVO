# Arduino Firmware

This directory contains the Arduino Mega 2560 firmware for the MAE 162 robot platform. The firmware handles all real-time control: motor PWM, encoder counting, sensor reading, and communication with the Raspberry Pi over a custom TLV (Type-Length-Value) protocol.

## Directory Structure

```
firmware/
├── arduino/                  # Main firmware sketch (upload this to the robot)
│   ├── arduino.ino           # Entry point (setup/loop)
│   └── src/                  # All firmware source code
│       ├── config.h          # Compile-time configuration (edit this first)
│       ├── pins.h            # All GPIO pin definitions (swap for Rev A ↔ Rev B)
│       ├── ISRScheduler.h/cpp  # Hard real-time ISR timer setup (Timer1 + Timer4)
│       ├── Scheduler.h/cpp   # Soft millis-based cooperative scheduler (loop-driven)
│       ├── drivers/          # Hardware abstractions
│       │   ├── DCMotor       # H-bridge PWM + direction control
│       │   ├── StepperMotor  # STEP/DIR/ENABLE stepper driver interface
│       │   ├── ServoController  # PCA9685 I2C servo controller
│       │   ├── UARTDriver    # TLV-framed serial comms (RPi link)
│       │   ├── NeoPixelDriver   # WS2812B RGB status LED
│       │   ├── IMUDriver     # ICM-20948 9-DoF IMU
│       │   ├── LidarDriver   # Garmin LIDAR-Lite v4 (I2C)
│       │   └── UltrasonicDriver # SparkFun Qwiic HC-SR04 (I2C)
│       ├── modules/          # Higher-level subsystems
│       │   ├── EncoderCounter   # Interrupt-driven quadrature counting (2x/4x)
│       │   ├── VelocityEstimator # Edge-timing velocity with moving-average filter
│       │   ├── RobotKinematics  # Odometry model (edit here for Ackermann / mecanum)
│       │   ├── SensorManager    # IMU fusion (Madgwick AHRS) + mag calibration
│       │   ├── PersistentStorage # EEPROM read/write (mag cal offsets)
│       │   ├── MessageCenter    # TLV packet assembly, dispatch, and state machine
│       │   ├── StepperManager   # Multi-motor step sequencing (Timer3 ISR)
│       │   └── UserIO           # Buttons, LEDs, NeoPixel patterns
│       └── lib/              # Vendored third-party libraries
│           ├── Fusion/       # Madgwick AHRS (x-io Technologies)
│           ├── tlvcodec      # TLV packet encoder/decoder
│           ├── PCA9685       # I2C PWM driver
│           ├── Adafruit_NeoPixel
│           ├── SparkFun_9DoF_IMU_Breakout (ICM-20948)
│           ├── SparkFun_Garmin_LIDAR-Lite_v4
│           ├── SparkFun_Qwiic_Ultrasonic
│           └── SparkFun_Toolkit  # SparkFun sfTk dependency
├── tests/                    # Standalone test sketches
│   ├── test_scheduler/
│   ├── test_uart_tlv/
│   ├── test_encoder/
│   ├── test_dc_motor_pwm/
│   ├── test_current_sensing/
│   ├── test_servo/
│   ├── test_stepper/
│   ├── test_user_io/
│   ├── test_voltage/
│   ├── test_eeprom/
│   └── test_i2c_scanner/
├── notes/                    # Design notes and analysis
│   ├── REV_A_TO_REV_B_CHANGES.md
│   ├── TIMER3_CONFLICT_ANALYSIS.md
│   └── technical_notes.md
├── pin_table_rev_A.md        # Complete GPIO mapping — PCB Rev. A
└── pin_table_rev_B.md        # Complete GPIO mapping — PCB Rev. B (in testing)
```

---

## How It Works

**Boot and idle.** On power-up, the firmware initializes all hardware (motors, sensors, UART) and enters the **IDLE** state. In IDLE, no motors run, but the Arduino is listening and will respond to configuration commands and magnetometer calibration requests.

**Starting up.** The Raspberry Pi sends a `SYS_CMD_START` message over UART. The firmware transitions to **RUNNING**, enables full telemetry (motor status, IMU, odometry, voltages), and begins accepting motor commands. If `SYS_CMD_STOP` is received, the firmware returns to IDLE and disables all actuators.

**Command–telemetry loop.** All communication uses the [TLV v2.0 protocol](../COMMUNICATION_PROTOCOL.md). The Raspberry Pi sends typed commands (enable a motor, set velocity, move a stepper, set a servo position, etc.). The Arduino continuously streams back sensor data at rates appropriate to each subsystem — motor status and IMU at 100 Hz, voltage at 10 Hz, system health at 1–10 Hz.

**Safety.** The firmware expects a heartbeat (any received TLV) at least every 2000 ms. If the link goes silent, the liveness timer expires, all actuators are immediately disabled, and the NeoPixel turns red. An emergency stop (`SYS_CMD_ESTOP`) does the same and requires an explicit `SYS_CMD_RESET` to recover. All of this happens in hardware-interrupt-driven code on the Arduino — the Raspberry Pi cannot accidentally leave motors running if the link drops. See **Fault and Warning Reference** below for the full fault/warning model.

**Scheduler.** A two-tier scheduling architecture ensures hard real-time guarantees for critical tasks even if `loop()` stalls:

*Hard real-time (ISR-driven — unblockable):*

| ISR | Rate | What it does |
|-----|------|--------------|
| TIMER1_OVF_vect | 200 Hz | DC Motor PID (every tick) |
| TIMER1_OVF_vect | 100 Hz | UART comms + heartbeat safety (every 2nd tick); `sei()` re-enables nested interrupts so USART2_RX_vect can drain the receive buffer during processing |
| TIMER4_OVF_vect | 100 Hz | IMU + Fusion AHRS (`update100Hz`) |
| TIMER4_OVF_vect | 50 Hz  | Lidar reads (`update50Hz`) |
| TIMER4_OVF_vect | 10 Hz  | Voltages + Ultrasonic (`update10Hz`) |
| TIMER3_OVF_vect | 10 kHz | Stepper pulse generation |

*Soft real-time (millis-based, `loop()`-driven):*

| Task | Rate | What it does |
|------|------|--------------|
| User I/O | 20 Hz | LED animations, button reads, NeoPixel status |

---

## Features

### Communication
- **TLV v2.0 protocol** over UART at 1 Mbps (Serial2, pins 16/17 via level shifter)
- Bidirectional: RPi sends commands, Arduino sends sensor data and status
- CRC-checked frames; liveness timeout (500 ms default, configurable) auto-disables actuators
- Firmware state machine: IDLE → RUNNING → ESTOP with explicit transitions
- Debug output on USB Serial (Serial0) at 115200 baud

### DC Motors (4 channels)
- H-bridge control via PWM (speed) + digital direction signals
- Interrupt-driven quadrature encoder counting: **2x mode** (phase A only) or **4x mode** (both phases)
- Edge-timing velocity estimator with configurable moving-average filter
- **Three control modes**: position cascade PID, velocity PID, or direct PWM (open loop)
- All PID gains runtime-configurable via TLV commands
- Per-motor direction inversion (corrects for reversed wiring)
- Differential-drive odometry: x, y, heading computed from encoder deltas

### Stepper Motors (4 channels)
- STEP/DIR/ENABLE interface compatible with A4988 / DRV8825 drivers
- Timer3 ISR at 10 kHz for precise pulse generation (up to 5000 steps/sec)
- Trapezoidal acceleration profiling
- Limit switch homing support
- Individual enable/disable per channel

### Servos (PCA9685)
- Up to 16 servo channels via I2C PWM controller
- Per-channel enable/disable with global output enable control
- Position command via pulse width (µs) or angle (degrees)

### IMU — ICM-20948 9-DoF
- 3-axis accelerometer (mg), gyroscope (DPS), magnetometer (µT)
- **Madgwick AHRS** sensor fusion via Fusion library (x-io Technologies)
- Outputs: quaternion, earth-frame linear acceleration, raw sensor values
- 9-DoF mode (with magnetometer) or 6-DoF fallback if uncalibrated
- **Magnetometer hard-iron calibration**: interactive calibration via TLV command, offsets stored to EEPROM
- IMU update rate: 100 Hz

### Distance Sensors (I2C via Qwiic)
- **Garmin LIDAR-Lite v4**: 5 cm – 10 m, ~1 cm resolution, up to 4 sensors, 50 Hz
- **SparkFun Qwiic Ultrasonic (HC-SR04)**: 2 cm – 400 cm, ~3 mm accuracy, up to 4 sensors, 15 Hz
- Sensor counts and I2C addresses configured in `config.h`

### Voltage Monitoring
- Battery voltage (1:6 divider, 0–24 V range) at 10 Hz
- 5V rail and servo rail monitors
- Per-motor current sense (ADC) for current feedback

### Persistent Storage (EEPROM)
- Survives power-off; ~100,000 write cycles per byte
- Stores: magnetometer calibration offsets (hard-iron X/Y/Z)
- Wheel geometry is compile-time only — edit `WHEEL_DIAMETER_MM` / `WHEEL_BASE_MM` in `src/modules/RobotKinematics.h`
- API: `PersistentStorage::init()` / `getMagCalibration()` / `setMagCalibration()` (see `src/modules/PersistentStorage.h`)

### User I/O
- 2 on-board push-buttons (INPUT_PULLUP)
- 8 shared limit switch / button inputs (JST XH 3-pin connectors, pins 40–41, 48–53)
- Status RGB LED: WS2812B NeoPixel (pin 42) — system state patterns
- 3 user LEDs: green (pin 44), blue (pin 45), orange (pin 46) — PWM brightness control
- 1 user LED: purple (pin 47) — digital only

---

## LED Status Reference

The robot has two independent LED systems: an RGB NeoPixel that is driven automatically by the firmware state machine, and five discrete LEDs that are controlled by the Raspberry Pi (or manually via the TLV API).

### NeoPixel (WS2812B, pin 42)

The NeoPixel always reflects the current firmware state. Its animation is updated at **20 Hz** and resets cleanly on every state transition.

| State | Color | Pattern | When it occurs |
|-------|-------|---------|----------------|
| **INIT** | Yellow | Solid | Brief window during `setup()` before IDLE is entered |
| **IDLE** | Emerald green | Slow breathing (~3 s period) | Waiting for `SYS_CMD_START`; no motors running |
| **RUNNING** | Rainbow | Hue sweep (~3.2 s per cycle) | Normal operation; motor commands accepted |
| **ERROR** | Red | Fast flash (0.5 s period, alternates bright red ↔ dim orange-red) | Heartbeat lost while running, battery fault (critical or overvoltage), or other safety fault |
| **ESTOP** | Red | Solid (not blinking) | Emergency stop received via `SYS_CMD_ESTOP`; requires `SYS_CMD_RESET` to recover |

> **ERROR vs ESTOP:** Both show red, but ERROR blinks and ESTOP is solid. ERROR can be cleared with `SYS_CMD_RESET → IDLE`; ESTOP also requires `SYS_CMD_RESET` but is triggered by an explicit emergency stop command rather than a watchdog fault.

#### Fault conditions that trigger ERROR

The `SafetyManager` runs inside the 100 Hz UART ISR and forces ERROR state when any of the following are detected:

| Fault | Trigger |
|-------|---------|
| Heartbeat timeout | No TLV received from RPi within `HEARTBEAT_TIMEOUT_MS` (default 2000 ms) while RUNNING |
| Battery critical | Battery voltage below `VBAT_CUTOFF_V` (set by `BATTERY_TYPE` in `config.h`) |
| Battery overvoltage | Battery voltage above `VBAT_OVERVOLTAGE_V` |

In all cases, actuators (DC motors, steppers, servos) are disabled atomically before the state transition.

> **No-battery interlock:** When no battery is connected the ADC reads ~0 V (`batteryVoltage_ = 0.0f`). The firmware does **not** enter ERROR in this case — that would make it impossible to use the robot over USB alone (e.g. for communication testing). Instead, all actuator *enable* commands (`DC_ENABLE`, `DC_SET_POSITION`, `DC_SET_VELOCITY`, `DC_SET_PWM`, `STEP_ENABLE`, `SERVO_ENABLE`) are **silently rejected** until `SensorManager::isBatteryPresent()` returns true (`batteryVoltage_ >= VBAT_MIN_PRESENT_V = 2.0 V`, configured in `config.h`). Disable commands always succeed. The UI will show VBAT=0 V in the system panel; the enable toggle will appear to do nothing until a battery is connected.

---

### Discrete LEDs

The five discrete LEDs are controlled by the Raspberry Pi via the `IO_SET_LED` TLV command. The firmware manages their animation (blink timing, breathing waveform) but does not automatically change them based on system state — their behavior is entirely application-defined.

| LED | Pin (Rev A) | PWM | Default use |
|-----|-------------|-----|-------------|
| **RED** | 11 | Yes (Timer1 OC1A) | Error / low battery indication |
| **GREEN** | 44 | Yes (Timer5) | System OK / ready |
| **BLUE** | 45 | Yes (Timer5) | User-defined |
| **ORANGE** | 46 | Yes (Timer5) | User-defined / warning |
| **PURPLE** | 47 | No (digital only) | User-defined |

> **Note:** In Rev B, LED_RED moves to pin 5 (Timer3 OC3A). The firmware automatically uses the correct OCR register — `analogWrite()` is not used for LED_RED to avoid corrupting the timer.

#### Available LED modes (`IO_SET_LED` TLV)

| Mode | Behavior |
|------|----------|
| `LED_OFF` | Off |
| `LED_ON` | Fully on at specified brightness |
| `LED_PWM` | Constant dim at specified brightness (0–255) |
| `LED_BLINK` | Square-wave toggle at specified period (ms) |
| `LED_BREATHE` | Triangle-wave fade in/out at specified period. On LED_PURPLE (no PWM), falls back to ON/OFF at brightness threshold 128 |

---

## Fault and Warning Reference

The firmware distinguishes between **hard faults** (which force ERROR state and disable all actuators) and **soft warnings** (which set a flag in the telemetry stream but allow continued operation). Both are reported in the `errorFlags` bitmask of the `SYS_STATUS` TLV.

The UI backend caches each new flag as a human-readable notification until the user clears it.

### Hard Faults → ERROR state

All actuators (DC motors, steppers, servos) are disabled atomically before the state transition. The robot stays in ERROR until `SYS_CMD_RESET` is received **and** the underlying condition has resolved.

| Flag | Value | Trigger | Condition cleared when |
|------|-------|---------|------------------------|
| `ERR_LIVENESS_LOST` | `0x20` | No TLV received from RPi within `HEARTBEAT_TIMEOUT_MS` (default 2000 ms) while RUNNING | RPi reconnects and sends heartbeats; user sends `SYS_CMD_RESET` |
| `ERR_UNDERVOLTAGE` (critical) | `0x01` | Battery below `VBAT_CUTOFF_V` (configured per `BATTERY_TYPE`) | Battery replaced/charged; `SYS_CMD_RESET` |
| `ERR_OVERVOLTAGE` | `0x02` | Battery above `VBAT_OVERVOLTAGE_V` | Wrong charger removed; `SYS_CMD_RESET` |

> **Recovery**: `SYS_CMD_RESET` transitions ERROR → IDLE. The underlying fault must also be resolved (battery OK, RPi link live) or SafetyManager will immediately re-trigger ERROR on the next 100 Hz check.

### Enable interlock → silent reject, no state change

These conditions prevent actuators from being enabled at all, without changing the system state. Disable commands are always accepted.

| Condition | Value | Effect |
|-----------|-------|--------|
| No battery / below `VBAT_MIN_PRESENT_V` (2.0 V) | — | All enable commands silently rejected. System stays IDLE/RUNNING. Battery voltage shows as 0 V (or very low) in telemetry. |

This covers the common case of running USB-only (no battery connected) for communication testing without accidentally triggering ERROR state or locking up the state machine.

### Soft Warnings → flag only, operation continues

These conditions set `errorFlags` bits and appear as UI notifications, but the firmware does **not** enter ERROR and does not stop unaffected actuators.

| Flag | Value | Trigger | Per-motor / system effect |
|------|-------|---------|---------------------------|
| `ERR_UNDERVOLTAGE` (warning) | `0x01` | Battery below `VBAT_WARN_V` but above `VBAT_CUTOFF_V` | None (LED blinks amber) |
| `ERR_I2C_ERROR` | `0x08` | PCA9685 servo controller reports I2C failure | Servos may misbehave; robot stays RUNNING |
| `ERR_IMU_ERROR` | `0x10` | IMU not responding on init or lost during operation | Orientation data unavailable; kinematics falls back to encoder-only odometry |
| `ERR_ENCODER_FAIL` | `0x04` | Motor commanded at >20% PWM but encoder position unchanged for 500 ms | **Only that motor is disabled.** Other motors continue. Re-enable by sending `DC_ENABLE` for that motor. |
| `ERR_LOOP_OVERRUN` | `0x40` | Any single UART ISR tick exceeded 10 ms | Warning sent. If >5% of ticks in a 2-second window overrun, **all DC motors soft-stop** (robot stays RUNNING; steppers and servos continue). Re-enable motors with `DC_ENABLE`. Clear the latch with `SYS_CMD_RESET`. |

### Summary table

| Condition | State change | Actuators | Recovery |
|-----------|-------------|-----------|----------|
| No battery (< 2.0 V) | None | Enable commands rejected | Connect battery |
| Battery low warning | None | Unaffected | Replace battery |
| Heartbeat timeout (>2 s) | → ERROR | All disabled | Fix link + `SYS_CMD_RESET` |
| Battery critical (undervoltage cutoff) | → ERROR | All disabled | Charge battery + `SYS_CMD_RESET` |
| Battery overvoltage | → ERROR | All disabled | Remove wrong charger + `SYS_CMD_RESET` |
| I2C error | None | Unaffected | Power cycle I2C bus |
| IMU error | None | Unaffected | Restart or `SYS_CMD_RESET` |
| Encoder stall (single motor) | None | **That motor only** disabled | `DC_ENABLE` for that motor |
| Loop overrun (persistent >5%) | None | **DC motors** soft-stop | `DC_ENABLE` + `SYS_CMD_RESET` |
| `SYS_CMD_ESTOP` (operator) | → ESTOP | All disabled | `SYS_CMD_RESET` |

---

## Pin Tables

Two PCB revisions are supported. The firmware selects pins from `pins.h`:

- **[Rev. A](notes/pin_table_rev_A.md)** — current production board (2x encoder mode for M3/M4)
- **[Rev. B](notes/pin_table_rev_B.md)** — in testing; full 4x quadrature on all motors via PCINT

> **Migration note:** Rev. B is not fully validated yet. Do not switch `pins.h` to Rev. B until hardware testing is complete. See [notes/REV_A_TO_REV_B_CHANGES.md](notes/REV_A_TO_REV_B_CHANGES.md) for the complete pin remapping rationale.

---

## Building and Uploading

### Prerequisites

Install **Arduino IDE 2.x** or **arduino-cli**. Required board package:
- `arduino:avr` (Arduino AVR Boards) — install via Board Manager, target: **Arduino Mega 2560**

Required libraries are vendored in `src/lib/` — no Library Manager installs needed.

### Arduino IDE

1. Open `firmware/arduino/arduino.ino`
2. Select **Tools → Board → Arduino Mega 2560**
3. Select the correct COM/serial port
4. Click **Upload** (Ctrl+U / ⌘U)

### arduino-cli

```bash
cd firmware/arduino
arduino-cli compile --fqbn arduino:avr:mega .
arduino-cli upload  --fqbn arduino:avr:mega --port /dev/ttyUSB0 .
```

Replace `/dev/ttyUSB0` with your port (`/dev/tty.usbmodem*` on macOS, `COM3` on Windows).

### Configuration

Edit `src/config.h` before building:
- Enable/disable motors, sensors, servo controller
- Set UART baud rate, heartbeat timeout, PID defaults
- Set `IMU_ENABLED`, `LIDAR_COUNT`, `ULTRASONIC_COUNT`
- Set `ENCODER_N_MODE` per motor (`ENCODER_2X` or `ENCODER_4X`)

Edit `src/modules/RobotKinematics.h` for wheel geometry:
- `WHEEL_DIAMETER_MM` — outer diameter of each drive wheel (mm)
- `WHEEL_BASE_MM` — centre-to-centre track width (mm)
- `ODOM_LEFT_MOTOR` / `ODOM_RIGHT_MOTOR` — which DC motor index is each wheel

---

## Test Sketches

Each test in `firmware/tests/` is a standalone Arduino sketch that exercises one subsystem. All tests run at **115200 baud** on USB Serial (Serial0).

See **[firmware/tests/README.md](tests/README.md)** for:
- How to create the required `src` symlink in each test directory
- Build and upload instructions (Arduino IDE and arduino-cli)
- Per-test hardware requirements, serial commands, and what to observe

---

## Known Issues

### EEPROM unused warning

The compiler emits `'EEPROM' defined but not used` from Arduino's `EEPROM.h` when included in translation units that don't call EEPROM functions directly. This is a harmless upstream header issue and does not affect behavior.
