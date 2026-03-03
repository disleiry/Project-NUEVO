#!/usr/bin/env python3
"""
test_uart_arduino.py - UART TLV v2.0 Baud-Rate Validation Test

Primary goal: find the highest reliable baud rate for the Arduino ↔ RPi link.

Usage:
    python3 test_uart_arduino.py [--port /dev/ttyAMA0] [--baud 1000000]

Suggested sweep (run once at each rate, read the quality report):
    python3 test_uart_arduino.py --baud 115200
    python3 test_uart_arduino.py --baud 230400
    python3 test_uart_arduino.py --baud 500000
    python3 test_uart_arduino.py --baud 1000000
    python3 test_uart_arduino.py --baud 2000000

Quality metrics tracked:
  Arduino → RPi:  decode_errors / total_frames  (CRC/framing failures in received data)
  RPi → Arduino:  uartRxErrors field in SYS_STATUS (CRC failures seen by the firmware)
  Link quality:   EXCELLENT <0.1% | GOOD <1% | MARGINAL <5% | FAIL ≥5%

Stats auto-print every 5 s. Press 's' + Enter for immediate report.

Hardware:
    Arduino Serial2 (pin 16 TX, pin 17 RX) → level shifter → RPi /dev/ttyAMA0
    Arduino USB Serial0 shows Arduino-side debug (open in a second terminal)

Requirements:
    pip3 install pyserial

Developed for MAE 162 Educational Robotics Platform (Winter/Spring 2026)
"""

import sys
import time
import argparse
import ctypes
import math
from typing import Optional

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip3 install pyserial")
    sys.exit(1)

# Add the src directory to path so we can import the codec and type defs
sys.path.insert(0, '../src')

from tlvcodec.src.encoder import Encoder
from tlvcodec.src.decoder import Decoder, DecodeErrorCode
from TLV_TypeDefs import *

# ============================================================================
# CONFIGURATION
# ============================================================================

DEFAULT_PORT       = '/dev/ttyAMA0'  # RPi UART; use /dev/ttyUSB0 for USB-serial adapter
DEFAULT_BAUD       = 1000000         # Must match RPI_BAUD_RATE in config.h
DEVICE_ID          = 0x01
HEARTBEAT_INTERVAL = 0.2             # seconds; Arduino liveness timeout = 500 ms
STATS_INTERVAL     = 5.0             # auto-print quality report every N seconds
ENABLE_CRC         = True

# ============================================================================
# PAYLOAD STRUCTS  (must match TLV_Payloads.h exactly — same field order/types)
# ============================================================================

# ---------- System ----------

class PayloadHeartbeat(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp", ctypes.c_uint32),   # RPi milliseconds since boot
        ("flags",     ctypes.c_uint8),    # Reserved — set 0
    ]
# 5 bytes

class PayloadSysCmd(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("command",  ctypes.c_uint8),     # SysCmdType: 1=START 2=STOP 3=RESET 4=ESTOP
        ("reserved", ctypes.c_uint8 * 3),
    ]
# 4 bytes

class PayloadSystemStatus(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("firmwareMajor",       ctypes.c_uint8),
        ("firmwareMinor",       ctypes.c_uint8),
        ("firmwarePatch",       ctypes.c_uint8),
        ("state",               ctypes.c_uint8),    # SystemState enum
        ("uptimeMs",            ctypes.c_uint32),
        ("lastRxMs",            ctypes.c_uint32),   # ms since last TLV from RPi
        ("lastCmdMs",           ctypes.c_uint32),   # ms since last non-heartbeat cmd
        ("batteryMv",           ctypes.c_uint16),
        ("rail5vMv",            ctypes.c_uint16),
        ("errorFlags",          ctypes.c_uint8),    # SystemErrorFlags bitmask
        ("attachedSensors",     ctypes.c_uint8),    # bit0=IMU, bit1=Lidar, bit2=US
        ("freeSram",            ctypes.c_uint16),
        ("loopTimeAvgUs",       ctypes.c_uint16),
        ("loopTimeMaxUs",       ctypes.c_uint16),
        ("uartRxErrors",        ctypes.c_uint16),
        ("wheelDiameterMm",     ctypes.c_float),
        ("wheelBaseMm",         ctypes.c_float),
        ("motorDirMask",        ctypes.c_uint8),
        ("neoPixelCount",       ctypes.c_uint8),
        ("heartbeatTimeoutMs",  ctypes.c_uint16),
        ("limitSwitchMask",     ctypes.c_uint16),
        ("stepperHomeLimitGpio",ctypes.c_uint8 * 4),
    ]
# 48 bytes

assert ctypes.sizeof(PayloadSystemStatus) == 48, \
    f"PayloadSystemStatus size wrong: {ctypes.sizeof(PayloadSystemStatus)}"

# SystemState values
SYS_STATE_NAMES = {0: "INIT", 1: "IDLE", 2: "RUNNING", 3: "ERROR", 4: "ESTOP"}

# ErrorFlags bitmask
ERROR_FLAGS = {
    0x01: "UNDERVOLTAGE",
    0x02: "OVERVOLTAGE",
    0x04: "ENCODER_FAIL",
    0x08: "I2C_ERROR",
    0x10: "IMU_ERROR",
    0x20: "LIVENESS_LOST",
    0x40: "LOOP_OVERRUN",
}

# ---------- DC Motors ----------

class PayloadDCEnable(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorId",  ctypes.c_uint8),   # Motor index 0–3
        ("mode",     ctypes.c_uint8),   # 0=disable 1=position 2=velocity 3=pwm
        ("reserved", ctypes.c_uint8 * 2),
    ]
# 4 bytes

class PayloadDCSetVelocity(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorId",     ctypes.c_uint8),
        ("reserved",    ctypes.c_uint8 * 3),
        ("targetTicks", ctypes.c_int32),    # Target velocity (ticks/sec)
    ]
# 8 bytes

class PayloadDCSetPWM(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motorId",  ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("pwm",      ctypes.c_int16),   # -255 to +255
    ]
# 4 bytes

class DCMotorStatus(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("mode",       ctypes.c_uint8),   # DCMotorMode
        ("faultFlags", ctypes.c_uint8),   # bit0=overcurrent, bit1=stall
        ("position",   ctypes.c_int32),   # Current position (ticks)
        ("velocity",   ctypes.c_int32),   # Current velocity (ticks/sec)
        ("targetPos",  ctypes.c_int32),
        ("targetVel",  ctypes.c_int32),
        ("pwmOutput",  ctypes.c_int16),
        ("currentMa",  ctypes.c_int16),   # -1 if not measured
        ("posKp",      ctypes.c_float),
        ("posKi",      ctypes.c_float),
        ("posKd",      ctypes.c_float),
        ("velKp",      ctypes.c_float),
        ("velKi",      ctypes.c_float),
        ("velKd",      ctypes.c_float),
    ]
# 46 bytes

assert ctypes.sizeof(DCMotorStatus) == 46, \
    f"DCMotorStatus size wrong: {ctypes.sizeof(DCMotorStatus)}"

class PayloadDCStatusAll(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("motors", DCMotorStatus * 4)]
# 184 bytes

DC_MODE_NAMES = {0: "DISABLED", 1: "POSITION", 2: "VELOCITY", 3: "PWM"}

# ---------- Steppers ----------

class StepperStatus(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("enabled",        ctypes.c_uint8),
        ("motionState",    ctypes.c_uint8),  # 0=IDLE 1=ACCEL 2=CRUISE 3=DECEL 4=HOMING 5=FAULT
        ("limitHit",       ctypes.c_uint8),
        ("reserved",       ctypes.c_uint8),
        ("commandedCount", ctypes.c_int32),
        ("targetCount",    ctypes.c_int32),
        ("currentSpeed",   ctypes.c_uint32),
        ("maxSpeed",       ctypes.c_uint32),
        ("acceleration",   ctypes.c_uint32),
    ]
# 24 bytes

assert ctypes.sizeof(StepperStatus) == 24, \
    f"StepperStatus size wrong: {ctypes.sizeof(StepperStatus)}"

class PayloadStepStatusAll(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("steppers", StepperStatus * 4)]
# 96 bytes

STEPPER_STATE_NAMES = {0: "IDLE", 1: "ACCEL", 2: "CRUISE", 3: "DECEL", 4: "HOMING", 5: "FAULT"}

# ---------- Servos ----------

class PayloadServoEnable(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("channel",  ctypes.c_uint8),   # 0–15; 0xFF = all
        ("enable",   ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 2),
    ]
# 4 bytes

class PayloadServoSetSingle(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("channel", ctypes.c_uint8),
        ("count",   ctypes.c_uint8),    # 1 for single
        ("pulseUs", ctypes.c_uint16),
    ]
# 4 bytes

class PayloadServoStatusAll(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("pca9685Connected", ctypes.c_uint8),
        ("pca9685Error",     ctypes.c_uint8),
        ("enabledMask",      ctypes.c_uint16),
        ("pulseUs",          ctypes.c_uint16 * 16),
    ]
# 36 bytes

assert ctypes.sizeof(PayloadServoStatusAll) == 36, \
    f"PayloadServoStatusAll size wrong: {ctypes.sizeof(PayloadServoStatusAll)}"

# ---------- Sensors ----------

class PayloadSensorVoltage(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("batteryMv",   ctypes.c_uint16),
        ("rail5vMv",    ctypes.c_uint16),
        ("servoRailMv", ctypes.c_uint16),
        ("reserved",    ctypes.c_uint16),
    ]
# 8 bytes

class PayloadSensorIMU(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("quatW",        ctypes.c_float),
        ("quatX",        ctypes.c_float),
        ("quatY",        ctypes.c_float),
        ("quatZ",        ctypes.c_float),
        ("earthAccX",    ctypes.c_float),
        ("earthAccY",    ctypes.c_float),
        ("earthAccZ",    ctypes.c_float),
        ("rawAccX",      ctypes.c_int16),
        ("rawAccY",      ctypes.c_int16),
        ("rawAccZ",      ctypes.c_int16),
        ("rawGyroX",     ctypes.c_int16),
        ("rawGyroY",     ctypes.c_int16),
        ("rawGyroZ",     ctypes.c_int16),
        ("magX",         ctypes.c_int16),
        ("magY",         ctypes.c_int16),
        ("magZ",         ctypes.c_int16),
        ("magCalibrated",ctypes.c_uint8),
        ("reserved",     ctypes.c_uint8),
        ("timestamp",    ctypes.c_uint32),
    ]
# 52 bytes

assert ctypes.sizeof(PayloadSensorIMU) == 52, \
    f"PayloadSensorIMU size wrong: {ctypes.sizeof(PayloadSensorIMU)}"

class PayloadSensorKinematics(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("x",         ctypes.c_float),
        ("y",         ctypes.c_float),
        ("theta",     ctypes.c_float),
        ("vx",        ctypes.c_float),
        ("vy",        ctypes.c_float),
        ("vTheta",    ctypes.c_float),
        ("timestamp", ctypes.c_uint32),
    ]
# 28 bytes

class PayloadIOStatus(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("buttonMask",     ctypes.c_uint16),
        ("ledBrightness",  ctypes.c_uint8 * 3),
        ("reserved",       ctypes.c_uint8),
        ("timestamp",      ctypes.c_uint32),
    ]
# 10 bytes fixed (+ 3 × neoPixelCount appended)

# ---------- NeoPixel ----------

class PayloadSetNeoPixel(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("index", ctypes.c_uint8),
        ("red",   ctypes.c_uint8),
        ("green", ctypes.c_uint8),
        ("blue",  ctypes.c_uint8),
    ]
# 4 bytes

# ============================================================================
# UART TEST CLASS
# ============================================================================

class UARTTest:
    def __init__(self, port: str, baudrate: int):
        self.port     = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.encoder  = Encoder(deviceId=DEVICE_ID, bufferSize=4096, crc=ENABLE_CRC)
        self.decoder  = Decoder(callback=self.decode_callback, crc=ENABLE_CRC)

        self.running             = True
        self.last_heartbeat_time = 0
        self.last_stats_time     = 0

        self.stats = {
            'heartbeats_sent':       0,
            'frames_received':       0,   # good frames (no decode error)
            'decode_errors':         0,   # Arduino→RPi CRC/framing failures
            'decode_errors_by_type': {},
            'by_type':               {},  # counts per TLV type name
            # RPi→Arduino direction — read from SYS_STATUS.uartRxErrors
            'arduino_uart_rx_errors_baseline': None,  # first value seen (pre-connection errors)
            'arduino_uart_rx_errors_current':  0,     # latest value from firmware
        }

    # ---- Connection ----

    def connect(self) -> bool:
        try:
            print(f"[UART] Opening {self.port} @ {self.baudrate} baud...")
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1
            )
            print(f"[UART] Connected")
            return True
        except serial.SerialException as e:
            print(f"[UART] ERROR: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[UART] Disconnected")

    # ---- TX helpers ----

    def _send(self, tlv_type: int, payload):
        self.encoder.reset()
        self.encoder.addPacket(tlv_type, ctypes.sizeof(payload), payload)
        length, buffer = self.encoder.wrapupBuffer()
        self.ser.write(buffer[:length])

    def send_heartbeat(self):
        p = PayloadHeartbeat()
        p.timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        p.flags = 0
        self._send(SYS_HEARTBEAT, p)
        self.stats['heartbeats_sent'] += 1

    def send_sys_cmd(self, cmd: int):
        """Send SYS_CMD: 1=START  2=STOP  3=RESET  4=ESTOP"""
        p = PayloadSysCmd()
        p.command = cmd
        self._send(SYS_CMD, p)
        names = {1: "START", 2: "STOP", 3: "RESET", 4: "ESTOP"}
        print(f"[TX] SYS_CMD_{names.get(cmd, cmd)}")

    def send_dc_enable(self, motor_id: int, mode: int):
        """mode: 0=disable 1=position 2=velocity 3=pwm"""
        p = PayloadDCEnable()
        p.motorId = motor_id
        p.mode    = mode
        self._send(DC_ENABLE, p)
        print(f"[TX] DC_ENABLE: Motor {motor_id}, mode={DC_MODE_NAMES.get(mode, mode)}")

    def send_dc_velocity(self, motor_id: int, ticks_per_sec: int):
        p = PayloadDCSetVelocity()
        p.motorId     = motor_id
        p.targetTicks = ticks_per_sec
        self._send(DC_SET_VELOCITY, p)
        print(f"[TX] DC_SET_VELOCITY: Motor {motor_id}, {ticks_per_sec} ticks/s")

    def send_dc_pwm(self, motor_id: int, pwm: int):
        """pwm: -255 to +255"""
        p = PayloadDCSetPWM()
        p.motorId = motor_id
        p.pwm     = max(-255, min(255, pwm))
        self._send(DC_SET_PWM, p)
        print(f"[TX] DC_SET_PWM: Motor {motor_id}, pwm={pwm}")

    def send_servo_enable(self, channel: int, enable: bool):
        p = PayloadServoEnable()
        p.channel = channel
        p.enable  = 1 if enable else 0
        self._send(SERVO_ENABLE, p)
        print(f"[TX] SERVO_ENABLE: Ch {channel}, enable={enable}")

    def send_servo_set(self, channel: int, pulse_us: int):
        p = PayloadServoSetSingle()
        p.channel = channel
        p.count   = 1
        p.pulseUs = pulse_us
        self._send(SERVO_SET, p)
        print(f"[TX] SERVO_SET: Ch {channel}, {pulse_us} µs")

    def send_neopixel(self, index: int, r: int, g: int, b: int):
        p = PayloadSetNeoPixel()
        p.index = index
        p.red   = r
        p.green = g
        p.blue  = b
        self._send(IO_SET_NEOPIXEL, p)
        print(f"[TX] IO_SET_NEOPIXEL: index={index}, RGB=({r},{g},{b})")

    # ---- RX ----

    def process_incoming(self):
        if self.ser.in_waiting > 0:
            data = self.ser.read(self.ser.in_waiting)
            self.decoder.decode(data)

    def decode_callback(self, error_code, frame_header, tlv_list):
        if error_code != DecodeErrorCode.NoError:
            self.stats['decode_errors'] += 1
            name = error_code.name if hasattr(error_code, 'name') else str(error_code)
            self.stats['decode_errors_by_type'][name] = \
                self.stats['decode_errors_by_type'].get(name, 0) + 1
            print(f"[RX] Decode error: {error_code}")
            return

        self.stats['frames_received'] += 1
        print(f"[RX] Frame from 0x{frame_header.deviceId:02X}, "
              f"seq={frame_header.frameNum}, {frame_header.numTlvs} TLV(s)")

        for tlv_type, tlv_len, tlv_data in tlv_list:
            type_name = TLV_NAMES.get(tlv_type, f"0x{tlv_type:X}")
            self.stats['by_type'][type_name] = self.stats['by_type'].get(type_name, 0) + 1
            self.handle_tlv(tlv_type, tlv_len, tlv_data)

    def handle_tlv(self, tlv_type: int, tlv_len: int, tlv_data: bytes):
        if tlv_type == SYS_STATUS:
            self._print_sys_status(tlv_len, tlv_data)

        elif tlv_type == DC_STATUS_ALL:
            self._print_dc_status(tlv_len, tlv_data)

        elif tlv_type == STEP_STATUS_ALL:
            self._print_step_status(tlv_len, tlv_data)

        elif tlv_type == SERVO_STATUS_ALL:
            self._print_servo_status(tlv_len, tlv_data)

        elif tlv_type == SENSOR_VOLTAGE:
            if tlv_len == ctypes.sizeof(PayloadSensorVoltage):
                v = PayloadSensorVoltage.from_buffer_copy(tlv_data)
                print(f"  [SENSOR_VOLTAGE] Batt={v.batteryMv} mV  "
                      f"5V={v.rail5vMv} mV  Servo={v.servoRailMv} mV")
            else:
                print(f"  [SENSOR_VOLTAGE] size mismatch ({tlv_len})")

        elif tlv_type == SENSOR_IMU:
            self._print_imu(tlv_len, tlv_data)

        elif tlv_type == SENSOR_KINEMATICS:
            if tlv_len == ctypes.sizeof(PayloadSensorKinematics):
                k = PayloadSensorKinematics.from_buffer_copy(tlv_data)
                print(f"  [KINEMATICS] x={k.x:.1f} mm  y={k.y:.1f} mm  "
                      f"θ={math.degrees(k.theta):.1f}°  "
                      f"vx={k.vx:.1f} mm/s  ω={math.degrees(k.vTheta):.1f}°/s")
            else:
                print(f"  [SENSOR_KINEMATICS] size mismatch ({tlv_len})")

        elif tlv_type == IO_STATUS:
            if tlv_len >= ctypes.sizeof(PayloadIOStatus):
                io = PayloadIOStatus.from_buffer_copy(tlv_data[:ctypes.sizeof(PayloadIOStatus)])
                print(f"  [IO_STATUS] buttons=0x{io.buttonMask:04X}  "
                      f"LEDs={list(io.ledBrightness)}")
            else:
                print(f"  [IO_STATUS] size mismatch ({tlv_len})")

        else:
            print(f"  [Type {TLV_NAMES.get(tlv_type, tlv_type)}] {tlv_len} bytes (not decoded)")

    def _print_sys_status(self, tlv_len: int, tlv_data: bytes):
        expected = ctypes.sizeof(PayloadSystemStatus)
        if tlv_len != expected:
            print(f"  [SYS_STATUS] size mismatch: expected {expected}, got {tlv_len}")
            return
        s = PayloadSystemStatus.from_buffer_copy(tlv_data)

        # Capture RPi→Arduino error count.
        # First value seen becomes the baseline (errors before Python connected).
        if self.stats['arduino_uart_rx_errors_baseline'] is None:
            self.stats['arduino_uart_rx_errors_baseline'] = s.uartRxErrors
            if s.uartRxErrors > 0:
                print(f"  [SYS_STATUS] NOTE: Arduino already had {s.uartRxErrors} "
                      f"uartRxErrors at connect time (pre-connection noise) — "
                      f"using as baseline, delta will be 0 if link is clean")
        self.stats['arduino_uart_rx_errors_current'] = s.uartRxErrors

        baseline = self.stats['arduino_uart_rx_errors_baseline']
        delta    = s.uartRxErrors - baseline

        state_name = SYS_STATE_NAMES.get(s.state, f"?{s.state}")
        errors = [name for bit, name in ERROR_FLAGS.items() if s.errorFlags & bit]
        delta_marker = f"  *** +{delta} NEW ***" if delta > 0 else ""
        print(f"  [SYS_STATUS] v{s.firmwareMajor}.{s.firmwareMinor}.{s.firmwarePatch}  "
              f"state={state_name}  uptime={s.uptimeMs/1000:.1f}s  "
              f"uartRxErr(RPi→Ard): total={s.uartRxErrors} baseline={baseline} delta={delta}"
              f"{delta_marker}")
        print(f"    errors={'|'.join(errors) if errors else 'none'}  "
              f"lastRx={s.lastRxMs} ms ago  "
              f"SRAM={s.freeSram} B  loop={s.loopTimeAvgUs}/{s.loopTimeMaxUs} µs")

    def _print_dc_status(self, tlv_len: int, tlv_data: bytes):
        expected = ctypes.sizeof(PayloadDCStatusAll)
        if tlv_len != expected:
            print(f"  [DC_STATUS_ALL] size mismatch: expected {expected}, got {tlv_len}")
            return
        d = PayloadDCStatusAll.from_buffer_copy(tlv_data)
        for i, m in enumerate(d.motors):
            if m.mode == 0:
                continue  # skip disabled motors
            print(f"  [DC M{i}] mode={DC_MODE_NAMES.get(m.mode, m.mode)}  "
                  f"pos={m.position}  vel={m.velocity} t/s  pwm={m.pwmOutput}")

    def _print_step_status(self, tlv_len: int, tlv_data: bytes):
        expected = ctypes.sizeof(PayloadStepStatusAll)
        if tlv_len != expected:
            print(f"  [STEP_STATUS_ALL] size mismatch: expected {expected}, got {tlv_len}")
            return
        d = PayloadStepStatusAll.from_buffer_copy(tlv_data)
        for i, st in enumerate(d.steppers):
            if not st.enabled:
                continue
            state_name = STEPPER_STATE_NAMES.get(st.motionState, f"?{st.motionState}")
            print(f"  [STEP S{i}] {state_name}  "
                  f"pos={st.commandedCount}  target={st.targetCount}  "
                  f"speed={st.currentSpeed} sps")

    def _print_servo_status(self, tlv_len: int, tlv_data: bytes):
        expected = ctypes.sizeof(PayloadServoStatusAll)
        if tlv_len != expected:
            print(f"  [SERVO_STATUS_ALL] size mismatch: expected {expected}, got {tlv_len}")
            return
        sv = PayloadServoStatusAll.from_buffer_copy(tlv_data)
        pca = "OK" if sv.pca9685Connected else "NOT FOUND"
        err = f" err=0x{sv.pca9685Error:02X}" if sv.pca9685Error else ""
        active = [(i, sv.pulseUs[i])
                  for i in range(16) if sv.enabledMask & (1 << i)]
        print(f"  [SERVO] PCA9685={pca}{err}  "
              f"enabled={'none' if not active else ', '.join(f'ch{i}={us}µs' for i,us in active)}")

    def _print_imu(self, tlv_len: int, tlv_data: bytes):
        expected = ctypes.sizeof(PayloadSensorIMU)
        if tlv_len != expected:
            print(f"  [SENSOR_IMU] size mismatch: expected {expected}, got {tlv_len}")
            return
        imu = PayloadSensorIMU.from_buffer_copy(tlv_data)
        # Convert quaternion to Euler (roll, pitch, yaw in degrees)
        w, x, y, z = imu.quatW, imu.quatX, imu.quatY, imu.quatZ
        roll  = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
        pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y-z*x)))))
        yaw   = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
        cal   = "9DOF" if imu.magCalibrated else "6DOF"
        print(f"  [IMU/{cal}] roll={roll:.1f}°  pitch={pitch:.1f}°  yaw={yaw:.1f}°  "
              f"accZ={imu.earthAccZ:.3f}g")

    # ---- Quality report ----

    def _quality_label(self, error_rate_pct: float) -> str:
        if error_rate_pct == 0.0:
            return "EXCELLENT (0%)"
        elif error_rate_pct < 0.1:
            return f"EXCELLENT ({error_rate_pct:.3f}%)"
        elif error_rate_pct < 1.0:
            return f"GOOD      ({error_rate_pct:.2f}%)"
        elif error_rate_pct < 5.0:
            return f"MARGINAL  ({error_rate_pct:.1f}%)"
        else:
            return f"FAIL      ({error_rate_pct:.1f}%)"

    def print_stats(self):
        good   = self.stats['frames_received']
        errors = self.stats['decode_errors']
        total  = good + errors

        baseline = self.stats['arduino_uart_rx_errors_baseline'] or 0
        current  = self.stats['arduino_uart_rx_errors_current']
        delta    = current - baseline   # errors since Python connected
        hb_sent  = self.stats['heartbeats_sent']

        # Arduino→RPi error rate
        a2r_rate = (errors / total * 100) if total > 0 else 0.0
        # RPi→Arduino error rate (delta only — excludes pre-connection noise)
        r2a_rate = (delta / hb_sent * 100) if hb_sent > 0 else 0.0

        print()
        print("=" * 62)
        print(f"  LINK QUALITY REPORT  —  {self.baudrate} baud")
        print("=" * 62)
        print(f"  Arduino → RPi  (Python decode errors / frames):")
        print(f"    Good frames : {good}")
        print(f"    Errors      : {errors}")
        if self.stats['decode_errors_by_type']:
            for k, v in self.stats['decode_errors_by_type'].items():
                print(f"      {k}: {v}")
        print(f"    Quality     : {self._quality_label(a2r_rate)}")
        print()
        print(f"  RPi → Arduino  (firmware uartRxErrors, delta since connect):")
        print(f"    Heartbeats sent     : {hb_sent}")
        print(f"    Errors at connect   : {baseline}  (pre-connection noise, excluded)")
        print(f"    New errors (delta)  : {delta}")
        print(f"    Quality             : {self._quality_label(r2a_rate)}")
        if baseline > 0 and delta == 0:
            print(f"    NOTE: all {baseline} errors occurred before Python connected —")
            print(f"          link is CLEAN since connection (likely boot noise on RX pin)")
        if self.stats['by_type']:
            print()
            print("  TLV counts received:")
            for k, v in sorted(self.stats['by_type'].items()):
                print(f"    {k}: {v}")
        print("=" * 62)
        print()

    # ---- Main loop ----

    def run(self):
        if not self.connect():
            return

        print("\n" + "=" * 62)
        print(f"  UART TLV Baud-Rate Validation  —  {self.baudrate} baud")
        print("=" * 62)
        print("Quality report auto-prints every 5 s.")
        print("Watch for decode_errors (Ard→RPi) and uartRxErrors (RPi→Ard).")
        print()
        print("Commands (type + Enter):")
        print("  1   - SYS_CMD_START  (IDLE → RUNNING, enables full telemetry)")
        print("  2   - SYS_CMD_STOP   (RUNNING → IDLE)")
        print("  3   - SYS_CMD_RESET  (ESTOP/ERROR → IDLE)")
        print("  4   - SYS_CMD_ESTOP")
        print("  e/d - DC motor 0 enable (velocity) / disable")
        print("  f/r - DC motor 0 forward / reverse 500 ticks/s")
        print("  p   - DC motor 0 direct PWM +150")
        print("  sa  - Servo ch 0 enable + center (1500 µs)")
        print("  sx  - Servo ch 0 disable")
        print("  l   - NeoPixel 0 green")
        print("  s   - Print quality report now")
        print("  h   - Send heartbeat manually")
        print("  q   - Quit")
        print("=" * 62)
        print()

        try:
            while self.running:
                now = time.time()

                # Auto-heartbeat
                if now - self.last_heartbeat_time >= HEARTBEAT_INTERVAL:
                    self.send_heartbeat()
                    self.last_heartbeat_time = now

                # Auto quality report
                if now - self.last_stats_time >= STATS_INTERVAL:
                    self.print_stats()
                    self.last_stats_time = now

                self.process_incoming()

                # Non-blocking stdin check
                import select
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    cmd = sys.stdin.readline().strip()

                    if cmd == '1':
                        self.send_sys_cmd(1)  # START
                    elif cmd == '2':
                        self.send_sys_cmd(2)  # STOP
                    elif cmd == '3':
                        self.send_sys_cmd(3)  # RESET
                    elif cmd == '4':
                        self.send_sys_cmd(4)  # ESTOP
                    elif cmd == 'e':
                        self.send_dc_enable(0, 2)   # velocity mode
                    elif cmd == 'd':
                        self.send_dc_enable(0, 0)   # disable
                    elif cmd == 'f':
                        self.send_dc_velocity(0, 500)
                    elif cmd == 'r':
                        self.send_dc_velocity(0, -500)
                    elif cmd == 'p':
                        self.send_dc_enable(0, 3)   # PWM mode
                        self.send_dc_pwm(0, 150)
                    elif cmd == 'sa':
                        self.send_servo_enable(0, True)
                        self.send_servo_set(0, 1500)
                    elif cmd == 'sx':
                        self.send_servo_enable(0, False)
                    elif cmd == 'l':
                        self.send_neopixel(0, 0, 255, 0)
                    elif cmd == 's':
                        self.print_stats()
                    elif cmd == 'h':
                        self.send_heartbeat()
                        print("[TX] Heartbeat sent manually")
                    elif cmd == 'q':
                        print("\n[Test] Exiting...")
                        self.running = False
                    elif cmd:
                        print(f"Unknown command: '{cmd}'")

                time.sleep(0.01)  # 100 Hz poll loop

        except KeyboardInterrupt:
            print("\n[Test] Interrupted by user")
        finally:
            self.print_stats()
            self.disconnect()

# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='UART TLV v2.0 Communication Test (pairs with test_uart_tlv.ino)')
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help=f'Baud rate (default: {DEFAULT_BAUD})')
    args = parser.parse_args()

    test = UARTTest(port=args.port, baudrate=args.baud)
    test.run()

if __name__ == '__main__':
    main()
