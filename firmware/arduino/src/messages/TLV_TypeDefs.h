#pragma once

#include <stdint.h>

// Define TLV type constants here so both server and client can use them:
// TLV type identifiers are 8-bit values on the wire.
// This file is auto-generated from TLV_TypeDefs.json - DO NOT EDIT MANUALLY

// ============================================================================
// TLV Type Constants
// ============================================================================

constexpr uint8_t SYS_HEARTBEAT = 1U;
constexpr uint8_t SYS_STATE = 2U;
constexpr uint8_t SYS_CMD = 3U;
constexpr uint8_t SYS_INFO_REQ = 4U;
constexpr uint8_t SYS_INFO_RSP = 5U;
constexpr uint8_t SYS_CONFIG_REQ = 6U;
constexpr uint8_t SYS_CONFIG_RSP = 7U;
constexpr uint8_t SYS_CONFIG_SET = 8U;
constexpr uint8_t SYS_POWER = 9U;
constexpr uint8_t SYS_DIAG_REQ = 10U;
constexpr uint8_t SYS_DIAG_RSP = 11U;
constexpr uint8_t SYS_ODOM_RESET = 12U;
constexpr uint8_t SYS_ODOM_PARAM_SET = 13U;
constexpr uint8_t SYS_ODOM_PARAM_REQ = 14U;
constexpr uint8_t SYS_ODOM_PARAM_RSP = 15U;
constexpr uint8_t DC_ENABLE = 16U;
constexpr uint8_t DC_SET_POSITION = 17U;
constexpr uint8_t DC_SET_VELOCITY = 18U;
constexpr uint8_t DC_SET_PWM = 19U;
constexpr uint8_t DC_STATE_ALL = 20U;
constexpr uint8_t DC_PID_REQ = 21U;
constexpr uint8_t DC_PID_RSP = 22U;
constexpr uint8_t DC_PID_SET = 23U;
constexpr uint8_t DC_RESET_POSITION = 24U;
constexpr uint8_t DC_HOME = 25U;
constexpr uint8_t STEP_ENABLE = 32U;
constexpr uint8_t STEP_MOVE = 33U;
constexpr uint8_t STEP_HOME = 34U;
constexpr uint8_t STEP_STATE_ALL = 35U;
constexpr uint8_t STEP_CONFIG_REQ = 36U;
constexpr uint8_t STEP_CONFIG_RSP = 37U;
constexpr uint8_t STEP_CONFIG_SET = 38U;
constexpr uint8_t SERVO_ENABLE = 48U;
constexpr uint8_t SERVO_SET = 49U;
constexpr uint8_t SERVO_STATE_ALL = 50U;
constexpr uint8_t SENSOR_IMU = 64U;
constexpr uint8_t SENSOR_KINEMATICS = 65U;
constexpr uint8_t SENSOR_ULTRASONIC_ALL = 66U;
constexpr uint8_t SENSOR_MAG_CAL_CMD = 67U;
constexpr uint8_t SENSOR_MAG_CAL_STATUS = 68U;
constexpr uint8_t IO_SET_LED = 80U;
constexpr uint8_t IO_SET_NEOPIXEL = 81U;
constexpr uint8_t IO_INPUT_STATE = 82U;
constexpr uint8_t IO_OUTPUT_STATE = 83U;

