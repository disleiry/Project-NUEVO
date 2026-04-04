#include "DCMotorBringup.h"

#include "../config.h"
#include "../pins.h"

uint16_t DCMotorBringup::countsPerRev() {
#if ENCODER_1_MODE == ENCODER_4X
    return (uint16_t)ENCODER_PPR;
#else
    return (uint16_t)(ENCODER_PPR / 2U);
#endif
}

void DCMotorBringup::initAll(DCMotor *motors,
                             IEncoderCounter &encoder1,
                             IEncoderCounter &encoder2,
                             IEncoderCounter &encoder3,
                             IEncoderCounter &encoder4) {
    initOne(motors[0], 0, encoder1,
            PIN_M1_ENC_A, PIN_M1_ENC_B, ENCODER_1_DIR_INVERTED, DC_MOTOR_1_DIR_INVERTED,
            PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2);
    initOne(motors[1], 1, encoder2,
            PIN_M2_ENC_A, PIN_M2_ENC_B, ENCODER_2_DIR_INVERTED, DC_MOTOR_2_DIR_INVERTED,
            PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2);
    initOne(motors[2], 2, encoder3,
            PIN_M3_ENC_A, PIN_M3_ENC_B, ENCODER_3_DIR_INVERTED, DC_MOTOR_3_DIR_INVERTED,
            PIN_M3_EN, PIN_M3_IN1, PIN_M3_IN2);
    initOne(motors[3], 3, encoder4,
            PIN_M4_ENC_A, PIN_M4_ENC_B, ENCODER_4_DIR_INVERTED, DC_MOTOR_4_DIR_INVERTED,
            PIN_M4_EN, PIN_M4_IN1, PIN_M4_IN2);
}

void DCMotorBringup::initOne(DCMotor &motor,
                             uint8_t motorId,
                             IEncoderCounter &encoder,
                             uint8_t encA,
                             uint8_t encB,
                             bool encoderInvert,
                             bool motorInvert,
                             uint8_t pinEN,
                             uint8_t pinIN1,
                             uint8_t pinIN2) {
    encoder.init(encA, encB, encoderInvert);
    motor.init(motorId, &encoder, motorInvert);
    motor.setPins(pinEN, pinIN1, pinIN2);
#if defined(PIN_M1_LIMIT)
    if (motorId == 0) {
        motor.setLimitPin(PIN_M1_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
    }
#endif
#if defined(PIN_M2_LIMIT)
    if (motorId == 1) {
        motor.setLimitPin(PIN_M2_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
    }
#endif
#if defined(PIN_M3_LIMIT)
    if (motorId == 2) {
        motor.setLimitPin(PIN_M3_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
    }
#endif
#if defined(PIN_M4_LIMIT)
    if (motorId == 3) {
        motor.setLimitPin(PIN_M4_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
    }
#endif
    motor.setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
    motor.setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
}
