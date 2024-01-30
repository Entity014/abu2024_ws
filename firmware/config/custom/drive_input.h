#ifndef DRIVE_INPUT_H
#define DRIVE_INPUT_H
#include "robot.h"
#define K_P 0
#define K_I 0
#define K_D 0

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

#define COUNTS_DIR_PER_REV1 600 // wheel1 direction encoder's no of ticks per rev
#define COUNTS_DIR_PER_REV2 600 // wheel2 direction encoder's no of ticks per rev
#define COUNTS_DIR_PER_REV3 600 // wheel3 direction encoder's no of ticks per rev
#define COUNTS_DIR_PER_REV4 600 // wheel4 direction encoder's no of ticks per rev

// DIRECTION ENCODER PINS
#define MOTOR1_DIR_ENCODER_A -1
#define MOTOR1_DIR_ENCODER_B -1

#define MOTOR2_DIR_ENCODER_A -1
#define MOTOR2_DIR_ENCODER_B -1

#define MOTOR3_DIR_ENCODER_A -1
#define MOTOR3_DIR_ENCODER_B -1

#define MOTOR4_DIR_ENCODER_A -1
#define MOTOR4_DIR_ENCODER_B -1

// INVERT DIRECTION ENCODER COUNTS
#define MOTOR1_DIR_ENCODER_INV false
#define MOTOR2_DIR_ENCODER_INV false
#define MOTOR3_DIR_ENCODER_INV false
#define MOTOR4_DIR_ENCODER_INV false

#ifdef BASE == SWERVE
#define COUNTS_ROT_PER_REV1 600 // wheel1 rotation encoder's no of ticks per rev
#define COUNTS_ROT_PER_REV2 600 // wheel2 rotation encoder's no of ticks per rev
#define COUNTS_ROT_PER_REV3 600 // wheel3 rotation encoder's no of ticks per rev
#define COUNTS_ROT_PER_REV4 600 // wheel4 rotation encoder's no of ticks per rev
// ROTATION ENCODER PINS
#define MOTOR1_ROT_ENCODER_A -1
#define MOTOR1_ROT_ENCODER_B -1

#define MOTOR2_ROT_ENCODER_A -1
#define MOTOR2_ROT_ENCODER_B -1

#define MOTOR3_ROT_ENCODER_A -1
#define MOTOR3_ROT_ENCODER_B -1

#define MOTOR4_ROT_ENCODER_A -1
#define MOTOR4_ROT_ENCODER_B -1

// INVERT ROTATION ENCODER COUNTS
#define MOTOR1_ROT_ENCODER_INV false
#define MOTOR2_ROT_ENCODER_INV false
#define MOTOR3_ROT_ENCODER_INV false
#define MOTOR4_ROT_ENCODER_INV false
#endif

#endif