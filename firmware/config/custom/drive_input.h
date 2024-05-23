#ifndef DRIVE_INPUT_H
#define DRIVE_INPUT_H
/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

#define COUNTS_PER_REV1 2500 // wheel1 direction encoder's no of ticks per rev
#define COUNTS_PER_REV2 2500 // wheel2 direction encoder's no of ticks per rev
#define COUNTS_PER_REV3 2500 // wheel3 direction encoder's no of ticks per rev
#define COUNTS_PER_REV4 2500 // wheel4 direction encoder's no of ticks per rev

// DIRECTION ENCODER PINS
#define MOTOR1_ENCODER_A 33
#define MOTOR1_ENCODER_B 34

#define MOTOR2_ENCODER_A 35
#define MOTOR2_ENCODER_B 36

#define MOTOR3_ENCODER_A 37
#define MOTOR3_ENCODER_B 38

#define MOTOR4_ENCODER_A 39
#define MOTOR4_ENCODER_B 40

// INVERT DIRECTION ENCODER COUNTS
#define MOTOR1_ENCODER_INV true
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV true
#define MOTOR4_ENCODER_INV false

#define START 23

#endif