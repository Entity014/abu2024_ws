#ifndef DRIVE_OUTPUT_H
#define DRIVE_OUTPUT_H
#include "robot.h"

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

#ifdef USE_SMILE_DRIVER
// INVERT DIR MOTOR DIRECTIONS
#define MOTOR1_DIR_INV false
#define MOTOR2_DIR_INV false
#define MOTOR3_DIR_INV false
#define MOTOR4_DIR_INV false

#define MOTOR1_DIR_PWM -1
#define MOTOR1_DIR_IN_A -1
#define MOTOR1_DIR_IN_B -1

#define MOTOR2_DIR_PWM -1
#define MOTOR2_DIR_IN_A -1
#define MOTOR2_DIR_IN_B -1

#define MOTOR3_DIR_PWM -1
#define MOTOR3_DIR_IN_A -1
#define MOTOR3_DIR_IN_B -1

#define MOTOR4_DIR_PWM -1
#define MOTOR4_DIR_IN_A -1
#define MOTOR4_DIR_IN_B -1

#ifdef BASE == SWERVE
// INVERT ROT MOTOR DIRECTIONS
#define MOTOR1_ROT_INV false
#define MOTOR2_ROT_INV false
#define MOTOR3_ROT_INV false
#define MOTOR4_ROT_INV false

#define MOTOR1_ROT_PWM -1
#define MOTOR1_ROT_IN_A -1
#define MOTOR1_ROT_IN_B -1

#define MOTOR2_ROT_PWM -1
#define MOTOR2_ROT_IN_A -1
#define MOTOR2_ROT_IN_B -1

#define MOTOR3_ROT_PWM -1
#define MOTOR3_ROT_IN_A -1
#define MOTOR3_ROT_IN_B -1

#define MOTOR4_ROT_PWM -1
#define MOTOR4_ROT_IN_A -1
#define MOTOR4_ROT_IN_B -1
#endif
#endif

#endif