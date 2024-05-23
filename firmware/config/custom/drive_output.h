#ifndef DRIVE_OUTPUT_H
#define DRIVE_OUTPUT_H

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV true
#define MOTOR2_INV true
#define MOTOR3_INV true
#define MOTOR4_INV true

#define MOTOR1_PWM 14
#define MOTOR1_IN_A 15
#define MOTOR1_IN_B 16

#define MOTOR2_PWM 28
#define MOTOR2_IN_A 9
#define MOTOR2_IN_B 10

#define MOTOR3_PWM 5
#define MOTOR3_IN_A 6
#define MOTOR3_IN_B 7

#define MOTOR4_PWM 2
#define MOTOR4_IN_A 3
#define MOTOR4_IN_B 4

#endif