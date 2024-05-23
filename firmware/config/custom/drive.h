#ifndef DRIVE_H
#define DRIVE_H
#include "robot.h"
#include "drive_input.h"
#include "drive_output.h"

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

#define USE_PRIK_KEE_NOO_DRIVER

#define RPM_RATIO 3.25
#define MOTOR_MAX_RPM 5500      // motor's max RPM
#define MAX_RPM_RATIO 1.0       // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define WHEEL_DIAMETER 0.0756   // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.63 // distance between left and right wheels

#endif