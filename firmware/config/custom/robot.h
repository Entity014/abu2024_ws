#ifndef ROBOT_H
#define ROBOT_H

#define LED_PIN 13

#define SWERVE_DRIVE_ROBOT
// #define USE_MPU9250_IMU

#define K_P 1.5
#define K_I 0.06
#define K_D 0.02

#define PWM_BITS 10         // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000 // PWM Frequency
#define PWM_MAX ((1 << PWM_BITS) - 1)
#define PWM_MIN ((1 << PWM_BITS) - 1) * -1

#endif
