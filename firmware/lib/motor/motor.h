#ifndef MOTOR_H
#define MOTOR_H

#include "default_motor.h"

#ifdef USE_PRIK_KEE_NOO_DRIVER
#define Motor PRIK
#endif
#ifdef USE_BTS7960_DRIVER
#define Motor BTS7960
#endif

#endif