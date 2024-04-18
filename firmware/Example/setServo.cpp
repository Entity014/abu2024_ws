#include <Arduino.h>
#include <PWMServo.h>
#include "config.h"

PWMServo servo1_controller;
PWMServo servo2_controller;
PWMServo servo3_controller;
PWMServo servo4_controller;

void setup()
{
    pinMode(EMERGENCY, OUTPUT);
    servo1_controller.attach(SERVO1);
    servo2_controller.attach(SERVO2);
    servo3_controller.attach(SERVO3);
    servo4_controller.attach(SERVO4);
}

void loop()
{
    digitalWrite(EMERGENCY, HIGH);
    servo1_controller.write(90);
    servo2_controller.write(90);
    servo3_controller.write(90);
    servo4_controller.write(90);
}