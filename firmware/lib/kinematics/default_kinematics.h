#ifndef DEFAULT_KINEMATIC_H
#define DEFAULT_KINEMATIC_H

#include "kinematics_interface.h"

class DifferentialDrive : public Kinematics
{
public:
    DifferentialDrive(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                      float rpm_ratio, float wheel_diameter, float wheels_y_distance)
        : Kinematics(robot_base, motor_max_rpm, max_rpm_ratio, rpm_ratio, wheel_diameter, wheels_y_distance){};
    velocities getVelocities(float rpm1, float rpm2, float rpm3, float rpm4) override;
    rpm getRPM(float linear_x, float linear_y, float angular_z) override;

protected:
    velocities calculateVelocities(float rpm1, float rpm2, float rpm3, float rpm4) override;
};

class SkidSteer : public Kinematics
{
public:
    SkidSteer(base robot_base, int motor_max_rpm, float max_rpm_ratio,
              float rpm_ratio, float wheel_diameter, float wheels_y_distance)
        : Kinematics(robot_base, motor_max_rpm, max_rpm_ratio, rpm_ratio, wheel_diameter, wheels_y_distance){};
    velocities getVelocities(float rpm1, float rpm2, float rpm3, float rpm4) override;
    rpm getRPM(float linear_x, float linear_y, float angular_z) override;

protected:
    velocities calculateVelocities(float rpm1, float rpm2, float rpm3, float rpm4) override;
};

class Mecanum : public Kinematics
{
public:
    Mecanum(base robot_base, int motor_max_rpm, float max_rpm_ratio,
            float rpm_ratio, float wheel_diameter, float wheels_y_distance)
        : Kinematics(robot_base, motor_max_rpm, max_rpm_ratio, rpm_ratio, wheel_diameter, wheels_y_distance){};
};

class SwerveDrive : public Kinematics
{
public:
    struct heading
    {
        float motor1;
        float motor2;
        float motor3;
        float motor4;
    };

    SwerveDrive(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                float rpm_ratio, float wheel_diameter, float wheels_y_distance)
        : Kinematics(robot_base, motor_max_rpm, max_rpm_ratio, rpm_ratio, wheel_diameter, wheels_y_distance){};
    velocities getVelocities(heading heading, float rpm1, float rpm2, float rpm3, float rpm4);
    heading getHeading(float linear_x, float linear_y, float angular_z);
    heading getBrake();

protected:
    req_velocity calculateReqVelocity(velocities_rpm velocities_rpm) override;
    rpm calculateRPM(req_velocity req_velocity) override;
    velocities calculateVelocities(heading heading, float rpm1, float rpm2, float rpm3, float rpm4);
    heading calculateHeading(req_velocity req_velocity);
    float calculateDegree(float degree);
    float calculateDirection(float speed, float req_a, float req_b);
};

#endif