// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KINEMATICS_INTERFACE_H
#define KINEMATICS_INTERFACE_H

#define RPM_TO_RPS 1 / 60

class Kinematics
{
public:
    enum base
    {
        DIFFERENTIAL_DRIVE,
        SKID_STEER,
        MECANUM,
        SWERVE
    };

    base base_platform_;

    struct rpm
    {
        float motor1;
        float motor2;
        float motor3;
        float motor4;
    };

    struct velocities
    {
        float linear_x;
        float linear_y;
        float angular_z;
    };

    struct velocities_rpm
    {
        float x_rpm;
        float y_rpm;
        float tan_rpm;
    };

    struct pwm
    {
        int motor1;
        int motor2;
        int motor3;
        int motor4;
    };

    struct req_velocity
    {
        float req_a;
        float req_b;
        float req_c;
        float req_d;
    };

    Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
               float rpm_ratio, float wheel_diameter, float wheels_y_distance);
    float getMaxRPM();
    virtual velocities getVelocities(float rpm1, float rpm2, float rpm3, float rpm4);
    virtual rpm getRPM(float linear_x, float linear_y, float angular_z);

protected:
    virtual rpm calculateRPM(req_velocity req_velocity);
    virtual req_velocity calculateReqVelocity(velocities_rpm velocities_rpm);
    virtual velocities calculateVelocities(float rpm1, float rpm2, float rpm3, float rpm4);
    velocities_rpm calculateRPMVelocities(float linear_x, float linear_y, float angular_z);
    int getTotalWheels(base robot_base);
    float rpm_ratio_;
    float wheel_circumference_;
    int total_wheels_;
    float wheels_y_distance_;
    float max_rpm_;
    float pwm_res_;
};

#endif