#include "Arduino.h"
#include "default_kinematics.h"

// DifferentialDrive
Kinematics::rpm DifferentialDrive::getRPM(float linear_x, float linear_y, float angular_z)
{
    linear_y = 0;
    return calculateRPM(calculateReqVelocity(calculateRPMVelocities(linear_x, linear_y, angular_z)));
}

Kinematics::velocities DifferentialDrive::getVelocities(float rpm1, float rpm2, float rpm3, float rpm4)
{
    rpm3 = 0.0;
    rpm4 = 0.0;
    return calculateVelocities(rpm1, rpm2, rpm3, rpm4);
}

Kinematics::velocities DifferentialDrive::calculateVelocities(float rpm1, float rpm2, float rpm3, float rpm4)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    // convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / total_wheels_) / 60.0; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_;                         // m/s

    average_rps_y = 0;
    vel.linear_y = average_rps_y;

    // convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / total_wheels_) / 60.0;
    vel.angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0); //  rad/s

    return vel;
}

// SkidSteer
Kinematics::rpm SkidSteer::getRPM(float linear_x, float linear_y, float angular_z)
{
    linear_y = 0;
    return calculateRPM(calculateReqVelocity(calculateRPMVelocities(linear_x, linear_y, angular_z)));
}

Kinematics::velocities SkidSteer::calculateVelocities(float rpm1, float rpm2, float rpm3, float rpm4)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    // convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / total_wheels_) / 60.0; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_;                         // m/s

    average_rps_y = 0;
    vel.linear_y = average_rps_y;

    // convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / total_wheels_) / 60.0;
    vel.angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0); //  rad/s

    return vel;
}

// SwerveDrive
Kinematics::velocities SwerveDrive::getVelocities(SwerveDrive::heading heading, float rpm1, float rpm2, float rpm3, float rpm4)
{
    return calculateVelocities(heading, rpm1, rpm2, rpm3, rpm4);
}

SwerveDrive::heading SwerveDrive::getHeading(float linear_x, float linear_y, float angular_z)
{
    return calculateHeading(calculateReqVelocity(calculateRPMVelocities(linear_x, linear_y, angular_z)));
}

SwerveDrive::heading getBrake()
{
    SwerveDrive::heading heading;
    heading.motor1 = PI / 4;
    heading.motor2 = -PI / 4;
    heading.motor3 = -PI / 4;
    heading.motor4 = PI / 4;
    return heading;
}

Kinematics::velocities SwerveDrive::calculateVelocities(SwerveDrive::heading heading, float rpm1, float rpm2, float rpm3, float rpm4)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    // convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)((rpm1 * cos(heading.motor1)) + (rpm2 * cos(heading.motor2)) + (rpm3 * cos(heading.motor3)) + (rpm4 * cos(heading.motor4))) / total_wheels_) / 60.0; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_;                                                                                                                         // m/s

    // convert average revolutions per minute in y axis to revolutions per second
    average_rps_y = ((float)((rpm1 * sin(heading.motor1)) + (rpm2 * sin(heading.motor2)) + (rpm3 * sin(heading.motor3)) + (rpm4 * sin(heading.motor4))) / total_wheels_) / 60.0; // RPM
    vel.linear_y = average_rps_y * wheel_circumference_;                                                                                                                         // m/s

    // convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)((rpm1) - (rpm2) + (rpm3) - (rpm4)) / total_wheels_) / 60.0 / 2;
    vel.angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0); //  rad/s

    return vel;
}

Kinematics::req_velocity SwerveDrive::calculateReqVelocity(Kinematics::velocities_rpm velocities_rpm)
{
    Kinematics::req_velocity req_velocity;
    req_velocity.req_a = velocities_rpm.x_rpm - velocities_rpm.tan_rpm;
    req_velocity.req_b = velocities_rpm.x_rpm + velocities_rpm.tan_rpm;
    req_velocity.req_c = velocities_rpm.y_rpm - velocities_rpm.tan_rpm;
    req_velocity.req_d = velocities_rpm.y_rpm + velocities_rpm.tan_rpm;
    return req_velocity;
}

Kinematics::rpm SwerveDrive::calculateRPM(Kinematics::req_velocity req_velocity)
{
    Kinematics::rpm rpm;
    // front-left motor
    rpm.motor1 = sqrt(pow(req_velocity.req_a, 2) + pow(req_velocity.req_c, 2));
    rpm.motor1 = calculateDirection(rpm.motor1, req_velocity.req_b, req_velocity.req_c);
    rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);

    // front-right motor
    rpm.motor2 = sqrt(pow(req_velocity.req_b, 2) + pow(req_velocity.req_c, 2));
    rpm.motor2 = calculateDirection(rpm.motor2, req_velocity.req_a, req_velocity.req_c);
    rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

    // rear-left motor
    rpm.motor3 = sqrt(pow(req_velocity.req_a, 2) + pow(req_velocity.req_d, 2));
    rpm.motor3 = calculateDirection(rpm.motor3, req_velocity.req_b, req_velocity.req_d);
    rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

    // rear-right motor
    rpm.motor4 = sqrt(pow(req_velocity.req_b, 2) + pow(req_velocity.req_d, 2));
    rpm.motor4 = calculateDirection(rpm.motor4, req_velocity.req_a, req_velocity.req_d);
    rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);
    return rpm;
}

SwerveDrive::heading SwerveDrive::calculateHeading(Kinematics::req_velocity req_velocity)
{
    SwerveDrive::heading heading;
    // heading.motor1 = ((req_velocity.req_c != 0) && (req_velocity.req_a != 0)) ? atan2(req_velocity.req_c, req_velocity.req_a) : PI / 4;
    heading.motor1 = atan2(req_velocity.req_c, req_velocity.req_b);
    heading.motor1 = calculateDegree(heading.motor1);
    heading.motor1 = constrain(heading.motor1, -PI, PI);

    // heading.motor2 = ((req_velocity.req_c != 0) && (req_velocity.req_b != 0)) ? atan2(req_velocity.req_c, req_velocity.req_b) : -PI / 4;
    heading.motor2 = atan2(req_velocity.req_c, req_velocity.req_a);
    heading.motor2 = calculateDegree(heading.motor2);
    heading.motor2 = constrain(heading.motor2, -PI, PI);

    // heading.motor3 = ((req_velocity.req_d != 0) && (req_velocity.req_b != 0)) ? atan2(req_velocity.req_d, req_velocity.req_b) : -PI / 4;
    heading.motor3 = atan2(req_velocity.req_d, req_velocity.req_b);
    heading.motor3 = calculateDegree(heading.motor3);
    heading.motor3 = constrain(heading.motor3, -PI, PI);

    // heading.motor4 = ((req_velocity.req_d != 0) && (req_velocity.req_a != 0)) ? atan2(req_velocity.req_d, req_velocity.req_a) : PI / 4;
    heading.motor4 = atan2(req_velocity.req_d, req_velocity.req_a);
    heading.motor4 = calculateDegree(heading.motor4);
    heading.motor4 = constrain(heading.motor4, -PI, PI);
    return heading;
}

float SwerveDrive::calculateDegree(float degree)
{
    return (roundf(degree * RAD_TO_DEG) > roundf((PI / 2) * RAD_TO_DEG)) ? degree - PI : (roundf(degree * RAD_TO_DEG) < roundf((-PI / 2) * RAD_TO_DEG)) ? degree + PI
                                                                                                                                                        : degree;
}

float SwerveDrive::calculateDirection(float speed, float req_a, float req_b)
{
    return (atan2(req_a, req_b) < 0) ? -speed : speed;
}
