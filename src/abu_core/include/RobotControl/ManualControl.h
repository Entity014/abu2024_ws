#ifndef ROBOTCONTROL_MANUALCONTROL_H
#define ROBOTCONTROL_MANUALCONTROL_H

#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace RobotControl
{
    class Robot
    {
    public:
        Robot() {}
        static double map(double x, double in[2], double out[2]);
        virtual geometry_msgs::msg::Twist joy_to_twist(sensor_msgs::msg::Joy joy_data) = 0;
        geometry_msgs::msg::Twist getVelocities(sensor_msgs::msg::Joy joy_data, double max_speed);

    private:
        geometry_msgs::msg::Twist calculateVelocities(geometry_msgs::msg::Twist twist_data, double max_speed);
    };

    class NonHolonomic : public Robot
    {
    public:
        NonHolonomic() {}
        geometry_msgs::msg::Twist joy_to_twist(sensor_msgs::msg::Joy joy_data) override;
    };

    class Holonomic : public Robot
    {
    public:
        Holonomic() {}
        geometry_msgs::msg::Twist joy_to_twist(sensor_msgs::msg::Joy joy_data) override;
    };
}

#endif
