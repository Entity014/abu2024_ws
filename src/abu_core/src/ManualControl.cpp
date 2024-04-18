#include "RobotControl/ManualControl.h"

namespace RobotControl
{
    double Robot::map(double x, double in[2], double out[2])
    {
        return (x - in[0]) * (out[1] - out[0]) / (in[1] - in[0]) + out[0];
    }

    geometry_msgs::msg::Twist NonHolonomic::joy_to_twist(sensor_msgs::msg::Joy joy_data)
    {
        geometry_msgs::msg::Twist velocities;
        velocities.linear.x = 0.0;
        velocities.linear.y = joy_data.axes[1];
        velocities.angular.z = joy_data.axes[2];
        return velocities;
    }

    geometry_msgs::msg::Twist Holonomic::joy_to_twist(sensor_msgs::msg::Joy joy_data)
    {
        geometry_msgs::msg::Twist velocities;
        velocities.linear.x = (joy_data.axes[0] != 0) ? joy_data.axes[0] : joy_data.axes[6];
        velocities.linear.y = (joy_data.axes[1] != 0) ? joy_data.axes[1] : joy_data.axes[7];
        velocities.angular.z = joy_data.axes[2];
        return velocities;
    }

    geometry_msgs::msg::Twist Robot::calculateVelocities(geometry_msgs::msg::Twist twist_data, double max_speed)
    {
        double in_range[2] = {-1.0, 1.0};
        double out_range[2] = {-max_speed, max_speed};

        geometry_msgs::msg::Twist velocities;
        velocities.linear.x = map(twist_data.linear.y, in_range, out_range);
        velocities.linear.y = map(twist_data.linear.x, in_range, out_range);
        velocities.angular.z = map(twist_data.angular.z, in_range, out_range);
        return velocities;
    }

    geometry_msgs::msg::Twist Robot::getVelocities(sensor_msgs::msg::Joy joy_data, double max_speed)
    {
        return calculateVelocities(joy_to_twist(joy_data), max_speed);
    }
}
