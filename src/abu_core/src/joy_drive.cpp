#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "RobotControl/ManualControl.h"

class JoyDrive : public rclcpp::Node
{
public:
    JoyDrive()
        : Node("joy_drive_node"), robot_type_(std::make_shared<RobotControl::Holonomic>())
    {
        pub_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS());
        sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS(), std::bind(&JoyDrive::sub_joy_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&JoyDrive::timer_callback, this));
        velocities_ = geometry_msgs::msg::Twist();

        this->declare_parameter("max_speed", 0.0);
    }

private:
    void timer_callback()
    {
        pub_velocity_->publish(velocities_);
    }
    void sub_joy_callback(const sensor_msgs::msg::Joy &joy_data)
    {
        double max_speed = this->get_parameter("max_speed").as_double();
        velocities_ = robot_type_->getVelocities(joy_data, max_speed);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<RobotControl::Holonomic> robot_type_;
    geometry_msgs::msg::Twist velocities_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyDrive>());
    rclcpp::shutdown();
    return 0;
}