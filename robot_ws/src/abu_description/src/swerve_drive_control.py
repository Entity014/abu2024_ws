#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rclpy import qos, Parameter


class SwerveController(Node):
    def __init__(self):
        super().__init__("swerve_controller_node")
        self.sub_vel = self.create_subscription(
            Twist,
            "cmd_vel",
            self.sub_vel_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_vel
        self.pub_wheel = self.create_publisher(
            Float64MultiArray,
            "wheel_controller/commands",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_steering = self.create_publisher(
            Float64MultiArray,
            "steering_controller/commands",
            qos_profile=qos.qos_profile_system_default,
        )
        self.wheel = Float64MultiArray()
        self.steering = Float64MultiArray()
        self.max_speed = 6.0

    def sub_vel_callback(self, cmd_vel):
        rep_a = cmd_vel.linear.x + cmd_vel.angular.z * (0.6 / 2)
        rep_b = cmd_vel.linear.x - cmd_vel.angular.z * (0.6 / 2)
        rep_c = cmd_vel.linear.y + cmd_vel.angular.z * (0.6 / 2)
        rep_d = cmd_vel.linear.y - cmd_vel.angular.z * (0.6 / 2)

        wheel_data = [
            np.sqrt(pow(rep_a, 2) + pow(rep_c, 2)),
            np.sqrt(pow(rep_b, 2) + pow(rep_c, 2)),
            np.sqrt(pow(rep_a, 2) + pow(rep_d, 2)),
            np.sqrt(pow(rep_b, 2) + pow(rep_d, 2)),
        ]
        steering_data = [
            np.arctan2(rep_c, rep_a),
            np.arctan2(rep_c, rep_b),
            np.arctan2(rep_d, rep_b),
            np.arctan2(rep_d, rep_a),
        ]

        data_max = np.max(wheel_data)

        if data_max > self.max_speed:
            for i in range(len(wheel_data)):
                wheel_data[i] = wheel_data[i] / data_max * self.max_speed

        self.wheel.data = wheel_data
        self.steering.data = steering_data

        self.pub_wheel.publish(self.wheel)
        self.pub_steering.publish(self.steering)

        # self.get_logger().info(
        #     f"{wheel_data[0]} {wheel_data[1]} {wheel_data[2]} {wheel_data[3]} m/s"
        # )
        # self.get_logger().info(
        #     f"{steering_data[0]} {steering_data[1]} {steering_data[2]} {steering_data[3]} rad"
        # )


def main(args=None):
    rclpy.init(args=args)

    sub = SwerveController()
    rclpy.spin(sub)
    sub.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
