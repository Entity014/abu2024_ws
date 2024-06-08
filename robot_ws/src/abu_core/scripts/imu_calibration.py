#!/usr/bin/env python3
import rclpy
import numpy as np
import transforms3d
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class ImuSubscriber(Node):
    def __init__(self):
        super().__init__("imu_subscriber")
        self.subscription = self.create_subscription(
            Imu,
            "camera/imu",
            self.imu_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.subscription
        self.sub_main = self.create_subscription(
            Int8,
            "robot/main",
            self.sub_robot_main_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_main
        self.sub_team = self.create_subscription(
            String,
            "robot/team",
            self.sub_team_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.publisher_ = self.create_publisher(
            Imu, "imu/data_raw", qos_profile=qos.qos_profile_system_default
        )
        self.robot_main_state = 0
        self.team = "none"

    def sub_robot_main_callback(self, msgin):
        self.robot_main_state = msgin.data

    def sub_team_callback(self, msgin):
        self.team = msgin.data

    def imu_callback(self, msgin):
        angular_velocity = [0.0, 0.0, 0.0]
        if abs(msgin.angular_velocity.x) > 0.015:
            angular_velocity[0] = msgin.angular_velocity.x
        if abs(msgin.angular_velocity.y) > 0.015:
            angular_velocity[1] = msgin.angular_velocity.y
        if abs(msgin.angular_velocity.z) > 0.015:
            angular_velocity[2] = msgin.angular_velocity.z
        msg = Imu()
        msg = msgin
        # if self.robot_main_state == 4 or self.robot_main_state == 10:
        #     if self.team == "BLUE":
        #         q = transforms3d.euler.euler2quat(0, 0, 0)
        #         msg.orientation.x = q[0]
        #         msg.orientation.y = q[1]
        #         msg.orientation.z = q[2]
        #         msg.orientation.w = q[3]
        #     elif self.team == "RED":
        #         q = transforms3d.euler.euler2quat(0, 0, np.pi / 2)
        #         msg.orientation.x = q[0]
        #         msg.orientation.y = q[1]
        #         msg.orientation.z = q[2]
        #         msg.orientation.w = q[3]
        msg.angular_velocity.x = angular_velocity[0]
        msg.angular_velocity.y = angular_velocity[1]
        msg.angular_velocity.z = angular_velocity[2]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init()

    sub = ImuSubscriber()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
