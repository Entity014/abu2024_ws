#!/usr/bin/env python3
import rclpy
import numpy as np
import transforms3d
from rclpy import qos
from rclpy.node import Node
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
        self.publisher_ = self.create_publisher(
            Imu, "imu/data_raw", qos_profile=qos.qos_profile_system_default
        )

    def imu_callback(self, msgin):
        angular_velocity = [0.0, 0.0, 0.0]
        if abs(msgin.angular_velocity.x) > 0.025:
            angular_velocity[0] = msgin.angular_velocity.x
        if abs(msgin.angular_velocity.y) > 0.025:
            angular_velocity[1] = msgin.angular_velocity.y
        if abs(msgin.angular_velocity.z) > 0.025:
            angular_velocity[2] = msgin.angular_velocity.z
        msg = Imu()
        msg = msgin
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
