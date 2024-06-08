#!/usr/bin/env python3
import rclpy
import numpy as np
import transforms3d
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3


class ImuSubscriber(Node):
    def __init__(self):
        super().__init__("imu_subscriber")
        self.subscription = self.create_subscription(
            Imu,
            "imu/data",
            self.imu_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.subscription
        self.publisher_ = self.create_publisher(
            Vector3, "imu/euler", qos_profile=qos.qos_profile_system_default
        )

    def imu_callback(self, msgin):
        orientation = [
            msgin.orientation.x,
            msgin.orientation.y,
            msgin.orientation.z,
            msgin.orientation.w,
        ]
        euler = np.degrees(transforms3d.euler.quat2euler(orientation))
        msg = Vector3()
        msg.x = -euler[0]
        msg.y = -euler[1]
        msg.z = -euler[2]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init()

    sub = ImuSubscriber()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
