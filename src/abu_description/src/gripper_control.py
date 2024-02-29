#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, Bool
from rclpy import qos, Parameter


class GripperController(Node):
    def __init__(self):
        super().__init__("gripper_controller_node")
        self.pub_gripper_right = self.create_publisher(
            Float64MultiArray,
            "right_gripper_controller/commands",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_gripper_left = self.create_publisher(
            Float64MultiArray,
            "left_gripper_controller/commands",
            qos_profile=qos.qos_profile_system_default,
        )
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.gripper_left = Float64MultiArray()
        self.gripper_right = Float64MultiArray()

    def timer_callback(self):
        self.pub_gripper_left.publish(self.gripper_left)
        self.pub_gripper_right.publish(self.gripper_right)


def main(args=None):
    rclpy.init(args=args)

    sub = GripperController()
    rclpy.spin(sub)
    sub.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
