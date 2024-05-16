#!/usr/bin/env python3
import rclpy
import math

from rclpy.node import Node
from std_msgs.msg import Int8, String
from rclpy import qos


class RobotState(Node):
    def __init__(self):
        super().__init__("robot_state_node")
        self.sub_button = self.create_subscription(
            Int8,
            "button/start",
            self.sub_button_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_button

        self.pub_state = self.create_publisher(
            String, "robot/state", qos_profile=qos.qos_profile_system_default
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.__previous_button = Int8()
        self.state = 0

    def sub_button_callback(self, msgin):
        if msgin.data != self.__previous_button:
            if msgin.data == 1:
                self.state += 1
            self.__previous_button = msgin.data

    def timer_callback(self):
        msg = String()
        if self.state == 0:
            msg.data = "IDLE"
        elif self.state == 1:
            msg.data = "START"
        elif self.state == 2:
            msg.data = "RESET"
        else:
            self.state = 0
        self.pub_state.publish(msg)


def main():
    rclpy.init()

    sub = RobotState()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
