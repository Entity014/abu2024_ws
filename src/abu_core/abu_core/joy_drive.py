import rclpy
import math
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
from rclpy import qos, Parameter


class JoyDrive(Node):
    def __init__(self):
        super().__init__("joy_drive_node")
        self.sub_joy = self.create_subscription(
            Joy, "joy", self.sub_joy_callback, qos_profile=qos.qos_profile_sensor_data
        )
        self.sub_joy

        self.pub_drive = self.create_publisher(
            Twist, "cmd_vel", qos_profile=qos.qos_profile_system_default
        )
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.declare_parameters(
            "",
            [
                ("max_speed", Parameter.Type.DOUBLE),
            ],
        )
        self.max_speed = 0.0
        self.x, self.y, self.rotation = [0, 0, 0]
        self.pre_mode_botton = 0
        self.mode_button = 0
        self.mode = 0
        self.drive_mode = 0
        self.joy_data = []
        self.speed = 0.0

    def sub_joy_callback(self, msg_in):
        self.joy_data = msg_in
        self.mode_button = self.joy_data.buttons[10]
        # if self.mode == 0 or self.mode == 1:
        self.x = self.joy_data.axes[0]
        self.y = self.joy_data.axes[1]
        self.rotation = self.joy_data.axes[2]
        # elif self.mode == 2:
        #     self.accel = self.joy_data.axes[4]
        #     self.decel = self.joy_data.axes[5]
        #     if self.accel == -1 and self.speed <= self.max_speed:
        #         self.speed += 0.05
        #     elif self.decel == -1 and self.speed >= -self.max_speed:
        #         self.speed -= 0.05
        #     if self.speed > 0 and self.accel != -1:
        #         self.speed -= 0.01
        #     elif self.speed < 0 and self.decel != -1:
        #         self.speed = 0.0
        #     if self.speed >= 0:
        #         self.rotation = self.joy_data.axes[0]
        #     elif self.speed < 0.0:
        #         self.rotation = -self.joy_data.axes[0]

    def timer_callback(self):
        msg = Twist()
        self.max_speed = (
            self.get_parameter("max_speed").get_parameter_value().double_value
        )
        if self.pre_mode_botton != self.mode_button:
            if self.mode_button == 1:
                self.mode += 1
            self.pre_mode_botton = self.mode_button

        # if self.mode == 0:
        #     msg.linear.x = self.y * self.max_speed
        # elif self.mode == 1:
        msg.linear.x = self.y * self.max_speed
        msg.linear.y = self.x * self.max_speed
        # elif self.mode == 2:
        #     msg.linear.x = self.speed
        # else:
        #     self.mode = 0
        msg.angular.z = self.rotation * self.max_speed
        self.get_logger().info(
            f"{msg.linear.x} {msg.linear.y} {msg.angular.z} {self.gripper_mode}"
        )
        self.pub_drive.publish(msg)


def main():
    rclpy.init()

    sub = JoyDrive()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
