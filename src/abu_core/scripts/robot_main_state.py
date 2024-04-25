#!/usr/bin/env python3
import rclpy
import math
import cv2
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Int8, String, Float32MultiArray, Bool, Int16MultiArray
from rclpy import qos


class RobotMainState(Node):
    def __init__(self):
        super().__init__("robot_main_state_node")
        self.sub_state = self.create_subscription(
            String,
            "robot/state",
            self.sub_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_state
        self.sub_goal_state = self.create_subscription(
            Bool,
            "goal/state",
            self.sub_goal_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_goal_state

        self.pub_main_state = self.create_publisher(
            Int8, "robot/main", qos_profile=qos.qos_profile_system_default
        )
        self.pub_ip = self.create_publisher(
            Float32MultiArray,
            "target_nav2_ip",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_goal = self.create_publisher(
            Float32MultiArray,
            "target_nav2_goal",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_gripper_hand = self.create_publisher(
            Int16MultiArray,
            "gripper/hand",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_gripper_arm = self.create_publisher(
            String,
            "gripper/arm",
            qos_profile=qos.qos_profile_system_default,
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.robot_state = String()
        self.robot_main_state = 0
        self.window_state = "normal"
        self.width, self.height = 1024, 600
        self.blue_color = (255, 250, 132)
        self.red_color = (145, 145, 255)
        self.yellow_color = (140, 249, 249)
        self.green_color = (160, 255, 160)
        self.team = "none"
        self.retry = "none"

        self.goal_state = Bool()
        self.__previous_goal_state = Bool()

    def sub_state_callback(self, msgin):
        self.robot_state = msgin.data

    def sub_goal_state_callback(self, msgin):
        self.goal_state = msgin.data
        if self.__previous_goal_state != self.goal_state:
            if self.goal_state:
                self.robot_main_state += 1
            self.__previous_goal_state = self.goal_state

    def timer_callback(self):
        msg = Int8()
        msg_gripper_arm = String()
        msg_gripper_hand = Int16MultiArray()
        if self.robot_state == "IDLE":
            self.terminal()
            msg_ip = Float32MultiArray()
            if self.team == "BLUE":
                if self.retry == "none":
                    msg_ip.data = [0.0, 0.0, 0.0]
                elif self.retry == "RETRY":
                    msg_ip.data = [5.15, -0.05, 0.0]
            elif self.team == "RED":
                if self.retry == "none":
                    msg_ip.data = [0.0, 10.8, 0.0]
                elif self.retry == "RETRY":
                    msg_ip.data = [5.15, 10.9, 0.0]
            self.pub_ip.publish(msg_ip)
        elif self.robot_state == "START":
            cv2.destroyAllWindows()
            msg_goal = Float32MultiArray()
            if self.robot_main_state == 0:
                msg_gripper_arm.data = "BOTTOM"
                msg_gripper_hand.data = [0, 110]
                if self.team == "BLUE":
                    msg_goal = [6.5, 0.0, 1.57]
                elif self.team == "RED":
                    msg_goal = [6.5, 10.9, -1.57]
            elif self.robot_main_state == 1:
                if self.team == "BLUE":
                    msg_goal = [6.5, 4.0, 0.0]
                elif self.team == "RED":
                    msg_goal = [6.5, 6.9, 0.0]
            elif self.robot_main_state == 2:
                if self.team == "BLUE":
                    msg_goal = [9.5, 4.0, -1.57]
                elif self.team == "RED":
                    msg_goal = [9.5, 6.9, 1.57]
            elif self.robot_main_state == 3:
                msg_gripper_arm.data = "BOTTOM"
                msg_gripper_hand.data = [0, 110]
            self.pub_goal.publish(msg_goal)
        elif self.robot_state == "RESET":
            self.robot_main_state = 0
            msg_gripper_arm.data = "BOTTOM"
            msg_gripper_hand.data = [0, 110]

        msg.data = self.robot_main_state
        self.pub_main_state.publish(msg)
        self.pub_gripper_arm.publish(msg_gripper_arm)
        self.pub_gripper_hand.publish(msg_gripper_hand)

    def terminal(self):
        name = "GUI"
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, self.width, self.height)
        cv2.setMouseCallback(name, self.mouse_callback)
        self.create_box_team(frame)
        self.create_box_retry(frame)
        self.toggle_color_team()
        self.toggle_color_retry()
        # self.toggle_fullscreen(name)
        cv2.imshow(name, frame)

        cv2.waitKey(1)

    def toggle_fullscreen(self, name):
        cv2.namedWindow(name, cv2.WND_PROP_FULLSCREEN)
        if self.window_state == "normal":
            cv2.setWindowProperty(name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        elif self.window_state == "fullscreen":
            cv2.setWindowProperty(name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    def toggle_color_team(self):
        if self.team == "BLUE":
            self.blue_color = (255, 0, 0)
            self.red_color = (145, 145, 255)
        elif self.team == "RED":
            self.blue_color = (255, 250, 132)
            self.red_color = (0, 0, 255)
        else:
            self.blue_color = (255, 250, 132)
            self.red_color = (145, 145, 255)

    def toggle_color_retry(self):
        if self.retry == "none":
            self.green_color = (0, 255, 0)
            self.yellow_color = (140, 249, 249)
        elif self.retry == "RETRY":
            self.green_color = (160, 255, 160)
            self.yellow_color = (0, 229, 255)

    def create_box_team(self, frame):
        cv2.rectangle(
            frame,
            (0, 0),
            (round(self.width / 2), round(self.height / 2)),
            self.blue_color,
            -1,
        )
        cv2.rectangle(
            frame,
            (round(self.width / 2), 0),
            (self.width, round(self.height / 2)),
            self.red_color,
            -1,
        )
        cv2.putText(
            frame,
            "BLUE TEAM",
            (
                40,
                (round(self.height / 4)) + 100,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (0, 0, 0),
            3,
        )
        cv2.putText(
            frame,
            "RED TEAM",
            (
                (round(3 * self.width / 4)),
                (round(self.height / 4)) + 100,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (0, 0, 0),
            3,
        )

    def create_box_retry(self, frame):
        cv2.rectangle(
            frame,
            (0, round(self.height / 2)),
            (round(self.width / 2), round(self.height)),
            self.green_color,
            -1,
        )
        cv2.rectangle(
            frame,
            (round(self.width / 2), round(self.height / 2)),
            (self.width, round(self.height)),
            self.yellow_color,
            -1,
        )
        cv2.putText(
            frame,
            "None",
            (
                40,
                (round(self.height / 2)) + 80,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (0, 0, 0),
            3,
        )
        cv2.putText(
            frame,
            "RETRY",
            (
                (round(3 * self.width / 4)) + 80,
                (round(self.height / 2)) + 80,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (0, 0, 0),
            3,
        )

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            if (
                x > 0
                and x < round(self.width / 2)
                and y > 0
                and y < round(self.height / 2)
            ):
                if self.team == "BLUE":
                    self.team = "none"
                else:
                    self.team = "BLUE"
            elif (
                x > round(self.width / 2)
                and x < round(self.width)
                and y > 0
                and y < round(self.height / 2)
            ):
                if self.team == "RED":
                    self.team = "none"
                else:
                    self.team = "RED"
            if (
                x > 0
                and x < round(self.width / 2)
                and y > round(self.height / 2)
                and y < round(self.height)
            ):
                self.retry = "none"
            elif (
                x > round(self.width / 2)
                and x < round(self.width)
                and y > round(self.height / 2)
                and y < round(self.height)
            ):
                if self.retry == "RETRY":
                    self.retry = "none"
                else:
                    self.retry = "RETRY"
            # if (self.window_state) == "normal":
            #     self.window_state = "fullscreen"
            # else:
            #     self.window_state = "normal"


def main():
    rclpy.init()

    sub = RobotMainState()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
