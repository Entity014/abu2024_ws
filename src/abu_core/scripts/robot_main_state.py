#!/usr/bin/env python3
import rclpy
import math
import cv2
import numpy as np
import time

from rclpy.node import Node
from std_msgs.msg import Int8, String, Float32MultiArray, Bool, Int16MultiArray
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
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
        self.sub_move = self.create_subscription(
            String,
            "color/move",
            self.sub_move_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_move
        self.sub_color_found = self.create_subscription(
            Bool,
            "color/found",
            self.sub_color_found_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_color_found
        self.sub_limit = self.create_subscription(
            Twist,
            "gripper/limit",
            self.sub_limit_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_limit
        self.sub_goal_state = self.create_subscription(
            Bool,
            "goal/state",
            self.sub_goal_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_goal_state
        self.sub_silo_pos = self.create_subscription(
            Vector3,
            "silo/pos",
            self.sub_silo_pos_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_silo_pos
        self.sub_lidar = self.create_subscription(
            LaserScan,
            "scan",
            self.sub_lidar_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_lidar

        self.pub_main_state = self.create_publisher(
            Int8, "robot/main", qos_profile=qos.qos_profile_system_default
        )
        self.pub_team = self.create_publisher(
            String, "robot/team", qos_profile=qos.qos_profile_system_default
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
        self.pub_gripper_motor = self.create_publisher(
            Bool,
            "gripper/motor",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            "cmd_vel",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_silo_type = self.create_publisher(
            String,
            "silo/type",
            qos_profile=qos.qos_profile_system_default,
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.robot_state = String()
        self.window_state = "normal"
        self.width, self.height = 1024, 600
        self.blue_color = (255, 250, 132)
        self.red_color = (145, 145, 255)
        self.yellow_color = (140, 249, 249)
        self.green_color = (160, 255, 160)
        self.team = "none"
        self.retry = "none"

        self.color_found = Bool()
        self.move = "none"
        self.goal_state = Bool()
        self.__previous_goal_state = Bool()

        self.top_limit = 0
        self.bottom_limit = 0

        self.color_state = 0
        self.robot_main_state = 0  # TODO: Edit here
        self.gripper_state = 0
        self.ball_type = 0
        self.silo_state = 0

        self.silo_pos = Vector3()
        self.silo_arr = [
            "silo5",
            "silo4",
            "silo3",
            "silo2",
            "silo1",
            "silo5",
            "silo3",
            "silo1",
            "silo5",
            "silo3",
            "silo1",
            "silo2",
        ]

    def sub_state_callback(self, msgin):
        self.robot_state = msgin.data

    def sub_goal_state_callback(self, msgin):
        self.goal_state = msgin.data
        if self.__previous_goal_state != self.goal_state:
            if self.goal_state:
                self.robot_main_state += 1
            self.__previous_goal_state = self.goal_state

    def sub_move_callback(self, msgin):
        self.move = msgin.data

    def sub_color_found_callback(self, msgin):
        self.color_found = msgin.data

    def sub_limit_callback(self, msgin):
        self.top_limit = msgin.linear.x
        self.bottom_limit = msgin.linear.y

    def sub_silo_pos_callback(self, msgin):
        self.silo_pos = msgin
        # self.get_logger().info(f"{self.silo_pos}")
        self.robot_main_state = 7

    def sub_lidar_callback(self, msgin):
        # self.get_logger().info(f"{self.color_state}")
        if msgin.ranges[352] < 0.5:
            self.color_state = 0
        elif msgin.ranges[1443] < 0.5:
            self.color_state = 1

    def timer_callback(self):
        msg = Int8()
        msg_team = String()
        msg_cmd_vel = Twist()
        msg_silo_type = String()
        msg_gripper_arm = String()
        msg_gripper_motor = Bool()
        msg_ip = Float32MultiArray()
        msg_goal = Float32MultiArray()
        msg_gripper_hand = Int16MultiArray()
        if self.robot_state == "IDLE":
            self.terminal()
            msg_gripper_motor.data = False
            msg_gripper_arm.data = "BOTTOM"
            msg_gripper_hand.data = [10, 90]
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
            self.pub_gripper_arm.publish(msg_gripper_arm)
            self.pub_gripper_hand.publish(msg_gripper_hand)
            self.pub_gripper_motor.publish(msg_gripper_motor)

        elif self.robot_state == "START":
            cv2.destroyAllWindows()
            if self.robot_main_state == 0:  # ? go to point 1 ( Slope stage 1 )
                if self.team == "BLUE":
                    msg_goal.data = [6.5, 0.0, 0.0]
                elif self.team == "RED":
                    msg_goal.data = [6.5, 10.9, 0.0]
                self.pub_goal.publish(msg_goal)
            elif self.robot_main_state == 1:  # ? go to point 2 ( Slope stage 2 )
                if self.team == "BLUE":
                    msg_goal.data = [6.5, 4.0, 0.0]
                elif self.team == "RED":
                    msg_goal.data = [6.5, 6.9, 0.0]
                self.pub_goal.publish(msg_goal)
            elif self.robot_main_state == 2:  # ? go to point 3 ( Silo )
                if self.team == "BLUE":
                    msg_goal.data = [9.5, 4.0, 1.57]
                elif self.team == "RED":
                    msg_goal.data = [9.5, 6.9, -1.57]
                self.pub_goal.publish(msg_goal)
            elif self.robot_main_state == 3:  # ? go to point 4 ( Ball )
                if self.team == "BLUE":
                    msg_goal.data = [9.5, 1.0, 1.57]
                elif self.team == "RED":
                    msg_goal.data = [9.5, 9.9, -1.57]
                if self.color_found:
                    msg_goal.data = [0.0, 0.0, 0.0, 0.0, 0.0]
                    self.robot_main_state = 4
                self.pub_goal.publish(msg_goal)
            elif self.robot_main_state == 4:  # ? find ball
                msg_gripper_motor.data = True
                msg_gripper_arm.data = "BOTTOM"
                msg_gripper_hand.data = [15, 90]
                if self.color_found:
                    if self.move == "LEFT":
                        msg_cmd_vel.linear.x = -0.2
                        msg_cmd_vel.angular.z = 0.2
                    elif self.move == "RIGHT":
                        msg_cmd_vel.linear.x = -0.2
                        msg_cmd_vel.angular.z = -0.2
                    elif self.move == "CENTER":
                        msg_cmd_vel.linear.x = -0.2
                        msg_cmd_vel.angular.z = 0.0
                else:
                    if self.color_state == 0:
                        msg_cmd_vel.linear.y = 0.2
                    elif self.color_state == 1:
                        msg_cmd_vel.linear.y = -0.2

                if self.move == "DONE":
                    msg_cmd_vel.linear.x = 0.0
                    msg_cmd_vel.angular.z = 0.0
                    self.ball_type = 0
                    self.robot_main_state = 5
                elif self.move == "FAIL":
                    self.ball_type = 1
                    self.robot_main_state = 5

                self.pub_cmd_vel.publish(msg_cmd_vel)
                self.pub_gripper_arm.publish(msg_gripper_arm)
                self.pub_gripper_hand.publish(msg_gripper_hand)
                self.pub_gripper_motor.publish(msg_gripper_motor)
            elif self.robot_main_state == 5:  # ? pick ball
                if self.ball_type == 0:
                    if self.gripper_state == 0:
                        msg_cmd_vel.linear.x = 0.0
                        msg_gripper_arm.data = "BOTTOM"
                        msg_gripper_hand.data = [0, 120]
                        msg_gripper_motor.data = True
                        self.pub_cmd_vel.publish(msg_cmd_vel)
                        self.pub_gripper_arm.publish(msg_gripper_arm)
                        self.pub_gripper_hand.publish(msg_gripper_hand)
                        self.pub_gripper_motor.publish(msg_gripper_motor)
                        time.sleep(0.4)
                        self.gripper_state = 1
                    elif self.gripper_state == 1:
                        msg_gripper_arm.data = "TOP"
                        self.pub_gripper_arm.publish(msg_gripper_arm)
                        self.gripper_state = 0
                        self.robot_main_state = 6  # TODO: Edit here
                elif self.ball_type == 1:
                    if self.gripper_state == 0:
                        msg_gripper_motor.data = True
                        msg_gripper_arm.data = "BOTTOM"
                        msg_gripper_hand.data = [0, 120]
                        self.pub_gripper_arm.publish(msg_gripper_arm)
                        self.pub_gripper_hand.publish(msg_gripper_hand)
                        self.pub_gripper_motor.publish(msg_gripper_motor)
                        time.sleep(0.4)
                        self.gripper_state = 1
                    elif self.gripper_state == 1:
                        msg_gripper_arm.data = "TOP"
                        self.pub_gripper_arm.publish(msg_gripper_arm)
                        time.sleep(1)
                        self.gripper_state = 2
                    elif self.gripper_state == 2:
                        msg_gripper_hand.data = [65, 120]
                        self.pub_gripper_hand.publish(msg_gripper_hand)
                        time.sleep(1)
                        self.gripper_state = 3
                    elif self.gripper_state == 3:
                        msg_gripper_hand.data = [65, 90]
                        self.pub_gripper_hand.publish(msg_gripper_hand)
                        time.sleep(0.6)
                        self.gripper_state = 4
                    elif self.gripper_state == 4:
                        msg_gripper_arm.data = "BOTTOM"
                        msg_gripper_hand.data = [0, 100]
                        msg_gripper_motor.data = False
                        self.pub_gripper_arm.publish(msg_gripper_arm)
                        self.pub_gripper_hand.publish(msg_gripper_hand)
                        self.pub_gripper_motor.publish(msg_gripper_motor)
                        if self.bottom_limit == 1:
                            self.gripper_state = 0
                            self.robot_main_state = 3
            elif self.robot_main_state == 6:  # ? select silo
                msg_gripper_motor.data = False
                msg_silo_type.data = self.silo_arr[self.silo_state]
                self.pub_gripper_motor.publish(msg_gripper_motor)
                self.pub_silo_type.publish(msg_silo_type)
            elif self.robot_main_state == 7:  # ? go to silo
                if self.team == "BLUE":
                    msg_goal.data = [self.silo_pos.x, self.silo_pos.y - 0.5, 1.57]
                elif self.team == "RED":
                    msg_goal.data = [self.silo_pos.x, self.silo_pos.y + 0.5, -1.57]
                self.pub_goal.publish(msg_goal)
            elif self.robot_main_state == 8:  # ? place ball to silo
                if self.gripper_state == 0:
                    msg_gripper_hand.data = [65, 120]
                    self.pub_gripper_hand.publish(msg_gripper_hand)
                    time.sleep(1)
                    self.gripper_state = 1
                elif self.gripper_state == 1:
                    msg_gripper_hand.data = [65, 90]
                    self.pub_gripper_hand.publish(msg_gripper_hand)
                    time.sleep(0.6)
                    self.gripper_state = 2
                elif self.gripper_state == 2:
                    msg_gripper_arm.data = "BOTTOM"
                    msg_gripper_hand.data = [0, 100]
                    self.pub_gripper_arm.publish(msg_gripper_arm)
                    self.pub_gripper_hand.publish(msg_gripper_hand)
                    if self.bottom_limit == 1:
                        self.gripper_state = 0
                        if self.silo_state < 11:
                            self.silo_state += 1
                        self.robot_main_state = 3

        elif self.robot_state == "RESET":
            self.robot_main_state = 0  # TODO: Edit here
            self.gripper_state = 0
            self.ball_type = 0
            self.color_state = 0
            self.timer_state = 0
            msg_gripper_arm.data = "BOTTOM"
            msg_gripper_motor.data = False
            msg_gripper_hand.data = [10, 90]
            msg_goal.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.pub_goal.publish(msg_goal)
            self.pub_gripper_arm.publish(msg_gripper_arm)
            self.pub_gripper_hand.publish(msg_gripper_hand)
            self.pub_gripper_motor.publish(msg_gripper_motor)

        msg.data = self.robot_main_state
        msg_team.data = self.team
        self.pub_team.publish(msg_team)
        self.pub_main_state.publish(msg)

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
