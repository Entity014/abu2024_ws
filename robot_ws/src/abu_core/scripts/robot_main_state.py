#!/usr/bin/env python3
import rclpy
import math
import cv2
import numpy as np
import time

from rclpy.node import Node
from std_msgs.msg import (
    Int8,
    String,
    Float32MultiArray,
    Bool,
    Int16MultiArray,
    Int8MultiArray,
)
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
        self.sub_color_type = self.create_subscription(
            String,
            "color/type",
            self.sub_color_type_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_color_type
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
        self.sub_silo_arr = self.create_subscription(
            Int8MultiArray,
            "silo/array",
            self.sub_silo_arr_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_silo_arr
        self.sub_imu_euler = self.create_subscription(
            Vector3,
            "imu/euler",
            self.sub_imu_euler_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_imu_euler

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
        self.width, self.height = 800, 600
        self.blue_color = (255, 250, 132)
        self.red_color = (145, 145, 255)
        self.yellow_color = (140, 249, 249)
        self.green_color = (160, 255, 160)
        self.team = "none"
        self.retry = "none"

        self.color_type = "none"
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
        self.slope_state = 0
        self.timer_state = 0
        self.start_time = None

        self.ball_silo_arr = np.zeros((5, 3), dtype=np.int8)
        self.silo_pos = Vector3()
        self.silo_arr = [
            "silo1",
            "silo2",
            "silo3",
            "silo4",
            # "silo5",
        ]
        self.silo_pos_arr_red = [
            # [0.14, 0.516],
            [0.14, 1.264],
            [0.14, 2.038],
            [0.14, 2.83],
            [0.14, 3.58],
        ]
        self.silo_pos_arr_blue = [
            [0.14, 3.58],
            [0.14, 2.83],
            [0.14, 2.038],
            [0.14, 1.264],
            # [0.14, 0.516],
        ]

        self.yaw = 90
        self.pitch = 90
        self.ranges_lidar = np.zeros(1800)
        self.silo_selected = "none"
        self.move_x = False
        self.move_y = False

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

    def sub_color_type_callback(self, msgin):
        self.color_type = msgin.data

    def sub_limit_callback(self, msgin):
        self.top_limit = msgin.linear.x
        self.bottom_limit = msgin.linear.y

    def sub_silo_pos_callback(self, msgin):
        # self.silo_pos = msgin
        # self.get_logger().info(f"{self.silo_pos}")
        self.robot_main_state = 13

    def sub_lidar_callback(self, msgin):
        # angles = np.linspace(msgin.angle_min, msgin.angle_max, len(msgin.ranges))
        # theta = [
        #     (i, theta if ((theta < -1.7 and theta > -2.605)) else np.inf)
        #     for i, theta in enumerate(angles)
        # ]
        # self.get_logger().info(f"{theta}")
        # if msgin.ranges[352] < 0.5:
        #     self.color_state = 0
        # elif msgin.ranges[1443] < 0.5:
        #     self.color_state = 1
        self.ranges_lidar = msgin.ranges

    def sub_silo_arr_callback(self, msgin):
        array_data = np.array(msgin.data, dtype=np.int8)
        self.ball_silo_arr = array_data.reshape((5, 3))
        self.ball_silo_arr = np.flipud(np.transpose(self.ball_silo_arr))
        index_ball3_arr = np.where(self.ball_silo_arr[0] == 0)
        index_ball2_arr = np.where(self.ball_silo_arr[1] != 0)
        index_ball1_arr = np.where(self.ball_silo_arr[2] == 0)
        msg_silo_type = String()
        for index in range(len(self.silo_arr)):
            if index in index_ball3_arr[0]:
                if index in index_ball2_arr[0]:
                    # if self.team == "BLUE":
                    #     msg_silo_type.data = self.silo_arr[4 - index]
                    #     self.silo_selected = self.silo_arr[4 - index]
                    # elif self.team == "RED":
                    #     msg_silo_type.data = self.silo_arr[index]
                    #     self.silo_selected = self.silo_arr[index]
                    self.silo_selected = self.silo_arr[index]
                    msg_silo_type.data = self.silo_selected
                    self.pub_silo_type.publish(msg_silo_type)
                    break
                elif index in index_ball1_arr[0]:
                    # if self.team == "BLUE":
                    #     msg_silo_type.data = self.silo_arr[4 - index]
                    #     self.silo_selected = self.silo_arr[4 - index]
                    # elif self.team == "RED":
                    #     msg_silo_type.data = self.silo_arr[index]
                    #     self.silo_selected = self.silo_arr[index]
                    self.silo_selected = self.silo_arr[index]
                    msg_silo_type.data = self.silo_selected
                    self.pub_silo_type.publish(msg_silo_type)
                    break
                elif (len(index_ball1_arr) == 5) and (index not in index_ball2_arr[0]):
                    # if self.team == "BLUE":
                    #     msg_silo_type.data = self.silo_arr[4 - index]
                    #     self.silo_selected = self.silo_arr[4 - index]
                    # elif self.team == "RED":
                    #     msg_silo_type.data = self.silo_arr[index]
                    #     self.silo_selected = self.silo_arr[index]
                    self.silo_selected = self.silo_arr[index]
                    msg_silo_type.data = self.silo_selected
                    self.pub_silo_type.publish(msg_silo_type)
                    break

    def sub_imu_euler_callback(self, msgin):
        self.yaw = msgin.x
        self.pitch = msgin.z

    def timer_callback(self):
        msg = Int8()
        msg_team = String()
        msg_cmd_vel = Twist()
        msg_gripper_arm = String()
        msg_gripper_motor = Bool()
        msg_ip = Float32MultiArray()
        msg_goal = Float32MultiArray()
        msg_gripper_hand = Int16MultiArray()
        if self.robot_state == "IDLE":
            self.terminal()
            msg_gripper_motor.data = False
            msg_gripper_arm.data = "BOTTOM"
            msg_gripper_hand.data = [0, 25]
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
                    msg_goal.data = [6.2, 0.0, 0.0]
                elif self.team == "RED":
                    msg_goal.data = [6.2, 10.9, 0.0]
                self.pub_goal.publish(msg_goal)
            elif self.robot_main_state == 1:  # ? go to point 2 ( Slope stage 2 )
                if self.team == "BLUE":
                    if self.retry == "RETRY":
                        msg_goal.data = [6.2, 3.9, -0.25]
                    else:
                        msg_goal.data = [6.2, 3.4, -0.25]
                elif self.team == "RED":
                    msg_goal.data = [6.2, 7.0, 0.25]
                self.pub_goal.publish(msg_goal)
            elif self.robot_main_state == 2:  # ? go to point 3 ( Silo )
                if self.team == "BLUE":
                    msg_goal.data = [9.5, 3.4, np.pi / 2]
                elif self.team == "RED":
                    msg_goal.data = [9.5, 7.5, -np.pi / 2]
                self.pub_goal.publish(msg_goal)
            elif self.robot_main_state == 3:  # ? go to point 4 ( Ball )
                self.move_x = False
                self.move_y = False
                if self.team == "BLUE":
                    self.rotation(0, 4)
                elif self.team == "RED":
                    self.rotation(180, 4)
                # if self.team == "BLUE":
                #     msg_goal.data = [9.5, 0.7, np.pi / 2]
                # elif self.team == "RED":
                #     msg_goal.data = [9.5, 10.2, -np.pi / 2]
                # # if self.color_type == self.team:
                # #     msg_goal.data = [0.0, 0.0, 0.0, 0.0, 0.0]
                # #     self.robot_main_state = 4
                # #     self.move = "none"
                # self.pub_goal.publish(msg_goal)
            elif self.robot_main_state == 4:
                self.move_to_pos(2.1, 2.1, 5)
                # self.move_to_pos(2.1, 2.6, 5)
            elif self.robot_main_state == 5:
                msg_gripper_motor.data = True
                msg_gripper_arm.data = "BOTTOM"
                msg_gripper_hand.data = [0, 25]
                if (
                    # self.color_type == self.team
                    self.move == "DONE"
                    or self.move == "FAIL"
                ):
                    msg_cmd_vel.linear.x = 0.0
                    self.timer_state = 0
                    self.robot_main_state = 6
                else:
                    msg_cmd_vel.linear.x = -0.8
                    if self.timer_state == 0:
                        self.start_time = time.time()
                        self.timer_state = 1
                    elif self.timer_state == 1:
                        elapsed_time = time.time() - self.start_time
                        # self.get_logger().info(f"{self.start_time, elapsed_time}")
                        if elapsed_time > 2:
                            self.robot_main_state = 6

                self.pub_cmd_vel.publish(msg_cmd_vel)
                self.pub_gripper_arm.publish(msg_gripper_arm)
                self.pub_gripper_hand.publish(msg_gripper_hand)
                self.pub_gripper_motor.publish(msg_gripper_motor)
            elif self.robot_main_state == 6:  # ? find ball
                msg_gripper_motor.data = True
                msg_gripper_arm.data = "BOTTOM"
                msg_gripper_hand.data = [0, 25]
                if self.color_type == self.team:
                    if self.move == "LEFT":
                        msg_cmd_vel.linear.x = -0.4
                        msg_cmd_vel.angular.z = 0.3
                    elif self.move == "RIGHT":
                        msg_cmd_vel.linear.x = -0.4
                        msg_cmd_vel.angular.z = -0.3
                    elif self.move == "CENTER":
                        msg_cmd_vel.linear.x = -0.4
                        msg_cmd_vel.angular.z = 0.0
                else:
                    if self.ranges_lidar[1443] < 0.6:
                        self.color_state = 1
                    elif self.ranges_lidar[352] < 0.6:
                        self.color_state = 2
                    if self.color_state == 0:
                        msg_cmd_vel.linear.y = 0.2
                    elif self.color_state == 1:
                        msg_cmd_vel.linear.y = -0.2
                    elif self.color_state == 2:
                        msg_cmd_vel.linear.x = 0.4
                        self.pub_cmd_vel.publish(msg_cmd_vel)
                        time.sleep(0.5)
                        self.color_state = 0

                if self.move == "DONE":
                    msg_cmd_vel.linear.x = 0.0
                    msg_cmd_vel.linear.y = 0.0
                    msg_cmd_vel.angular.z = 0.0
                    msg_gripper_motor.data = False
                    self.ball_type = 0
                    self.robot_main_state = 7
                elif self.move == "FAIL":
                    msg_cmd_vel.linear.x = 0.0
                    msg_cmd_vel.linear.y = 0.0
                    msg_cmd_vel.angular.z = 0.0
                    msg_gripper_motor.data = False
                    self.ball_type = 1
                    self.robot_main_state = 7

                self.pub_cmd_vel.publish(msg_cmd_vel)
                self.pub_gripper_arm.publish(msg_gripper_arm)
                self.pub_gripper_hand.publish(msg_gripper_hand)
                self.pub_gripper_motor.publish(msg_gripper_motor)
            elif self.robot_main_state == 7:  # ? pick ball
                if self.ball_type == 0:
                    if self.gripper_state == 0:
                        msg_cmd_vel.linear.x = 0.0
                        msg_gripper_arm.data = "BOTTOM"
                        msg_gripper_hand.data = [35, 20]
                        msg_gripper_motor.data = False
                        self.pub_cmd_vel.publish(msg_cmd_vel)
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
                        msg_gripper_hand.data = [35, 75]
                        self.pub_gripper_hand.publish(msg_gripper_hand)
                        self.gripper_state = 0
                        self.move = "none"
                        self.robot_main_state = 8  # TODO: Edit here
                elif self.ball_type == 1:
                    if self.gripper_state == 0:
                        msg_gripper_motor.data = False
                        msg_gripper_arm.data = "BOTTOM"
                        msg_gripper_hand.data = [35, 20]
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
                        msg_gripper_hand.data = [35, 75]
                        self.pub_gripper_hand.publish(msg_gripper_hand)
                        time.sleep(1)
                        self.gripper_state = 3
                    elif self.gripper_state == 3:
                        msg_gripper_hand.data = [0, 75]
                        self.pub_gripper_hand.publish(msg_gripper_hand)
                        time.sleep(0.6)
                        self.gripper_state = 4
                    elif self.gripper_state == 4:
                        msg_gripper_arm.data = "BOTTOM"
                        msg_gripper_hand.data = [20, 20]
                        msg_gripper_motor.data = False
                        self.pub_gripper_arm.publish(msg_gripper_arm)
                        self.pub_gripper_hand.publish(msg_gripper_hand)
                        self.pub_gripper_motor.publish(msg_gripper_motor)
                        if self.bottom_limit == 1:
                            self.gripper_state = 0
                            self.move = "none"
                        if self.team == "BLUE":
                            self.rotation(0, 6)
                        elif self.team == "RED":
                            self.rotation(180, 6)
            elif self.robot_main_state == 8:  # ? go to silo
                if self.team == "BLUE":
                    self.rotation(0, 9)
                elif self.team == "RED":
                    self.rotation(180, 9)
                # if self.team == "BLUE":
                #     msg_goal.data = [9.5, 3.2, np.pi / 2]
                # elif self.team == "RED":
                #     msg_goal.data = [9.5, 8.7, -np.pi / 2]
                # msg_gripper_motor.data = False
                # self.pub_gripper_motor.publish(msg_gripper_motor)
                # self.pub_goal.publish(msg_goal)
            elif self.robot_main_state == 9:
                if self.slope_state == 0:
                    msg_cmd_vel.linear.x = 0.8
                    if self.ranges_lidar[380] < 1.0:
                        msg_cmd_vel.linear.y = 0.4
                    elif self.ranges_lidar[1350] < 1.0:
                        msg_cmd_vel.linear.y = -0.4
                    if self.pitch >= 95:
                        self.slope_state = 1
                elif self.slope_state == 1:
                    msg_cmd_vel.linear.y = 0.0
                    msg_cmd_vel.linear.x = 0.8
                    if self.pitch <= 91:
                        self.slope_state = 2
                elif self.slope_state == 2:
                    msg_cmd_vel.linear.y = 0.0
                    msg_cmd_vel.linear.x = 0.0
                    self.robot_main_state = 10
                self.pub_cmd_vel.publish(msg_cmd_vel)
            elif self.robot_main_state == 10:
                # msgin.ranges[897] Front msgin.ranges[1350] Right [Red]  msgin.ranges[380] Left [Blue]
                # self.get_logger().info(f"{self.ranges_lidar[897]}")
                self.move_to_pos(2.1, 2.1, 11)
            elif self.robot_main_state == 11:
                self.move_x = False
                self.move_y = False
                if self.team == "BLUE":
                    self.rotation(0, 12)
                elif self.team == "RED":
                    self.rotation(180, 12)
            elif self.robot_main_state == 12:  # ? select silo
                # self.get_logger().info(f"{self.ball_silo_arr}")
                pass
            elif self.robot_main_state == 13:  # ? go to silo
                # self.get_logger().info(
                #     f"{self.silo_pos_arr_red[self.silo_arr.index(self.silo_selected)]}"
                # )
                if self.team == "BLUE":
                    self.move_to_pos(
                        self.silo_pos_arr_blue[self.silo_arr.index(self.silo_selected)][
                            0
                        ],
                        self.silo_pos_arr_blue[self.silo_arr.index(self.silo_selected)][
                            1
                        ],
                        14,
                    )
                elif self.team == "RED":
                    self.move_to_pos(
                        self.silo_pos_arr_red[self.silo_arr.index(self.silo_selected)][
                            0
                        ],
                        self.silo_pos_arr_red[self.silo_arr.index(self.silo_selected)][
                            1
                        ],
                        14,
                    )
                # if self.team == "BLUE":
                #     msg_goal.data = [self.silo_pos.x, self.silo_pos.y - 0.7, np.pi / 2]
                # elif self.team == "RED":
                #     msg_goal.data = [self.silo_pos.x, self.silo_pos.y + 0.7, -np.pi / 2]
                # self.pub_goal.publish(msg_goal)
            elif self.robot_main_state == 14:  # ? place ball to silo
                self.move_x = False
                self.move_y = False
                if self.gripper_state == 0:
                    msg_gripper_hand.data = [0, 75]
                    self.pub_gripper_hand.publish(msg_gripper_hand)
                    time.sleep(0.6)
                    self.gripper_state = 1
                elif self.gripper_state == 1:
                    msg_gripper_arm.data = "BOTTOM"
                    msg_gripper_hand.data = [20, 20]
                    self.pub_gripper_arm.publish(msg_gripper_arm)
                    self.pub_gripper_hand.publish(msg_gripper_hand)
                    if self.bottom_limit == 1:
                        self.gripper_state = 0
                        self.slope_state = 0
                        self.timer_state = 0
                        self.move = "none"
                        self.color_type = "none"
                        self.robot_main_state = 3

        elif self.robot_state == "RESET":
            self.robot_main_state = 0  # TODO: Edit here
            self.gripper_state = 0
            self.ball_type = 0
            self.color_state = 0
            self.timer_state = 0
            self.slope_state = 0
            self.yaw = 90
            self.pitch = 90
            msg_gripper_arm.data = "BOTTOM"
            msg_gripper_motor.data = False
            msg_gripper_hand.data = [2, 25]
            msg_goal.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.pub_goal.publish(msg_goal)
            self.pub_gripper_arm.publish(msg_gripper_arm)
            self.pub_gripper_hand.publish(msg_gripper_hand)
            self.pub_gripper_motor.publish(msg_gripper_motor)

        msg.data = self.robot_main_state
        msg_team.data = self.team
        self.pub_team.publish(msg_team)
        self.pub_main_state.publish(msg)

    def rotation(self, degree, next_state):
        msg_cmd_vel = Twist()
        if self.yaw > 0:
            yaw_need = degree - self.yaw
        else:
            yaw_need = -(degree + self.yaw)

        if yaw_need > 3:
            msg_cmd_vel.angular.z = -np.interp(yaw_need, [0, 180], [0.6, 2.0])
        elif yaw_need < -3:
            msg_cmd_vel.angular.z = -np.interp(yaw_need, [-180, 0], [-2.0, -0.6])
        else:
            msg_cmd_vel.angular.z = 0.0
            self.robot_main_state = next_state
        self.pub_cmd_vel.publish(msg_cmd_vel)

    def move_to_pos(self, x, y, next_state):
        msg_cmd_vel = Twist()
        x_need = abs(self.ranges_lidar[897] - x)
        y_blue_need = abs(self.ranges_lidar[380] - y)
        y_red_need = abs(self.ranges_lidar[1350] - y)
        # self.get_logger().info(f"{y_red_need}")
        if self.ranges_lidar[897] == np.inf:
            msg_cmd_vel.linear.x = 0.0
            self.move_x = True
        elif self.ranges_lidar[897] > x + 0.15:
            msg_cmd_vel.linear.x = np.interp(x_need + 0.15, [0, 1.0], [0.2, 0.6])
        elif self.ranges_lidar[897] < x:
            msg_cmd_vel.linear.x = -np.interp(x_need, [0, 1.0], [0.2, 0.6])
        else:
            msg_cmd_vel.linear.x = 0.0
            self.move_x = True

        if self.team == "BLUE":
            if self.ranges_lidar[380] > y + 0.15:
                msg_cmd_vel.linear.y = -np.interp(
                    y_blue_need + 0.15, [0, 1.0], [0.2, 0.6]
                )
            elif self.ranges_lidar[380] < y:
                msg_cmd_vel.linear.y = np.interp(y_blue_need, [0, 1.0], [0.2, 0.6])
            else:
                msg_cmd_vel.linear.y = 0.0
                self.move_y = True
        elif self.team == "RED":
            if self.ranges_lidar[1350] > y + 0.15:
                msg_cmd_vel.linear.y = np.interp(
                    y_red_need + 0.15, [0, 1.0], [0.2, 0.6]
                )
            elif self.ranges_lidar[1350] < y:
                msg_cmd_vel.linear.y = -np.interp(y_red_need, [0, 1.0], [0.2, 0.6])
            else:
                msg_cmd_vel.linear.y = 0.0
                self.move_y = True

        if self.move_x and self.move_y:
            self.robot_main_state = next_state
        self.pub_cmd_vel.publish(msg_cmd_vel)

    def terminal(self):
        name = "GUI"
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        # self.toggle_fullscreen(name)
        cv2.resizeWindow(name, self.width, self.height)
        cv2.setMouseCallback(name, self.mouse_callback)
        self.create_box_team(frame)
        self.create_box_retry(frame)
        self.toggle_color_team()
        self.toggle_color_retry()
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
