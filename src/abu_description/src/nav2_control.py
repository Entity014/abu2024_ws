#!/usr/bin/env python3
import rclpy
import math

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler
from rclpy import qos


class Nav2Control(Node):
    def __init__(self):
        super().__init__("nav2_control_node")
        self.sub_goal = self.create_subscription(
            Float32MultiArray,
            "target_nav2_goal",
            self.sub_goal_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_goal
        self.sub_ip = self.create_subscription(
            Float32MultiArray,
            "target_nav2_ip",
            self.sub_ip_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_ip
        self.navigator = BasicNavigator()
        self.__previous_target_goal = Float32MultiArray()
        self.__previous_target_ip = Float32MultiArray()

    def sub_goal_callback(self, goal):
        if goal.data == self.__previous_target_goal:
            return
        target = self.get_point(goal.data[0], goal.data[1], goal.data[2])
        self.navigator.goToPose(target)
        self.__previous_target_goal = goal.data

    def sub_ip_callback(self, initial_pose):
        if initial_pose.data == self.__previous_target_ip:
            return
        ip = self.get_initial_pose(
            initial_pose.data[0], initial_pose.data[1], initial_pose.data[2]
        )
        self.navigator.setInitialPose(ip)
        self.__previous_target_ip = initial_pose.data

    def get_initial_pose(self, x, y, yaw):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.position.z = 0.0
        (
            initial_pose.pose.orientation.x,
            initial_pose.pose.orientation.y,
            initial_pose.pose.orientation.z,
            initial_pose.pose.orientation.w,
        ) = quaternion_from_euler(0, 0, math.radians(yaw))

        return initial_pose

    def get_point(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        (
            goal_pose.pose.orientation.x,
            goal_pose.pose.orientation.y,
            goal_pose.pose.orientation.z,
            goal_pose.pose.orientation.w,
        ) = quaternion_from_euler(0, 0, math.radians(yaw))
        return goal_pose

    def waypoint(self, pos_waypoint):
        target_waypoint = [
            self.get_point(point[0], point[1], point[2]) for point in pos_waypoint
        ]
        self.navigator.followWaypoints(target_waypoint)


def main():
    rclpy.init()

    sub = Nav2Control()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
