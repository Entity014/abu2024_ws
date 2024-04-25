#!/usr/bin/env python3
import rclpy
import math

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from rclpy import qos

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


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
        self.pub_state = self.create_publisher(
            Bool, "goal/state", qos_profile=qos.qos_profile_system_default
        )
        self.pub_ip = self.create_publisher(
            PoseWithCovarianceStamped,
            "initialpose",
            qos_profile=qos.qos_profile_system_default,
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.navigator = BasicNavigator()
        self.cli = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.goal_status = None
        self.__previous_target_goal = Float32MultiArray()
        self.__previous_target_ip = Float32MultiArray()
        self.__previous_state = Bool()

    def timer_callback(self):
        state = self.get_state()
        if self.__previous_state != state:
            msg = Bool()
            msg.data = state
            self.pub_state.publish(msg)
            self.goal_status = None
            self.__previous_state = state

    def sub_goal_callback(self, goal):
        if goal.data == self.__previous_target_goal:
            return
        if len(goal.data) == 3:
            target = self.get_point(goal.data[0], goal.data[1], goal.data[2])
            self.cli.wait_for_server()
            self.send_goal_future = self.cli.send_goal_async(target)
            self.send_goal_future.add_done_callback(self.goal_callback)
            self.__previous_target_goal = goal.data

    def sub_ip_callback(self, initial_pose):
        if initial_pose.data == self.__previous_target_ip:
            return
        if len(initial_pose.data) == 3:
            ip = self.get_initial_pose(
                initial_pose.data[0], initial_pose.data[1], initial_pose.data[2]
            )
            self.pub_ip.publish(ip)
            self.__previous_target_ip = initial_pose.data

    def get_initial_pose(self, x, y, yaw):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0
        (
            initial_pose.pose.pose.orientation.x,
            initial_pose.pose.pose.orientation.y,
            initial_pose.pose.pose.orientation.z,
            initial_pose.pose.pose.orientation.w,
        ) = quaternion_from_euler(0, 0, math.radians(yaw))

        return initial_pose

    def get_point(self, x, y, yaw):
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = "map"
        goal_pose.pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.pose.position.x = x
        goal_pose.pose.pose.position.y = y
        goal_pose.pose.pose.position.z = 0.0
        (
            goal_pose.pose.pose.orientation.x,
            goal_pose.pose.pose.orientation.y,
            goal_pose.pose.pose.orientation.z,
            goal_pose.pose.pose.orientation.w,
        ) = quaternion_from_euler(0, 0, math.radians(yaw))
        return goal_pose

    def waypoint(self, pos_waypoint):
        target_waypoint = [
            self.get_point(point[0], point[1], point[2]) for point in pos_waypoint
        ]
        self.navigator.followWaypoints(target_waypoint)

    def goal_callback(self, future):
        res = future.result()
        if res is None or not res.accepted:
            return
        future = res.get_result_async()
        future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        # If there is a result, we consider navigation completed and save the
        # result code to be checked in the `update()` method.
        self.goal_status = future.result().status

    def get_state(self):
        if self.goal_status is not None:
            if self.goal_status == GoalStatus.STATUS_SUCCEEDED:
                return True
            else:
                return False
        return False


def main():
    rclpy.init()

    sub = Nav2Control()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
