#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Float32MultiArray, Bool
from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import abubot_left
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import QoSProfile


class GripperControl(Node):
    def __init__(self):
        super().__init__("gripper_control")
        self._callback_group = ReentrantCallbackGroup()
        self._moveit2_arm = MoveIt2(
            node=self,
            joint_names=abubot_left.joint_names(),
            base_link_name=abubot_left.base_link_name(),
            end_effector_name=abubot_left.end_effector_name(),
            group_name=abubot_left.MOVE_GROUP_ARM,
            execute_via_moveit=True,
            callback_group=self._callback_group,
        )
        self._moveit2_hand = GripperInterface(
            node=self,
            gripper_joint_names=abubot_left.gripper_joint_names(),
            open_gripper_joint_positions=abubot_left.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=abubot_left.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=abubot_left.MOVE_GROUP_GRIPPER,
            callback_group=self._callback_group,
        )

        self._moveit2_arm.max_velocity = 1.0
        self._moveit2_arm.max_acceleration = 1.0

        self.__previous_target_arm_joint = Float32MultiArray()
        self.__previous_target_hand_joint = Bool()
        self.create_subscription(
            msg_type=Float32MultiArray,
            topic="/target_pose_left_arm",
            callback=self.target_joint_arm_callback,
            qos_profile=qos.qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            msg_type=Bool,
            topic="/target_left_hand",
            callback=self.target_hand_callback,
            qos_profile=qos.qos_profile_sensor_data,
            callback_group=self._callback_group,
        )

        self.get_logger().info("Initialization successful.")

    def target_joint_arm_callback(self, msg: Float32MultiArray):

        if msg.data == self.__previous_target_arm_joint:
            return
        self._moveit2_arm.move_to_configuration(msg.data)
        self.__previous_target_arm_joint = msg.data

    def target_hand_callback(self, msg: Bool):
        if msg.data == self.__previous_target_hand_joint:
            return
        if msg.data:
            self._moveit2_hand.close()
        elif not msg.data:
            self._moveit2_hand.open()
        self.__previous_target_hand_joint = msg.data


def main(args=None):
    rclpy.init(args=args)

    target_follower = GripperControl()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(target_follower)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
