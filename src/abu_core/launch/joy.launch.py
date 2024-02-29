import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    function_config_path = PathJoinSubstitution(
        [FindPackageShare("abu_core"), "config", "params.yaml"]
    )

    node_joy = Node(package="joy", executable="joy_node")
    node_joy_drive = Node(
        package="abu_core",
        executable="joy_node",
        parameters=[function_config_path],
    )
    node_joy_gripper = Node(
        package="abu_description",
        executable="gripper_control.py",
    )
    ld.add_action(node_joy)
    ld.add_action(node_joy_drive)
    ld.add_action(node_joy_gripper)

    return ld
