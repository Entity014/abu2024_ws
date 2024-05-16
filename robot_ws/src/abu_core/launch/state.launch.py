import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    node_state = Node(
        package="abu_core",
        executable="robot_state.py",
    )

    node_main_state = Node(
        package="abu_core",
        executable="robot_main_state.py",
    )
    ld.add_action(node_state)
    ld.add_action(node_main_state)

    return ld
