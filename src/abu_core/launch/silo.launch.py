import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    silo_broadcaster_node = Node(
        package="abu_core",
        executable="silo_broadcaster.py",
    )

    silo_listener_node = Node(
        package="abu_core",
        executable="silo_listener.py",
    )
    ld.add_action(silo_broadcaster_node)
    ld.add_action(silo_listener_node)

    return ld
