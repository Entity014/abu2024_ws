import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    node_ball_detection = Node(
        package="abu_core",
        executable="ball_model.py",
    )
    node_ball_silo = Node(
        package="abu_core",
        executable="ball_silo.py",
    )

    ld.add_action(node_ball_detection)
    ld.add_action(node_ball_silo)

    return ld
