import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "panda", package_name="panda_moveit_config"
    ).to_moveit_configs()
    ld = generate_demo_launch(moveit_config)
    gazebo_node = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_factory.so",
            "-s",
            "libgazebo_ros_init.so",
        ],
        output="screen",
    )
    gazebo_spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-entity",
            "abu_bot",
        ],
    )
    ld.add_action(gazebo_node)
    ld.add_action(gazebo_spawn_node)
    return ld
