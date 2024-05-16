import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

MAP_NAME = "map"  # change to the name of your own map here


def generate_launch_description():
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "rviz", "navigation.rviz"]
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "maps", f"{MAP_NAME}.yaml"]
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "config", "navigation.yaml"]
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("sim")}],
    )

    nav2_control_node = Node(
        package="abu_description",
        executable="nav2_control.py",
        name="nav2_control",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="sim",
                default_value="true",
                description="Enable use_sime_time to true",
            ),
            DeclareLaunchArgument(
                name="rviz", default_value="true", description="Run rviz"
            ),
            DeclareLaunchArgument(
                name="map",
                default_value=default_map_path,
                description="Navigation map path",
            ),
            rviz_node,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": LaunchConfiguration("sim"),
                    "params_file": nav2_config_path,
                }.items(),
            ),
            nav2_control_node,
        ]
    )
