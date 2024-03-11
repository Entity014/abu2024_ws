import os
from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "config", "slam.yaml"]
    )

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "config", "navigation.yaml"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "rviz", "slam.rviz"]
    )

    slam_param_name = "slam_params_file"

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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch_path),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("sim"),
                    "params_file": nav2_config_path,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch_path),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("sim"),
                    slam_param_name: slam_config_path,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
                condition=IfCondition(LaunchConfiguration("rviz")),
                parameters=[{"use_sim_time": LaunchConfiguration("sim")}],
            ),
        ]
    )
