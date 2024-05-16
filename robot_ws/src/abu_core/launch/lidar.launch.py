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
        [FindPackageShare("abu_core"), "config", "config.yaml"]
    )

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare("sllidar_ros2"), "launch", "sllidar_a3_launch.py"]
    )

    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={"params_file": function_config_path}.items(),
    )

    node_lidar_filter = Node(
        package="abu_core",
        executable="lidar_filter.py",
        parameters=[function_config_path],
    )

    # node_lidar_filter = Node(
    #     package="laser_filters",
    #     executable="scan_to_scan_filter_chain",
    #     parameters=[function_config_path],
    # )

    ld.add_action(launch_lidar)
    ld.add_action(node_lidar_filter)

    return ld
