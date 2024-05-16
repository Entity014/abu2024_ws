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
        [FindPackageShare("abu_core"), "config", "sim_locations.yaml"]
    )

    node_bt = Node(
        package="abu_core",
        executable="autonomy_node",
        parameters=[{"location_file": function_config_path}],
    )
    ld.add_action(node_bt)

    return ld
