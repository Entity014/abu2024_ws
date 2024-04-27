import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    localization_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_core"), "launch", "localization.launch.py"]
    )
    microros_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_core"), "launch", "microros.launch.py"]
    )
    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_core"), "launch", "navigation.launch.py"]
    )
    state_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_core"), "launch", "state.launch.py"]
    )
    detection_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_core"), "launch", "detection.launch.py"]
    )

    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path),
    )
    launch_microros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(microros_launch_path),
    )
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
    )
    launch_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(state_launch_path),
    )
    launch_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(detection_launch_path),
    )

    ld.add_action(launch_microros)
    # ld.add_action(launch_localization)
    # ld.add_action(launch_navigation)
    ld.add_action(launch_state)
    ld.add_action(launch_detection)

    os.system(
        "gnome-terminal -e 'bash -c \"ros2 launch abu_core localization.launch.py\"'"
    )
    os.system(
        "gnome-terminal -e 'bash -c \"ros2 launch abu_core navigation.launch.py\"'"
    )
    return ld
