import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    rgbd_launch_path = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    launch_rgbd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rgbd_launch_path),
        launch_arguments={
            "enable_accel": "true",
            "enable_gyro": "true",
            "pointcloud.enable": "true",
            "unite_imu_method": "1",
        }.items(),
    )

    ld.add_action(launch_rgbd)

    return ld
