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
            "unite_imu_method": "1",
        }.items(),
    )

    imu_calibration_node = Node(
        package="abu_core",
        executable="imu_calibration.py",
    )
    imu_euler_node = Node(
        package="abu_core",
        executable="imu_euler.py",
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[
            {
                "use_mag": False,
                # "gain": 0.0,
                "fixed_frame": "imu_link",
            }
        ],
        # remappings=[("/imu/data_raw", "/camera/imu")],
    )

    ld.add_action(launch_rgbd)
    ld.add_action(imu_calibration_node)
    ld.add_action(imu_filter_node)
    ld.add_action(imu_euler_node)

    return ld
