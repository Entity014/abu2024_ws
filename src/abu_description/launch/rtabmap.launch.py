import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    rtabmap_launch_path = PathJoinSubstitution(
        [FindPackageShare("rtabmap_launch"), "launch", "rtabmap.launch.py"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "rviz", "rtabmap.rviz"]
    )

    launch_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            "use_sim_time": "true",
            "args": "-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1 --Reg/Force3DoF true --Grid/RangeMin 0.2",
            "localization": "true",
            "rtabmap_viz": "false",
            "rviz": "false",
            "namespace": "",
            "rviz_cfg": rviz_config_path,
            # Lidar
            "subscribe_scan": "true",
            "scan_topic": "/scan_filtered",
            # odom
            "frame_id": "base_footprint",
            "visual_odometry": "false",
            "odom_topic": "/odom",
            # RGBD
            "rgb_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/depth/image_rect_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "approx_sync": "true",
            "qos": "2",
        }.items(),
    )

    ld.add_action(launch_rtabmap)
    return ld
