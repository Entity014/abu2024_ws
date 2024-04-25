#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type = LaunchConfiguration("channel_type", default="serial")
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    serial_baudrate = LaunchConfiguration(
        "serial_baudrate", default="256000"
    )  # for A3 is 256000
    frame_id = LaunchConfiguration("frame_id", default="laser")
    inverted = LaunchConfiguration("inverted", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")
    scan_mode = LaunchConfiguration("scan_mode", default="Sensitivity")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "channel_type",
                default_value=channel_type,
                description="Specifying channel type of lidar",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value=serial_port,
                description="Specifying usb port to connected lidar",
            ),
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value=serial_baudrate,
                description="Specifying usb port baudrate to connected lidar",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value=frame_id,
                description="Specifying frame_id of lidar",
            ),
            DeclareLaunchArgument(
                "inverted",
                default_value=inverted,
                description="Specifying whether or not to invert scan data",
            ),
            DeclareLaunchArgument(
                "angle_compensate",
                default_value=angle_compensate,
                description="Specifying whether or not to enable angle_compensate of scan data",
            ),
            DeclareLaunchArgument(
                "scan_mode",
                default_value=scan_mode,
                description="Specifying scan mode of lidar",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Full path to the ROS2 parameters file to use for sllidar node",
            ),
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                parameters=[
                    {
                        "channel_type": channel_type,
                        "serial_port": serial_port,
                        "serial_baudrate": serial_baudrate,
                        "frame_id": frame_id,
                        "inverted": inverted,
                        "angle_compensate": angle_compensate,
                        "scan_mode": scan_mode,
                    },
                    params_file,
                ],
                output="screen",
                respawn=True,
            ),
        ]
    )
