import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "config", "ekf.yaml"]
    )

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_core"), "launch", "lidar.launch.py"]
    )
    rgbd_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_core"), "launch", "rgbd.launch.py"]
    )
    rtabmap_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "launch", "rtabmap.launch.py"]
    )
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "launch", "description.launch.py"]
    )

    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
    )
    launch_rgbd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rgbd_launch_path),
    )
    launch_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
    )
    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            "use_sim_time": "true",
            "publish_joints": "true",
            "publish_controller": "false",
            "rviz": "false",
        }.items(),
    )

    node_rf20 = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom_rf2o",
                "publish_tf": False,
                "base_frame_id": "base_footprint",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 10.0,
            }
        ],
    )

    node_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path],
        remappings=[("odometry/filtered", "odom")],
    )

    node_tf2 = Node(
        package="tf2_ros",
        namespace="scan_to_map",
        executable="static_transform_publisher",
        arguments=["0.232", "0", "0", "0", "0", "0", "odom", "scan_filtered"],
    )
    node_imu_connect = Node(
        package="abu_core",
        executable="imu.py",
    )

    node_imu = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[{"use_mag": False}],
    )

    ld.add_action(launch_description)
    ld.add_action(launch_lidar)
    # ld.add_action(node_tf2)
    ld.add_action(launch_rgbd)
    # ld.add_action(node_imu_connect)
    # ld.add_action(node_imu)
    # ld.add_action(launch_rtabmap)
    ld.add_action(node_rf20)
    ld.add_action(node_ekf)

    return ld
