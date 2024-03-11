import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = True

    config_path = PathJoinSubstitution(
        [FindPackageShare("abu_core"), "config", "config.yaml"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "launch", "description.launch.py"]
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "worlds", "abu2024_seed_blue.world"]
    )

    node_lidar_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[config_path],
    )

    rtabmap_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "launch", "rtabmap.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="world", default_value=world_path, description="Gazebo world"
            ),
            DeclareLaunchArgument(
                name="rtabmap", default_value="true", description="Run rtabmap"
            ),
            ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_factory.so",
                    "-s",
                    "libgazebo_ros_init.so",
                    LaunchConfiguration("world"),
                ],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="urdf_spawner",
                output="screen",
                arguments=[
                    "-topic",
                    "/robot_description",
                    "-entity",
                    "abu_bot",
                    "-x",
                    "11.55",
                    "-y",
                    "0.4",
                    "-z",
                    "0.2",
                    "-Y",
                    "1.57",
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch_path),
                launch_arguments={
                    "use_sim_time": str(use_sim_time),
                    "publish_joints": "false",
                    "publish_controller": "true",
                    "rviz": "false",
                }.items(),
            ),
            node_lidar_filter,
            Node(
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry",
                output="screen",
                parameters=[
                    {
                        "laser_scan_topic": "/scan_filtered",
                        "odom_topic": "/odom_rf2o",
                        "publish_tf": False,
                        "base_frame_id": "base_footprint",
                        "odom_frame_id": "odom",
                        "init_pose_from_topic": "",
                        "freq": 20.0,
                    }
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, ekf_config_path],
                remappings=[("odometry/filtered", "odom")],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch_path),
                condition=IfCondition(LaunchConfiguration("rtabmap")),
            ),
        ]
    )
