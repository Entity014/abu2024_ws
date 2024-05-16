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
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    use_sim_time = True

    moveit_config = MoveItConfigsBuilder(
        "abu_bot", package_name="abu_moveit_config"
    ).to_moveit_configs()
    ld = generate_demo_launch(moveit_config)

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "worlds", "abu2024_seed_blue.world"]
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("abu_core"), "config", "config.yaml"]
    )

    world_argument = DeclareLaunchArgument(
        name="world", default_value=world_path, description="Gazebo world"
    )

    node_lidar_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[config_path],
    )

    node_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "steering_controller",
            "wheel_controller",
        ],
    )
    gazebo_node = ExecuteProcess(
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
    )
    gazebo_spawn_node = Node(
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
    )

    node_lidar_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[config_path],
    )

    localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, ekf_config_path],
        remappings=[("odometry/filtered", "odom")],
    )
    swerve_control_node = Node(
        package="abu_description",
        executable="swerve_drive_control.py",
        name="swerve_drive_control",
        output="screen",
    )
    left_gripper_control_node = Node(
        package="abu_description",
        executable="gripper_control_left.py",
        name="gripper_control",
        output="screen",
    )
    right_gripper_control_node = Node(
        package="abu_description",
        executable="gripper_control_right.py",
        name="gripper_control",
        output="screen",
    )

    rtabmap_launch_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "launch", "rtabmap.launch.py"]
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
    )

    rf2o_node = Node(
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
    )

    ld.add_action(world_argument)
    ld.add_action(gazebo_node)
    ld.add_action(gazebo_spawn_node)
    ld.add_action(node_lidar_filter)
    ld.add_action(rf2o_node)
    ld.add_action(localization_node)
    ld.add_action(node_controller)
    ld.add_action(swerve_control_node)
    ld.add_action(left_gripper_control_node)
    ld.add_action(right_gripper_control_node)
    ld.add_action(rtabmap_launch)
    return ld
