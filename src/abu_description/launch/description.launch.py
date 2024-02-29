import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    robot_base = "robot"
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("abu_description"),
            "urdf",
            f"{robot_base}.urdf.xacro",
        ]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("abu_description"), "rviz", "description.rviz"]
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    node_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "steering_controller",
            "wheel_controller",
            "left_gripper_controller",
            "right_gripper_controller",
        ],
        condition=IfCondition(LaunchConfiguration("publish_controller")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="urdf", default_value=urdf_path, description="URDF path"
            ),
            DeclareLaunchArgument(
                name="publish_joints",
                default_value="true",
                description="Launch joint_states_publisher",
            ),
            DeclareLaunchArgument(
                name="publish_controller",
                default_value="true",
                description="Launch controller_manager",
            ),
            DeclareLaunchArgument(
                name="rviz", default_value="false", description="Run rviz"
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=IfCondition(LaunchConfiguration("publish_joints")),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "robot_description": Command(
                            ["xacro ", LaunchConfiguration("urdf")]
                        ),
                    }
                ],
            ),
            node_controller,
            Node(
                package="abu_description",
                executable="swerve_drive_control.py",
                name="swerve_drive_control",
                output="screen",
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=node_controller,
                    on_exit=[node_rviz],
                )
            ),
        ]
    )
