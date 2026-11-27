"""Standalone opt-in wheel odometry for a motor-off diagnostic graph."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("journal_path", description="New private JSONL evidence file"),
        DeclareLaunchArgument("uart_port", default_value="/dev/ttyAMA0"),
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution([
            FindPackageShare("sie_ros2"), "config", "wheel_odometry_v1.yaml",
        ])),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        Node(
            package="sie_ros2", executable="wheel_odometry_node", output="screen",
            parameters=[LaunchConfiguration("config"), {
                "journal_path": ParameterValue(LaunchConfiguration("journal_path"), value_type=str),
                "uart_port": ParameterValue(LaunchConfiguration("uart_port"), value_type=str),
                "publish_tf": ParameterValue(LaunchConfiguration("publish_tf"), value_type=bool),
            }],
        ),
    ])
