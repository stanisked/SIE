from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = LaunchConfiguration("config")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value="",
            description="Optional YAML file with parameters for all SIE nodes.",
        ),
        Node(
            package="sie_ros2",
            executable="perception_contract_node",
            name="sie_perception_contract",
            parameters=[config] if config else [],
        ),
        Node(
            package="sie_ros2",
            executable="navigation_contract_node",
            name="sie_navigation_contract",
            parameters=[config] if config else [],
        ),
        Node(
            package="sie_ros2",
            executable="supervisor_contract_node",
            name="sie_supervisor_contract",
            parameters=[config] if config else [],
        ),
    ])
