from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [get_package_share_directory("sie_ros2"), "config", "sie_pipeline.yaml"]
    )
    config = LaunchConfiguration("config")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="YAML file with parameters for all phase-1 SIE nodes.",
        ),
        Node(
            package="sie_ros2",
            executable="perception_contract_node",
            name="sie_perception_contract",
            parameters=[config],
        ),
        Node(
            package="sie_ros2",
            executable="navigation_contract_node",
            name="sie_navigation_contract",
            parameters=[config],
        ),
        Node(
            package="sie_ros2",
            executable="supervisor_contract_node",
            name="sie_supervisor_contract",
            parameters=[config],
        ),
    ])
