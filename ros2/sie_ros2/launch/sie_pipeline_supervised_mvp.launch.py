from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [get_package_share_directory("sie_ros2"), "config", "sie_pipeline.yaml"]
    )
    config = LaunchConfiguration("config")
    project_root = LaunchConfiguration("project_root")
    execution_profile = LaunchConfiguration("execution_profile")
    base_url = LaunchConfiguration("base_url")
    confirmation = LaunchConfiguration("operator_session_confirmation")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument(
            "project_root",
            description="Absolute SIE repository root on the Pi.",
        ),
        DeclareLaunchArgument(
            "execution_profile",
            description="Absolute bounded-forward supervised profile path.",
        ),
        DeclareLaunchArgument("base_url", default_value="http://192.168.0.17"),
        DeclareLaunchArgument(
            "operator_session_confirmation",
            description="Must equal SUPERVISED_PERSON_APPROACH_SESSION.",
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
            parameters=[
                config,
                {
                    "execution_enabled": True,
                    "project_root": project_root,
                    "execution_profile": execution_profile,
                    "base_url": base_url,
                    "operator_session_confirmation": confirmation,
                },
            ],
        ),
    ])
