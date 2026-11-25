from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [get_package_share_directory("sie_ros2"), "config", "sie_pipeline.yaml"]
    )
    config = LaunchConfiguration("config")
    enable_ld19 = LaunchConfiguration("enable_ld19")
    ld19_port = LaunchConfiguration("ld19_port")
    ld19_baudrate = LaunchConfiguration("ld19_baudrate")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="YAML file with parameters for all phase-1 SIE nodes.",
        ),
        DeclareLaunchArgument(
            "enable_ld19",
            default_value="false",
            description=(
                "Opt in to read-only LD19 LaserScan and a single provisional static TF. "
                "Requires ldlidar_stl_ros2 in the sourced ROS 2 environment; "
                "stop any independently launched LD19/TF publishers first."
            ),
        ),
        DeclareLaunchArgument(
            "ld19_port",
            default_value="/dev/ttyUSB0",
            description="LD19 CP2102 serial device, preferably a stable /dev/serial/by-id path.",
        ),
        DeclareLaunchArgument(
            "ld19_baudrate",
            default_value="230400",
            description="LD19 UART baudrate.",
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
        Node(
            condition=IfCondition(enable_ld19),
            package="ldlidar_stl_ros2",
            executable="ldlidar_stl_ros2_node",
            name="siee_ld19",
            output="screen",
            parameters=[{
                "product_name": "LDLiDAR_LD19",
                "port_name": ld19_port,
                "port_baudrate": ParameterValue(ld19_baudrate, value_type=int),
                "topic_name": "/sie/lidar/scan_raw",
                "frame_id": "ld19_sensor",
                "laser_scan_dir": True,
                "enable_angle_crop_func": False,
            }],
        ),
        Node(
            condition=IfCondition(enable_ld19),
            package="tf2_ros",
            executable="static_transform_publisher",
            name="siee_ld19_static_tf",
            output="screen",
            arguments=[
                "--x", "-0.023", "--y", "0.0", "--z", "0.212",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "base_link", "--child-frame-id", "ld19_sensor",
            ],
        ),
    ])
