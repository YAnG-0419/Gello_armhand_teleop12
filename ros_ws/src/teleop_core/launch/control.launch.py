from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enabled = DeclareLaunchArgument("enabled", default_value="false")
    gateway = Node(
        package="teleop_core",
        executable="safety_gateway",
        output="screen",
        parameters=[
            {
                "enabled": ParameterValue(
                    LaunchConfiguration("enabled"), value_type=bool
                )
            }
        ],
    )
    splitter = Node(
        package="teleop_core",
        executable="joint_splitter",
        output="screen",
    )
    return LaunchDescription([enabled, gateway, splitter])
