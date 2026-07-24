from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("listen_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("command_port", default_value="5560"),
        DeclareLaunchArgument("feedback_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("feedback_port", default_value="5561"),
    ]
    bridge = Node(
        package="pico_teleop_bridge",
        executable="bridge",
        output="screen",
        parameters=[
            {
                "listen_host": LaunchConfiguration("listen_host"),
                "command_port": ParameterValue(
                    LaunchConfiguration("command_port"), value_type=int
                ),
                "feedback_host": LaunchConfiguration("feedback_host"),
                "feedback_port": ParameterValue(
                    LaunchConfiguration("feedback_port"), value_type=int
                ),
            }
        ],
    )
    return LaunchDescription(arguments + [bridge])
