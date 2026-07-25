"""Launch the LinkerHand G20 bridge.

Hardware output is off by default. Bring it up in dry-run first, inspect
`/linker_hand_bridge/{side}/mapped_command`, and only then relaunch with
`enabled:=true` and a conservative slew rate.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="5570"),
        DeclareLaunchArgument(
            "sides",
            default_value="both",
            description="left, right, or both",
        ),
        DeclareLaunchArgument(
            "enabled",
            default_value="false",
            description="Publish to the vendor control topics. Off by default.",
        ),
        DeclareLaunchArgument(
            "publish_rate",
            default_value="30.0",
            description="Must not exceed the vendor driver's 30 Hz limit.",
        ),
        DeclareLaunchArgument("watchdog_timeout", default_value="0.25"),
        DeclareLaunchArgument(
            "max_command_rate",
            default_value="200.0",
            description="Slew limit in vendor range units per second.",
        ),
    ]
    node = Node(
        package="linker_hand_bridge",
        executable="bridge",
        name="linker_hand_bridge",
        output="screen",
        parameters=[
            {
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "sides": LaunchConfiguration("sides"),
                "enabled": LaunchConfiguration("enabled"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "watchdog_timeout": LaunchConfiguration("watchdog_timeout"),
                "max_command_rate": LaunchConfiguration("max_command_rate"),
            }
        ],
    )
    return LaunchDescription([*arguments, node])
