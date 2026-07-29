"""Launch the model-profiled LinkerHand safety bridge.

Hardware output is always on (the dry-run mode was removed 2026-07-29);
`/linker_hand_bridge/{side}/mapped_command` still mirrors every command
for inspection.
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
        DeclareLaunchArgument("left_model", default_value="g20"),
        DeclareLaunchArgument("right_model", default_value="g20"),
        DeclareLaunchArgument(
            "publish_rate",
            default_value="30.0",
            description="Must not exceed the vendor driver's 30 Hz limit.",
        ),
        DeclareLaunchArgument("watchdog_timeout", default_value="0.25"),
        DeclareLaunchArgument(
            "max_command_rate",
            default_value="200.0",
            description=(
                "Requested slew limit per second in each model profile's "
                "canonical units; the profile may impose a lower cap."
            ),
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
                "left_model": LaunchConfiguration("left_model"),
                "right_model": LaunchConfiguration("right_model"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "watchdog_timeout": LaunchConfiguration("watchdog_timeout"),
                "max_command_rate": LaunchConfiguration("max_command_rate"),
            }
        ],
    )
    return LaunchDescription([*arguments, node])
