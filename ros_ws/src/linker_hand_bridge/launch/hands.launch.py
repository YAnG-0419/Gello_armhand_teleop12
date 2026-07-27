"""Bring up both Linker Hand G20 drivers and the teleoperation bridge.

One command for the whole robot side of PICO hand teleoperation. The only other
process needed is the PICO host script, which runs in the Conda environment:

    python teleop_sources/pico/scripts/hardware/teleop_hands.py

Bus-to-side mapping is confirmed from each hand's own reported comm ID:
`can0` is the left hand at 0x28, `can1` is the right at 0x27.

`linker_hand_sdk` with `hand_joint:=G20` is motionless at startup, unlike
`linker_hand_advanced_g20`, which snaps to a default pose at full speed and
torque during construction. The bridge sets the requested operational speed
shortly after startup, because the vendor driver never initializes speed for
G20.

Output is disabled by default. The Compose `hand-control` service explicitly
enables it as an operator-facing hardware action.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIDES = (("left", "can0"), ("right", "can1"))


def _nodes(context):
    # Resolved eagerly so a typo like `sides:=letf` fails the launch instead
    # of silently starting zero hand drivers.
    sides = LaunchConfiguration("sides").perform(context)
    if sides not in ("left", "right", "both"):
        raise ValueError(f"sides must be left, right, or both, got {sides!r}")
    drivers = [
        Node(
            package="linker_hand_ros2_sdk",
            executable="linker_hand_sdk",
            # Both driver instances hardcode the same node name, so rename them
            # or the second one collides with the first.
            name=f"linker_hand_{side}",
            output="screen",
            parameters=[
                {
                    "hand_type": side,
                    "hand_joint": "G20",
                    "can": channel,
                    "is_touch": LaunchConfiguration("is_touch"),
                }
            ],
        )
        for side, channel in SIDES
        if sides in ("both", side)
    ]
    bridge = Node(
        package="linker_hand_bridge",
        executable="bridge",
        name="linker_hand_bridge",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "sides": LaunchConfiguration("sides"),
                "enabled": LaunchConfiguration("enabled"),
                "max_command_rate": LaunchConfiguration("max_command_rate"),
                "initial_speed": LaunchConfiguration("initial_speed"),
                "initial_torque": LaunchConfiguration("initial_torque"),
                "initial_thumb_torque": LaunchConfiguration(
                    "initial_thumb_torque"
                ),
                "abduction_invert": LaunchConfiguration("abduction_invert"),
            }
        ],
    )
    return [*drivers, bridge]


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("port", default_value="5570"),
        DeclareLaunchArgument(
            "sides",
            default_value="both",
            description="left, right, or both",
        ),
        DeclareLaunchArgument(
            "enabled",
            default_value="false",
            description="Publish to the vendor control topics.",
        ),
        DeclareLaunchArgument(
            "max_command_rate",
            default_value="1500.0",
            description=(
                "Slew limit in vendor units per second. At 1500 a full 0..255 "
                "sweep takes about 0.17 s at the 30 Hz publish rate, so the hand's "
                "own firmware speed becomes the binding constraint rather than this "
                "limiter, which is where it belongs. Drop to 200 for cautious runs "
                "after changing the mapping."
            ),
        ),
        DeclareLaunchArgument(
            "initial_speed",
            default_value="255",
            description=(
                "Joint speed 0..255 requested at startup; 0 disables. This also "
                "raises the force the fingers apply before the firmware backs off, "
                "so lower it, for example to 120, when grasping something fragile."
            ),
        ),
        DeclareLaunchArgument(
            "initial_torque",
            default_value="200",
            description=(
                "Maximum torque 0..255 for the four non-thumb fingers at "
                "startup; 0 disables (must match initial_thumb_torque's "
                "enablement). 200 is the vendor's own convention for models "
                "it does initialize. The vendor driver never initializes G20 "
                "torque on its own. Lower for fragile objects."
            ),
        ),
        DeclareLaunchArgument(
            "initial_thumb_torque",
            default_value="250",
            description=(
                "Maximum thumb torque 0..255 at startup; raised above the "
                "other fingers because the operator's button-press needs the "
                "thumb pad force (2026-07-26)."
            ),
        ),
        DeclareLaunchArgument(
            "abduction_invert",
            default_value="false",
            description=(
                "Flip the abduction polarity away from the vendor default. Set "
                "it if the operator spreads their fingers and the hand closes its "
                "finger gaps instead of opening them."
            ),
        ),
        DeclareLaunchArgument("is_touch", default_value="false"),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_nodes)])
