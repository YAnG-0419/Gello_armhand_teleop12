"""Bring up model-specific Linker Hand drivers and the safety bridge.

This launch supports a left G20 and a right G20 or O30i. PICO optical hand
tracking remains a G20 source; the right O30i source is the MANUS pipeline.

The G20 bus-to-side mapping is confirmed from each hand's own reported comm ID:
`can0` is the left hand at 0x28. The current O30i uses HOP CAN-FD through the
vendor `libcanbus` USB adapter, device 0/channel 0, with right-hand request ID
0x01 and response ID 0x401. A transparent SocketCAN adapter may use `can1`.

`linker_hand_sdk` with `hand_joint:=G20` is motionless at startup, unlike
`linker_hand_advanced_g20`, which snaps to a default pose at full speed and
torque during construction. The bridge sets the requested operational speed
shortly after startup, because the vendor driver never initializes speed for
G20.

Output is disabled by default. The Compose `hand-control` service explicitly
enables it as an operator-facing hardware action.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from linker_hand_bridge.profiles import create_hand_profile

SIDES = (("left", "can0"), ("right", "can1"))


def _nodes(context):
    # Resolved eagerly so a typo like `sides:=letf` fails the launch instead
    # of silently starting zero hand drivers.
    sides = LaunchConfiguration("sides").perform(context)
    if sides not in ("left", "right", "both"):
        raise ValueError(f"sides must be left, right, or both, got {sides!r}")
    models = {
        side: LaunchConfiguration(f"{side}_model").perform(context).strip().lower()
        for side, _ in SIDES
    }
    if any(not model for model in models.values()):
        raise ValueError("left_model and right_model must be non-empty")
    calibration_verified = (
        LaunchConfiguration("o30_calibration_verified")
        .perform(context)
        .strip()
        .lower()
    )
    if calibration_verified not in {"true", "false"}:
        raise ValueError("o30_calibration_verified must be true or false")

    def calibration_vector(argument):
        text = LaunchConfiguration(argument).perform(context)
        try:
            values = [float(value.strip()) for value in text.split(",")]
        except ValueError as error:
            raise ValueError(f"{argument} must be a comma-separated numeric vector") from error
        if len(values) != 20 or any(
            not math.isfinite(value) or value < 0.0 or value > 255.0
            for value in values
        ):
            raise ValueError(f"{argument} must contain 20 values in [0, 255]")
        return values

    tick_at_lower = calibration_vector("o30_tick_at_lower")
    tick_at_upper = calibration_vector("o30_tick_at_upper")
    for side, _ in SIDES:
        if sides in ("both", side):
            # Fail before any driver process starts if the safety bridge has no
            # contract for the requested hardware.
            create_hand_profile(models[side], side=side)
    selected_o30 = [
        side
        for side, _ in SIDES
        if sides in ("both", side) and models[side] == "o30i"
    ]
    o30_transport = (
        LaunchConfiguration("o30_transport").perform(context).strip().lower()
    )
    if selected_o30 and o30_transport not in {"socketcan", "libcanbus"}:
        raise ValueError("o30_transport must be socketcan or libcanbus")
    for argument in ("o30_canfd_device", "o30_canfd_channel"):
        try:
            index = int(LaunchConfiguration(argument).perform(context))
        except ValueError as error:
            raise ValueError(f"{argument} must be an integer") from error
        if selected_o30 and index < 0:
            raise ValueError(f"{argument} must not be negative")
    if selected_o30 and calibration_verified != "true":
        raise ValueError(
            "real O30i output requires o30_calibration_verified:=true"
        )
    if selected_o30:
        if any(
            lower == upper
            for lower, upper in zip(tick_at_lower, tick_at_upper, strict=True)
        ):
            raise ValueError(
                "O30i lower/upper calibration ticks must differ for every joint"
            )
        for argument in ("o30_command_timeout", "o30_state_timeout"):
            try:
                timeout = float(LaunchConfiguration(argument).perform(context))
            except ValueError as error:
                raise ValueError(f"{argument} must be numeric") from error
            if not math.isfinite(timeout) or timeout <= 0.0:
                raise ValueError(f"{argument} must be positive and finite")

    drivers = []
    for side, channel in SIDES:
        if sides not in ("both", side):
            continue
        if models[side] == "o30i":
            drivers.append(
                Node(
                    package="linker_hand_ros2_sdk",
                    executable="linker_hand_o30i",
                    name=f"linker_hand_{side}_o30i",
                    output="screen",
                    parameters=[
                        {
                            "hand_type": side,
                            "transport": LaunchConfiguration(
                                "o30_transport"
                            ),
                            "can": channel,
                            "frame_id": 1 if side == "right" else 2,
                            "canfd_device": LaunchConfiguration(
                                "o30_canfd_device"
                            ),
                            "canfd_channel": LaunchConfiguration(
                                "o30_canfd_channel"
                            ),
                            "libcanbus_path": LaunchConfiguration(
                                "o30_libcanbus_path"
                            ),
                            "output_enabled": True,
                            "calibration_verified": True,
                            "tick_at_lower": tick_at_lower,
                            "tick_at_upper": tick_at_upper,
                            "command_timeout": LaunchConfiguration(
                                "o30_command_timeout"
                            ),
                            "state_timeout": LaunchConfiguration(
                                "o30_state_timeout"
                            ),
                        }
                    ],
                )
            )
            continue
        drivers.append(
            Node(
                package="linker_hand_ros2_sdk",
                executable="linker_hand_sdk",
                # Both driver instances hardcode the same node name, so rename
                # them or the second one collides with the first.
                name=f"linker_hand_{side}",
                output="screen",
                remappings=[
                    (
                        "/cb_hand_setting_cmd",
                        f"/cb_{side}_hand_setting_cmd",
                    )
                ],
                parameters=[
                    {
                        "hand_type": side,
                        "hand_joint": models[side].upper(),
                        "can": channel,
                        "is_touch": LaunchConfiguration("is_touch"),
                    }
                ],
            )
        )
    bridge = Node(
        package="linker_hand_bridge",
        executable="bridge",
        name="linker_hand_bridge",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "sides": LaunchConfiguration("sides"),
                "left_model": models["left"],
                "right_model": models["right"],
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
            "left_model",
            default_value="g20",
            description="Registered bridge and vendor-driver model for the left hand.",
        ),
        DeclareLaunchArgument(
            "right_model",
            default_value="g20",
            description="Registered bridge and vendor-driver model for the right hand.",
        ),
        DeclareLaunchArgument(
            "o30_calibration_verified",
            default_value="false",
            description=(
                "Required true before real O30i output. Confirms that the "
                "configured radians-to-tick mapping was reviewed."
            ),
        ),
        DeclareLaunchArgument(
            "o30_transport",
            default_value="libcanbus",
            description=(
                "O30i adapter transport: libcanbus for the connected metal "
                "USB-CANFD adapter, or socketcan for a native CAN interface."
            ),
        ),
        DeclareLaunchArgument("o30_canfd_device", default_value="0"),
        DeclareLaunchArgument("o30_canfd_channel", default_value="0"),
        DeclareLaunchArgument(
            "o30_libcanbus_path",
            default_value="",
            description=(
                "Optional libcanbus.so override; empty uses the packaged runtime."
            ),
        ),
        DeclareLaunchArgument(
            "o30_tick_at_lower",
            default_value=",".join(["0"] * 20),
            description="Twenty O30i ticks corresponding to the URDF lower limits.",
        ),
        DeclareLaunchArgument(
            "o30_tick_at_upper",
            default_value=",".join(["255"] * 20),
            description="Twenty O30i ticks corresponding to the URDF upper limits.",
        ),
        DeclareLaunchArgument(
            "o30_command_timeout",
            default_value="0.25",
            description=(
                "Seconds without a command before the O30i driver disables all "
                "joints terminally."
            ),
        ),
        DeclareLaunchArgument(
            "o30_state_timeout",
            default_value="0.5",
            description=(
                "Seconds without valid O30i position feedback before the driver "
                "disables all joints terminally."
            ),
        ),
        DeclareLaunchArgument(
            "max_command_rate",
            default_value="1500.0",
            description=(
                "Requested per-second slew limit in each profile's canonical "
                "units. Profiles impose their own upper cap: G20 uses ticks and "
                "O30i uses URDF radians."
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
