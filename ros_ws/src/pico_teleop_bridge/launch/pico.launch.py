import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_bridge(context):
    path = LaunchConfiguration("config").perform(context)
    with open(path, "r", encoding="utf-8") as config_file:
        root = yaml.safe_load(config_file)
    if not isinstance(root, dict):
        raise ValueError(f"{path}: expected a mapping")
    expected_sections = {"udp", "host"}
    sections = set(root)
    if sections != expected_sections:
        missing = sorted(expected_sections - sections)
        unknown = sorted(sections - expected_sections)
        details = []
        if missing:
            details.append(f"missing sections: {', '.join(missing)}")
        if unknown:
            details.append(f"unknown sections: {', '.join(unknown)}")
        raise ValueError(f"{path}: {'; '.join(details)}")
    if not isinstance(root["udp"], dict) or not isinstance(root["host"], dict):
        raise ValueError(f"{path}: udp and host must be mappings")
    udp = root["udp"]
    required = {
        "command_host",
        "command_port",
        "state_host",
        "state_port",
        "state_timeout",
    }
    fields = set(udp)
    missing = sorted(required - fields)
    unknown = sorted(fields - required)
    if missing or unknown:
        details = []
        if missing:
            details.append(f"missing udp keys: {', '.join(missing)}")
        if unknown:
            details.append(f"unknown udp keys: {', '.join(unknown)}")
        raise ValueError(f"{path}: {'; '.join(details)}")
    required_host = {
        "translation_scale",
        "rotation_scale",
        "grip_threshold",
        "control_rate",
        "max_joint_speed",
        "xr_ready_timeout",
        "robot_state_wait_timeout",
    }
    host_fields = set(root["host"])
    missing_host = sorted(required_host - host_fields)
    unknown_host = sorted(host_fields - required_host)
    if missing_host or unknown_host:
        details = []
        if missing_host:
            details.append(f"missing host keys: {', '.join(missing_host)}")
        if unknown_host:
            details.append(f"unknown host keys: {', '.join(unknown_host)}")
        raise ValueError(f"{path}: {'; '.join(details)}")
    return [
        Node(
            package="pico_teleop_bridge",
            executable="bridge",
            output="screen",
            parameters=[
                {
                    "listen_host": udp["command_host"],
                    "command_port": udp["command_port"],
                    "feedback_host": udp["state_host"],
                    "feedback_port": udp["state_port"],
                    "state_timeout": udp["state_timeout"],
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", description="Required PICO configuration file"
            ),
            OpaqueFunction(function=launch_bridge),
        ]
    )
