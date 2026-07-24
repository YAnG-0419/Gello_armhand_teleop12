#!/usr/bin/env python3
import argparse

import yaml

from pico_bimanual_franka_teleop.hardware import DualFr3HardwareTeleop


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError(f"{args.config}: expected a mapping")
    required_sections = {"udp", "host"}
    missing_sections = required_sections - config.keys()
    unknown_sections = config.keys() - required_sections
    if missing_sections or unknown_sections:
        details = []
        if missing_sections:
            details.append(f"missing sections: {', '.join(sorted(missing_sections))}")
        if unknown_sections:
            details.append(f"unknown sections: {', '.join(sorted(unknown_sections))}")
        raise ValueError(f"{args.config}: {'; '.join(details)}")
    udp = config["udp"]
    host = config["host"]
    if not isinstance(udp, dict) or not isinstance(host, dict):
        raise ValueError(f"{args.config}: udp and host must be mappings")
    required_udp = {
        "command_host",
        "command_port",
        "state_host",
        "state_port",
        "state_timeout",
    }
    required_host = {
        "translation_scale",
        "rotation_scale",
        "grip_threshold",
        "control_rate",
        "max_joint_speed",
        "xr_ready_timeout",
        "robot_state_wait_timeout",
    }
    missing_udp = required_udp - udp.keys()
    missing_host = required_host - host.keys()
    unknown_udp = udp.keys() - required_udp
    unknown_host = host.keys() - required_host
    if missing_udp or missing_host or unknown_udp or unknown_host:
        details = []
        if missing_udp:
            details.append(f"missing udp keys: {', '.join(sorted(missing_udp))}")
        if missing_host:
            details.append(f"missing host keys: {', '.join(sorted(missing_host))}")
        if unknown_udp:
            details.append(f"unknown udp keys: {', '.join(sorted(unknown_udp))}")
        if unknown_host:
            details.append(f"unknown host keys: {', '.join(sorted(unknown_host))}")
        raise ValueError(f"{args.config}: {'; '.join(details)}")
    command_port = int(udp["command_port"])
    state_port = int(udp["state_port"])
    state_timeout = float(udp["state_timeout"])
    translation_scale = float(host["translation_scale"])
    rotation_scale = float(host["rotation_scale"])
    grip_threshold = float(host["grip_threshold"])
    control_rate = float(host["control_rate"])
    max_joint_speed = float(host["max_joint_speed"])
    xr_ready_timeout = float(host["xr_ready_timeout"])
    robot_state_wait_timeout = float(host["robot_state_wait_timeout"])
    if not 1 <= command_port <= 65535 or not 1 <= state_port <= 65535:
        raise ValueError(f"{args.config}: UDP ports must be between 1 and 65535")
    positive_values = {
        "state_timeout": state_timeout,
        "translation_scale": translation_scale,
        "rotation_scale": rotation_scale,
        "control_rate": control_rate,
        "max_joint_speed": max_joint_speed,
        "xr_ready_timeout": xr_ready_timeout,
        "robot_state_wait_timeout": robot_state_wait_timeout,
    }
    if any(value <= 0 for value in positive_values.values()):
        raise ValueError(
            f"{args.config}: timeout, scale, rate, and speed values must be positive"
        )
    if not 0 < grip_threshold <= 1:
        raise ValueError(f"{args.config}: grip_threshold must be in (0, 1]")
    teleop = DualFr3HardwareTeleop(
        command_host=str(udp["command_host"]),
        command_port=command_port,
        state_host=str(udp["state_host"]),
        state_port=state_port,
        state_timeout=state_timeout,
        translation_scale=translation_scale,
        rotation_scale=rotation_scale,
        grip_threshold=grip_threshold,
        control_rate=control_rate,
        max_joint_speed=max_joint_speed,
        xr_ready_timeout=xr_ready_timeout,
        robot_state_wait_timeout=robot_state_wait_timeout,
    )
    teleop.run()


if __name__ == "__main__":
    main()
