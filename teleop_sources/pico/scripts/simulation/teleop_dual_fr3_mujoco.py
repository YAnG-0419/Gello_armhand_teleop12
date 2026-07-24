#!/usr/bin/env python3
import argparse

import yaml

from pico_bimanual_franka_teleop.simulation import DualFr3Simulation


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock-xr", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError(f"{args.config}: expected a mapping")
    if set(config) != {"udp", "host"}:
        raise ValueError(f"{args.config}: must contain exactly: udp, host")
    if not isinstance(config["udp"], dict) or not isinstance(config["host"], dict):
        raise ValueError(f"{args.config}: udp and host must be mappings")
    required_udp = {
        "command_host",
        "command_port",
        "state_host",
        "state_port",
        "state_timeout",
    }
    if set(config["udp"]) != required_udp:
        raise ValueError(
            f"{args.config}: udp must contain exactly: "
            f"{', '.join(sorted(required_udp))}"
        )
    host = config["host"]
    required = {
        "translation_scale",
        "rotation_scale",
        "grip_threshold",
        "control_rate",
        "max_joint_speed",
        "xr_ready_timeout",
        "robot_state_wait_timeout",
    }
    if set(host) != required:
        raise ValueError(
            f"{args.config}: host must contain exactly: {', '.join(sorted(required))}"
        )
    simulation = DualFr3Simulation(
        mock_xr=args.mock_xr,
        translation_scale=float(host["translation_scale"]),
        rotation_scale=float(host["rotation_scale"]),
        grip_threshold=float(host["grip_threshold"]),
        control_rate=float(host["control_rate"]),
        max_joint_speed=float(host["max_joint_speed"]),
        xr_ready_timeout=float(host["xr_ready_timeout"]),
    )
    simulation.run(duration=args.duration, headless=args.headless)


if __name__ == "__main__":
    main()
