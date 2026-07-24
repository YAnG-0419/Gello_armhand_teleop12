#!/usr/bin/env python3
import argparse

from pico_bimanual_franka_teleop.config import load_config
from pico_bimanual_franka_teleop.hardware import DualFr3HardwareTeleop


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = load_config(args.config, require_tracker_serials=True)
    teleop = DualFr3HardwareTeleop(
        command_host=config.udp.command_host,
        command_port=config.udp.command_port,
        state_host=config.udp.state_host,
        state_port=config.udp.state_port,
        state_timeout=config.udp.state_timeout,
        translation_scale=config.host.translation_scale,
        rotation_scale=config.host.rotation_scale,
        control_rate=config.host.control_rate,
        max_joint_speed=config.host.max_joint_speed,
        robot_state_wait_timeout=config.host.robot_state_wait_timeout,
        tracker_serials=config.input.serials,
        tracker_to_control=config.input.tracker_to_control,
        tracker_ready_timeout=config.input.ready_timeout,
        tracker_stale_timeout=config.input.stale_timeout,
        keyboard_device=config.input.keyboard_device,
    )
    teleop.run()


if __name__ == "__main__":
    main()
