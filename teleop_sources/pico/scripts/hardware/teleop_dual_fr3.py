#!/usr/bin/env python3
import argparse

from pico_bimanual_franka_teleop.hardware import DualFr3HardwareTeleop


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=5560)
    parser.add_argument("--state-port", type=int, default=5561)
    parser.add_argument("--translation-scale", type=float, default=0.5)
    parser.add_argument("--control-rate", type=float, default=100.0)
    args = parser.parse_args()
    teleop = DualFr3HardwareTeleop(
        command_host=args.command_host,
        command_port=args.command_port,
        state_port=args.state_port,
        translation_scale=args.translation_scale,
        control_rate=args.control_rate,
    )
    teleop.run()


if __name__ == "__main__":
    main()
