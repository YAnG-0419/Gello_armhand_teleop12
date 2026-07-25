#!/usr/bin/env python3
import argparse
from functools import partial
from pathlib import Path

from pico_bimanual_franka_teleop.config import load_config
from pico_bimanual_franka_teleop.hardware import DualFr3HardwareTeleop

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Dual FR3 teleoperation from PICO. Add --hands to also forward the "
            "optical hand skeletons to hand_retarget_service.py, which must be "
            "running. Retargeting happens in that separate process because doing "
            "it here holds the GIL long enough to wreck the 100 Hz arm loop."
        )
    )
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--input",
        required=True,
        choices=("controllers", "motion-trackers"),
    )
    # Hand options are CLI arguments rather than YAML, matching how --input is
    # handled: what is being driven is an explicit choice per run, and this keeps
    # existing configuration files valid.
    parser.add_argument(
        "--hands",
        action="store_true",
        help="also forward optical hand skeletons to hand_retarget_service.py",
    )
    parser.add_argument("--hand-host", default="127.0.0.1")
    parser.add_argument(
        "--hand-port",
        type=int,
        default=5571,
        help="where hand_retarget_service.py listens (default: 5571)",
    )
    parser.add_argument(
        "--hand-rate",
        type=float,
        default=30.0,
        help="skeleton datagrams per second per side (default: 30)",
    )
    parser.add_argument(
        "--hand-sides",
        default="both",
        choices=("left", "right", "both"),
    )
    args = parser.parse_args()

    hand_sender_factory = None
    if args.hands:
        if args.input != "motion-trackers":
            parser.error(
                "--hands requires --input motion-trackers: holding a controller "
                "occupies the operator's hand, so the optical skeleton cannot "
                "describe a grasp"
            )
        from pico_bimanual_franka_teleop.hand_teleop import HandSkeletonForwarder

        sides = ("left", "right") if args.hand_sides == "both" else (args.hand_sides,)
        hand_sender_factory = partial(
            HandSkeletonForwarder,
            host=args.hand_host,
            port=args.hand_port,
            rate=args.hand_rate,
            sides=sides,
        )

    config = load_config(args.config)
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
        input_config=config.input,
        input_type=args.input,
        hand_sender_factory=hand_sender_factory,
    )
    teleop.run()


if __name__ == "__main__":
    main()
