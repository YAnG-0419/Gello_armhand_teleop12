#!/usr/bin/env python3
import argparse

from pico_bimanual_franka_teleop.env_guard import ensure_ros_free_process

ensure_ros_free_process()

from functools import partial
from pathlib import Path

from pico_bimanual_franka_teleop.config import load_config
from pico_bimanual_franka_teleop.hardware import DualFr3HardwareTeleop

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Dual FR3 teleoperation from PICO. Add --hands to also drive the "
            "Linker Hands from optical hand tracking, retargeted inline in this "
            "process and sent to linker_hand_bridge."
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
        "--debug-log",
        default=None,
        help="write one JSONL row per control tick, capturing tracker pose, "
        "mapped target, commanded and measured state, for offline analysis of "
        "following quality",
    )
    parser.add_argument(
        "--hands",
        action="store_true",
        help="also retarget optical hand tracking to the Linker Hands",
    )
    parser.add_argument("--hand-host", default="127.0.0.1")
    parser.add_argument(
        "--hand-port",
        type=int,
        default=5570,
        help="where linker_hand_bridge listens (default: 5570)",
    )
    parser.add_argument(
        "--hand-rate",
        type=float,
        default=30.0,
        help="hand commands per second per side; the vendor driver drops "
        "commands above about 100 Hz (default: 30)",
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
        from pico_bimanual_franka_teleop.hand_teleop import HandPipeline

        sides = ("left", "right") if args.hand_sides == "both" else (args.hand_sides,)
        hand_sender_factory = partial(
            HandPipeline,
            assets_dir=Path(__file__).resolve().parents[4] / "assets" / "linkerhand_l20",
            host=args.hand_host,
            port=args.hand_port,
            rate=args.hand_rate,
            sides=sides,
        )

    debug_logger = None
    if args.debug_log:
        from pico_bimanual_franka_teleop.debug_log import FollowDebugLogger

        debug_logger = FollowDebugLogger(args.debug_log)
        print(f"debug log -> {args.debug_log}")

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
        debug_logger=debug_logger,
    )
    teleop.run()


if __name__ == "__main__":
    main()
