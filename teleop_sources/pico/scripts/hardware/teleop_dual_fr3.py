#!/usr/bin/env python3
import argparse

from pico_bimanual_franka_teleop.env_guard import ensure_ros_free_process

ensure_ros_free_process()

import subprocess
from functools import partial
from pathlib import Path

from pico_bimanual_franka_teleop.config import load_config
from pico_bimanual_franka_teleop.hardware import DualFr3HardwareTeleop

REPO_ROOT = Path(__file__).resolve().parents[4]


def invoke_reset() -> tuple[bool, str]:
    """Call /reset_to_initial_pose through the container, blocking until done.

    The operator process is deliberately ROS-free (env_guard), so the reset goes
    through the same `docker compose run` path the runbook documents. The
    trajectory itself can take ~20 s for large displacements, plus container
    startup; the timeout is generous because killing the call does not stop the
    controller-side trajectory anyway.
    """
    completed = subprocess.run(
        [
            "docker", "compose", "run", "--rm", "tools",
            "ros2", "service", "call",
            "/reset_to_initial_pose", "std_srvs/srv/Trigger", "{}",
        ],
        cwd=REPO_ROOT / "docker",
        capture_output=True,
        text=True,
        timeout=120.0,
    )
    output = (completed.stdout + completed.stderr).strip()
    succeeded = completed.returncode == 0 and "success=True" in completed.stdout
    return succeeded, output[-400:]


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
        choices=("controllers", "motion-trackers", "hand-roots"),
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
    parser.add_argument(
        "--hand-debug-log",
        default=None,
        help="write live canonical landmarks, emitted hand joints, and thumb "
        "fidelity metrics to JSONL (requires --hands)",
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
    if args.hand_debug_log and not args.hands:
        parser.error("--hand-debug-log requires --hands")

    hand_sender_factory = None
    if args.hands:
        if args.input == "controllers":
            parser.error(
                "--hands cannot be combined with --input controllers: holding "
                "a controller occupies the operator's hand, so the optical "
                "skeleton cannot describe a grasp"
            )
        from pico_bimanual_franka_teleop.hand_teleop import HandPipeline

        sides = ("left", "right") if args.hand_sides == "both" else (args.hand_sides,)
        hand_sender_factory = partial(
            HandPipeline,
            assets_dir=REPO_ROOT / "assets" / "linkerhand_l20",
            host=args.hand_host,
            port=args.hand_port,
            rate=args.hand_rate,
            sides=sides,
            debug_log=args.hand_debug_log,
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
        reset_invoker=invoke_reset,
    )
    # The keyboard's `q` (and Ctrl-C) surface as KeyboardInterrupt; run()'s
    # finally block has already closed hands, robot, and input by the time it
    # reaches here.
    try:
        teleop.run()
    except KeyboardInterrupt:
        print("\nteleop stopped")


if __name__ == "__main__":
    main()
