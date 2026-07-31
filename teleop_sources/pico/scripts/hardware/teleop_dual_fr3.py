#!/usr/bin/env python3
import argparse

from pico_bimanual_franka_teleop.env_guard import ensure_ros_free_process

ensure_ros_free_process()

import subprocess
from pathlib import Path

from pico_bimanual_franka_teleop.config import load_config
from pico_bimanual_franka_teleop.hand_worker import HandWorker
from pico_bimanual_franka_teleop.hardware import DualFr3HardwareTeleop
from pico_bimanual_franka_teleop.xr_input import PicoSession, create_pico_input

REPO_ROOT = Path(__file__).resolve().parents[4]


def invoke_reset(side: str | None = None) -> tuple[bool, str]:
    """Call /reset_to_initial_pose (optionally one side) via the container.

    The operator process is deliberately ROS-free (env_guard), so the reset goes
    through the same `docker compose run` path the runbook documents. The
    trajectory itself can take ~20 s for large displacements, plus container
    startup; the timeout is generous because killing the call does not stop the
    controller-side trajectory anyway.
    """
    service = "/reset_to_initial_pose" + (f"/{side}" if side else "")
    completed = subprocess.run(
        [
            "docker", "compose", "run", "--rm", "tools",
            "ros2", "service", "call",
            service, "std_srvs/srv/Trigger", "{}",
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
            "Unified FR3 and LinkerHand teleoperation. PICO or VIVE Trackers "
            "supply arm poses; PICO optical tracking or MANUS may supply hand "
            "poses in this same operator process."
        )
    )
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--arm-source",
        required=True,
        choices=("controllers", "motion-trackers", "hand-roots", "vive-trackers"),
    )
    parser.add_argument(
        "--vive-config",
        default=None,
        help="VIVE Tracker configuration; required with --arm-source vive-trackers",
    )
    parser.add_argument("--control-host", default="127.0.0.1")
    parser.add_argument(
        "--control-port",
        type=int,
        default=5590,
        help="JSON-TCP operator control port; the PySide6 GUI "
        "(teleop_sources/gui) connects here (default: 5590)",
    )
    # Hand options are CLI arguments rather than YAML, matching how --arm-source is
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
        "--hand-source",
        default="none",
        choices=("none", "pico", "manus"),
        help="hand source integrated into this operator process (default: none)",
    )
    parser.add_argument(
        "--hand-debug-log",
        default=None,
        help="write live canonical landmarks, emitted hand joints, and thumb "
        "fidelity metrics to JSONL (requires a hand source)",
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
    parser.add_argument("--left-hand-model", default="g20")
    parser.add_argument(
        "--right-hand-model",
        default=None,
        help=(
            "right hand model; defaults to g20 for PICO and o30i for MANUS"
        ),
    )
    args = parser.parse_args()
    if args.hand_debug_log and args.hand_source == "none":
        parser.error("--hand-debug-log requires a hand source")
    if args.arm_source == "vive-trackers" and not args.vive_config:
        parser.error("--arm-source vive-trackers requires --vive-config")
    if args.arm_source != "vive-trackers" and args.vive_config:
        parser.error("--vive-config is only valid with --arm-source vive-trackers")

    if args.hand_source == "pico" and args.arm_source == "controllers":
        parser.error(
            "--hand-source pico cannot be combined with --arm-source "
            "controllers: holding a controller occupies the operator's "
            "hand, so the optical skeleton cannot describe a grasp"
        )

    debug_logger = None
    if args.debug_log:
        from pico_bimanual_franka_teleop.debug_log import FollowDebugLogger

        debug_logger = FollowDebugLogger(args.debug_log)
        print(f"debug log -> {args.debug_log}")

    config = load_config(args.config)

    from pico_bimanual_franka_teleop.control_server import (
        OperatorConsole,
        OperatorControlServer,
    )

    ui = OperatorConsole()
    server = OperatorControlServer((args.control_host, args.control_port), ui)
    server.start()
    print(
        f"operator control server on {args.control_host}:{args.control_port} "
        "- connect the GUI (teleop_sources/gui) to engage"
    )

    pico_session = None
    arm_source = None
    hands = None
    try:
        if args.arm_source == "vive-trackers":
            from vive_tracker_teleop import ViveTrackerInput, load_vive_config

            arm_source = ViveTrackerInput(
                load_vive_config(args.vive_config),
                ui,
            )
        else:
            pico_session = PicoSession()
            arm_source = create_pico_input(
                config.input,
                args.arm_source,
                keyboard=ui,
                xrt_client=pico_session.client,
            )
        hand_pipeline = None
        if args.hand_source == "pico":
            if pico_session is None:
                pico_session = PicoSession()
            assert pico_session.client is not None
            from pico_bimanual_franka_teleop.hand_teleop import HandPipeline

            sides = ("left", "right")
            hand_pipeline = HandPipeline(
                pico_session.client,
                assets_root=REPO_ROOT / "assets",
                host=args.hand_host,
                port=args.hand_port,
                rate=args.hand_rate,
                sides=sides,
                models={
                    side: (getattr(args, f"{side}_hand_model") or "g20")
                    for side in sides
                },
                debug_log=args.hand_debug_log,
            )
        elif args.hand_source == "manus":
            from manus_teleop import ManusHandPipeline

            hand_pipeline = ManusHandPipeline(
                host=args.hand_host,
                port=args.hand_port,
                rate=args.hand_rate,
                debug_log=args.hand_debug_log,
                dynamic_sides=("left", "right"),
                models={
                    "left": args.left_hand_model,
                    "right": args.right_hand_model or "o30i",
                },
            )
        if hand_pipeline is not None:
            hands = HandWorker(
                hand_pipeline, tick_rate=config.host.control_rate
            )

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
            arm_source=arm_source,
            operator=ui,
            hands=hands,
            debug_logger=debug_logger,
            reset_invoker=invoke_reset,
        )
        # The keyboard's `q` (and Ctrl-C) surface as KeyboardInterrupt; run()'s
        # finally block has already closed hands, robot, and input by the time
        # it reaches here.
        try:
            teleop.run()
        except KeyboardInterrupt:
            pass
    finally:
        try:
            if hands is not None:
                hands.close()
            if arm_source is not None:
                arm_source.close()
            if pico_session is not None:
                pico_session.close()
        finally:
            server.close()
    print("\nteleop stopped")


if __name__ == "__main__":
    main()
