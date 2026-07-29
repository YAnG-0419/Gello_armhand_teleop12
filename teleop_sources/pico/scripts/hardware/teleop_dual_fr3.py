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
            "Unified FR3 and LinkerHand teleoperation. PICO supplies arm poses; "
            "PICO optical tracking or MANUS may supply hand poses in this same "
            "operator process."
        )
    )
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--arm-source",
        required=True,
        choices=("controllers", "motion-trackers", "hand-roots"),
    )
    parser.add_argument(
        "--ui",
        default="tui",
        choices=("tui", "plain"),
        help="tui splits the terminal into status, operator, and process "
        "panes; plain keeps ordinary line output (default: tui)",
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
        choices=("none", "pico", "manus", "right-only-manus"),
        help="hand source integrated into this operator process; "
        "right-only-manus is a deprecated alias for manus with "
        "--hand-sides right (default: none)",
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
    parser.add_argument(
        "--hand-sides",
        default=None,
        choices=("left", "right", "both"),
        help="dynamic hand sides for --hand-source pico or manus "
        "(default: both)",
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

    hand_sender_factory = None
    if args.hand_source == "pico":
        if args.arm_source == "controllers":
            parser.error(
                "--hand-source pico cannot be combined with --arm-source "
                "controllers: holding a controller occupies the operator's "
                "hand, so the optical skeleton cannot describe a grasp"
            )
        from pico_bimanual_franka_teleop.hand_teleop import HandPipeline

        hand_sides = args.hand_sides or "both"
        sides = ("left", "right") if hand_sides == "both" else (hand_sides,)
        models = {
            side: (getattr(args, f"{side}_hand_model") or "g20")
            for side in sides
        }
        hand_sender_factory = partial(
            HandPipeline,
            assets_root=REPO_ROOT / "assets",
            host=args.hand_host,
            port=args.hand_port,
            rate=args.hand_rate,
            sides=sides,
            models=models,
            debug_log=args.hand_debug_log,
        )
    elif args.hand_source in ("manus", "right-only-manus"):
        if args.arm_source != "motion-trackers":
            parser.error(
                f"{args.hand_source} requires --arm-source motion-trackers"
            )
        if args.hand_source == "right-only-manus":
            if args.hand_sides is not None:
                parser.error(
                    "right-only-manus fixes the dynamic side to right; "
                    "use --hand-source manus with --hand-sides instead"
                )
            hand_sides = "right"
        else:
            hand_sides = args.hand_sides or "both"
        dynamic_sides = (
            ("left", "right") if hand_sides == "both" else (hand_sides,)
        )
        from manus_teleop import ManusHandPipeline

        def create_manus_pipeline(_xrt):
            return ManusHandPipeline(
                host=args.hand_host,
                port=args.hand_port,
                rate=args.hand_rate,
                debug_log=args.hand_debug_log,
                dynamic_sides=dynamic_sides,
                models={
                    "left": args.left_hand_model,
                    "right": args.right_hand_model or "o30i",
                },
            )

        hand_sender_factory = create_manus_pipeline

    debug_logger = None
    if args.debug_log:
        from pico_bimanual_franka_teleop.debug_log import FollowDebugLogger

        debug_logger = FollowDebugLogger(args.debug_log)
        print(f"debug log -> {args.debug_log}")

    config = load_config(args.config)

    ui = None
    if args.ui == "tui":
        if args.arm_source == "controllers":
            print("the TUI needs a keyboard-based arm source; plain output")
        else:
            from pico_bimanual_franka_teleop.tui import TeleopTui

            device = (
                config.input.motion_trackers.keyboard_device
                if args.arm_source == "motion-trackers"
                else config.input.hand_roots.keyboard_device
            )
            try:
                ui = TeleopTui(device)
            except OSError as error:
                print(f"TUI unavailable ({error}); plain output")

    try:
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
            input_type=args.arm_source,
            hand_sender_factory=hand_sender_factory,
            debug_logger=debug_logger,
            reset_invoker=invoke_reset,
            ui=ui,
        )
        # The keyboard's `q` (and Ctrl-C) surface as KeyboardInterrupt; run()'s
        # finally block has already closed hands, robot, and input by the time
        # it reaches here.
        try:
            teleop.run()
        except KeyboardInterrupt:
            pass
    finally:
        # Normally already closed by the input that adopted it; idempotent.
        # Restoring the terminal here lets a construction-failure traceback
        # reach the screen instead of the captured pipe.
        if ui is not None:
            ui.close()
    print("\nteleop stopped")


if __name__ == "__main__":
    main()
