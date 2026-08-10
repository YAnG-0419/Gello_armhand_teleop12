#!/usr/bin/env python3
import argparse
import sys

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
            "Unified FR3 and LinkerHand teleoperation. GELLO supplies incremental "
            "arm joints by default; legacy pose sources remain available. PICO "
            "optical tracking or MANUS may supply hand poses independently."
        )
    )
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--arm-source",
        required=True,
        choices=(
            "gello",
            "controllers",
            "motion-trackers",
            "hand-roots",
            "vive-trackers",
        ),
    )
    parser.add_argument(
        "--gello-config",
        default=str(REPO_ROOT / "config" / "gello.yaml"),
        help="dual GELLO identities, directions, and incremental-control limits",
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
        choices=("none", "pico", "manus", "wuji"),
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
    parser.add_argument(
        "--left-hand-model",
        default=None,
        help=(
            "legacy combined hand/method name; defaults to g20 for PICO and "
            "o30i+sharpa for MANUS. For MANUS use o30i_casadi (or omit) for "
            "dual O30i; plain o30i selects the right-only landmark path"
        ),
    )
    parser.add_argument(
        "--right-hand-model",
        default=None,
        help=(
            "legacy combined hand/method name; defaults to g20 for PICO and "
            "o30i+sharpa for MANUS"
        ),
    )
    parser.add_argument(
        "--right-hand-strategy-config",
        type=Path,
        default=None,
        help=(
            "enable the powderweighing right-hand MANUS strategy using this "
            "pose/trigger JSON; the left hand remains on its configured method"
        ),
    )
    parser.add_argument(
        "--wuji-sides",
        choices=("left", "right", "both"),
        default="both",
        help="Wuji hardware sides to drive (only with --hand-source wuji)",
    )
    for side in ("left", "right"):
        parser.add_argument(
            f"--wuji-{side}-model",
            choices=("wuji_hand", "wuji_hand_2"),
            default="wuji_hand_2",
            help=f"physical {side} Wuji hand model",
        )
        parser.add_argument(
            f"--wuji-{side}-address",
            default="",
            help=f"{side} Wuji Hand 2 SDK address (IP:PORT)",
        )
        parser.add_argument(
            f"--wuji-{side}-serial",
            default="",
            help=f"{side} original Wuji Hand USB serial",
        )
    parser.add_argument("--wuji-kp", type=float, default=3.0)
    parser.add_argument("--wuji-kd", type=float, default=0.1)
    parser.add_argument(
        "--wuji-current-limit",
        type=float,
        default=1.5,
        help="Wuji Hand 2 per-joint current limit in amps",
    )
    args = parser.parse_args()
    if args.hand_debug_log and args.hand_source == "none":
        parser.error("--hand-debug-log requires a hand source")
    if args.arm_source == "vive-trackers" and not args.vive_config:
        parser.error("--arm-source vive-trackers requires --vive-config")
    if args.arm_source != "vive-trackers" and args.vive_config:
        parser.error("--vive-config is only valid with --arm-source vive-trackers")
    if args.right_hand_strategy_config and args.hand_source != "manus":
        parser.error(
            "--right-hand-strategy-config requires --hand-source manus"
        )
    if args.hand_source != "wuji" and any(
        (
            args.wuji_left_address,
            args.wuji_right_address,
            args.wuji_left_serial,
            args.wuji_right_serial,
        )
    ):
        parser.error("--wuji-*-address/serial requires --hand-source wuji")

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
        if args.arm_source == "gello":
            from pico_bimanual_franka_teleop.gello_input import (
                DualGelloJointInput,
                load_gello_config,
            )

            arm_source = DualGelloJointInput(
                load_gello_config(args.gello_config),
                ui,
            )
        elif args.arm_source == "vive-trackers":
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
            from manus_teleop.pipeline import (
                DEFAULT_HANDS,
                DEFAULT_METHODS,
                split_legacy_model,
            )

            task_config = None
            if args.right_hand_strategy_config is not None:
                import sys

                # Task packages live at the repository root and are optional,
                # so they are not part of the editable core-source installs.
                sys.path.insert(0, str(REPO_ROOT))
                from powderweighing.strategy import load_config as load_task_config

                try:
                    task_config = load_task_config(
                        args.right_hand_strategy_config
                    )
                except (OSError, KeyError, TypeError, ValueError) as error:
                    parser.error(
                        f"invalid right-hand strategy config: {error}"
                    )

            # Dual O30i + sharpa is the MANUS default. Plain "o30i" still means
            # the right-only landmark retargeter, so only decode CLI overrides.
            hands = dict(DEFAULT_HANDS)
            methods = dict(DEFAULT_METHODS)
            for side, legacy in (
                ("left", args.left_hand_model),
                ("right", args.right_hand_model),
            ):
                if legacy is not None:
                    hands[side], methods[side] = split_legacy_model(legacy)

            hand_pipeline = ManusHandPipeline(
                host=args.hand_host,
                port=args.hand_port,
                rate=args.hand_rate,
                debug_log=args.hand_debug_log,
                dynamic_sides=("left", "right"),
                hands=hands,
                methods=methods,
            )
            if task_config is not None:
                from powderweighing.strategy import install_on_manus_pipeline

                try:
                    install_on_manus_pipeline(hand_pipeline, task_config)
                except (TypeError, ValueError):
                    hand_pipeline.close()
                    raise
                print(
                    "right hand strategy -> powderweighing "
                    f"({args.right_hand_strategy_config})"
                )
        elif args.hand_source == "wuji":
            sys.path.insert(0, str(REPO_ROOT / "integrations" / "wuji"))
            from pipeline import WujiHandPipeline

            sides = (
                ("left", "right")
                if args.wuji_sides == "both"
                else (args.wuji_sides,)
            )
            models = {
                side: getattr(args, f"wuji_{side}_model") for side in sides
            }
            addresses = {
                side: getattr(args, f"wuji_{side}_address") for side in sides
            }
            serials = {
                side: getattr(args, f"wuji_{side}_serial") for side in sides
            }
            hand_pipeline = WujiHandPipeline(
                sides=sides,
                models=models,
                addresses=addresses,
                serials=serials,
                rate=args.hand_rate,
                kp=args.wuji_kp,
                kd=args.wuji_kd,
                current_limit=args.wuji_current_limit,
                debug_log=args.hand_debug_log,
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
